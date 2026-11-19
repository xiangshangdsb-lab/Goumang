"""Record S31/C30D patrol runs and train a small look-ahead model."""

import argparse
import getpass
import json
import os
import sys
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


DARK_FEATURES = 24
MIN_HORIZON_US = 200_000
MAX_HORIZON_US = 350_000
MAX_GAP_US = 300_000


def record(args):
    token = os.environ.get("GOUMANG_TOKEN") or getpass.getpass("S31 控制码: ")
    request = Request(args.url.rstrip("/") + "/api/line-sample",
                      headers={"X-TD-Token": token})
    deadline = time.monotonic() + args.seconds
    last = None
    count = 0
    with args.out.open("x", encoding="utf-8") as file:
        while time.monotonic() < deadline:
            started = time.monotonic()
            try:
                with urlopen(request, timeout=1.0) as response:
                    sample = json.load(response)
                if sample.get("v") != 1:
                    raise ValueError("Unsupported sample format")
                key = sample["frame_us"], sample["valid"]
                if key != last:
                    file.write(json.dumps(sample, separators=(",", ":")) + "\n")
                    count += int(sample["valid"])
                    last = key
            except HTTPError as exc:
                if exc.code in (401, 404):
                    raise SystemExit("控制码错误或设备固件未启用采样接口") from exc
                file.write('{"valid":false}\n')
                last = None
            except (URLError, TimeoutError):
                file.write('{"valid":false}\n')
                last = None
            time.sleep(max(0.0, 0.125 - (time.monotonic() - started)))
    print(f"Recorded {count} valid frames in {args.out}")


def read_pairs(paths):
    from bisect import bisect_left

    inputs = []
    targets = []
    for path in paths:
        segment = []

        def collect():
            ccd_times = [r["ccd_us"] for r in segment]
            for i, sample in enumerate(segment):
                future = bisect_left(ccd_times, sample["frame_us"] + MIN_HORIZON_US, i + 1)
                if future == len(segment):
                    break
                label = segment[future]
                if label["ccd_us"] - sample["frame_us"] > MAX_HORIZON_US:
                    continue
                inputs.append([x / 100.0 for x in sample["dark"]] +
                              [(sample["ccd"] - 64) / 64.0, sample["speed"] / 500.0])
                targets.append([(label["ccd"] - 64) / 64.0])

        with path.open(encoding="utf-8") as file:
            for row, line in enumerate(file, 1):
                sample = json.loads(line)
                if not sample.get("valid"):
                    collect()
                    segment.clear()
                    continue
                dark = sample["dark"]
                if (sample.get("v") != 1 or len(dark) != DARK_FEATURES or
                    any(not 0 <= x <= 100 for x in dark) or
                    not 0 <= sample["ccd"] < 128 or not 20 <= sample["speed"] <= 500 or
                    not 0 <= sample["frame_us"] - sample["ccd_us"] <= 161_000):
                    raise ValueError(f"{path}:{row}: invalid sample")
                if segment and (not 0 < sample["frame_us"] - segment[-1]["frame_us"] <= MAX_GAP_US
                                or sample["ccd_us"] < segment[-1]["ccd_us"]):
                    collect()
                    segment.clear()
                segment.append(sample)
        collect()
    return inputs, targets


def fit(np, x, y, epochs):
    rng = np.random.default_rng(3100)
    dims = (DARK_FEATURES + 2, 16, 8, 1)
    weights = [rng.normal(0, (2 / n) ** 0.5, (n, m)).astype(np.float32)
               for n, m in zip(dims, dims[1:])]
    biases = [np.zeros(m, dtype=np.float32) for m in dims[1:]]
    # Adam updates. Each batch only uses one shuffled traversal of the route samples.
    params = [*weights, *biases]
    momentums = [np.zeros_like(p) for p in params]
    variances = [np.zeros_like(p) for p in params]
    step = 0
    for _ in range(epochs):
        order = rng.permutation(len(x))
        for start in range(0, len(order), 64):
            idx = order[start:start + 64]
            a0 = x[idx]
            a1 = np.maximum(a0 @ weights[0] + biases[0], 0)
            a2 = np.maximum(a1 @ weights[1] + biases[1], 0)
            pred = a2 @ weights[2] + biases[2]
            d3 = (pred - y[idx]) * (2 / len(idx))
            d2 = (d3 @ weights[2].T) * (a2 > 0)
            d1 = (d2 @ weights[1].T) * (a1 > 0)
            derivatives = [a0.T @ d1, a1.T @ d2, a2.T @ d3,
                           d1.sum(axis=0), d2.sum(axis=0), d3.sum(axis=0)]
            step += 1
            for p, m, v, grad in zip(params, momentums, variances, derivatives):
                m[:] = 0.9 * m + 0.1 * grad
                v[:] = 0.999 * v + 0.001 * grad * grad
                p -= 0.003 * (m / (1 - 0.9 ** step)) / (np.sqrt(v / (1 - 0.999 ** step)) + 1e-8)
    return weights, biases


def predict(np, x, weights, biases):
    for w, b in zip(weights[:2], biases[:2]):
        x = np.maximum(x @ w + b, 0)
    return np.clip(x @ weights[2] + biases[2], -1, 1)


def export(weights, biases, path):
    content = ["#pragma once", "", "/* Offline-trained S31 line model. */",
               "#define TD_LINE_MODEL_AVAILABLE 1", ""]

    def literal(value):
        return f"{float(value):.9e}f"

    def array(name, data):
        values = list(map(literal, data))
        content.append(f"static const float {name}[{len(values)}] = {{")
        for i in range(0, len(values), 6):
            content.append("    " + ", ".join(values[i:i + 6]) + ",")
        content.extend(("};", ""))

    for i, w in enumerate(weights, start=1):
        array(f"td_line_w{i}", w.T.ravel())
    array("td_line_b1", biases[0])
    array("td_line_b2", biases[1])
    content.append(f"static const float td_line_b3 = {literal(biases[2][0])};")
    path.write_text("\n".join(content) + "\n", encoding="utf-8")


def train(args):
    import numpy as np

    if {p.resolve() for p in args.train} & {p.resolve() for p in args.test}:
        raise SystemExit("训练和验证需要不同的实车记录")
    train_x, train_y = read_pairs(args.train)
    test_x, test_y = read_pairs(args.test)
    if len(train_x) < 32 or len(test_x) < 16:
        raise SystemExit(f"有效样本不足（训练 {len(train_x)}，验证 {len(test_x)}）")
    x = np.asarray(train_x, dtype=np.float32)
    y = np.asarray(train_y, dtype=np.float32)
    tx = np.asarray(test_x, dtype=np.float32)
    ty = np.asarray(test_y, dtype=np.float32)
    weights, biases = fit(np, x, y, args.epochs)
    pred = predict(np, tx, weights, biases)
    baseline_error = float(np.mean(np.abs(tx[:, -2:-1] - ty)) * 100)
    model_error = float(np.mean(np.abs(pred - ty)) * 100)
    p95 = float(np.percentile(np.abs(pred - ty), 95) * 100)
    if not np.isfinite([baseline_error, model_error, p95]).all():
        raise SystemExit("训练结果包含非有限数值")
    export(weights, biases, args.out)
    print(json.dumps({"train_pairs": len(x), "test_pairs": len(tx),
                      "ccd_baseline_mae": baseline_error, "model_mae": model_error,
                      "model_error_p95": p95, "weights": str(args.out)}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(required=True)
    collect = commands.add_parser("record")
    collect.add_argument("url")
    collect.add_argument("out", type=Path)
    collect.add_argument("--seconds", type=int, default=60)
    collect.set_defaults(run=record)
    training = commands.add_parser("train")
    training.add_argument("--train", type=Path, nargs="+", required=True)
    training.add_argument("--test", type=Path, nargs="+", required=True)
    training.add_argument("--out", type=Path, required=True)
    training.add_argument("--epochs", type=int, default=100)
    training.set_defaults(run=train)
    args = parser.parse_args()
    try:
        args.run(args)
    except (OSError, ValueError, KeyError) as exc:
        print(exc, file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
