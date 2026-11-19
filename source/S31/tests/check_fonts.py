"""Check generated LVGL cmap coverage against all local runtime text sources."""
from pathlib import Path
import re

main = Path(__file__).resolve().parents[1] / "main"
fonts = main / "interface/panel/fonts"
required = set(range(32, 127))
for source in main.rglob("*"):
    if source.suffix in (".c", ".h", ".cpp") and not source.name.startswith("td_font_"):
        required.update(ord(c) for c in source.read_text(encoding="utf-8")
                        if ord(c) > 127 and not c.isspace())


def font_data(path):
    text = path.read_text(encoding="utf-8")
    arrays = {}
    for name, body in re.findall(r"static const [^{;]+? (unicode_list_\d+)\[\] = \{(.*?)\};", text, re.S):
        arrays[name] = [int(n, 16) for n in re.findall(r"0x([0-9a-f]+)", body)]
    cmap = text.split("static const lv_font_fmt_txt_cmap_t cmaps[]", 1)[1].split("};", 1)[0]
    mapping = {}
    for block in re.findall(r"\{([^{}]+)\}", cmap):
        start = int(re.search(r"range_start = (\d+)", block)[1])
        length = int(re.search(r"range_length = (\d+)", block)[1])
        glyph = int(re.search(r"glyph_id_start = (\d+)", block)[1])
        name = re.search(r"unicode_list = (\w+)", block)[1]
        offsets = arrays[name] if name != "NULL" else range(length)
        mapping.update((start + offset, glyph + index) for index, offset in enumerate(offsets))
    descriptors = []
    for row in re.findall(r"\{(\.bitmap_index[^{}]+)\}", text):
        descriptors.append({key: int(value) for key, value in re.findall(r"\.(\w+) = (-?\d+)", row)})
    bitmap = text.split("glyph_bitmap[] = {", 1)[1].split("};", 1)[0]
    bitmap = re.sub(r"/\*.*?\*/", "", bitmap, flags=re.S)
    pixels = bytes(int(n, 16) for n in re.findall(r"0x([0-9a-f]+)", bitmap))
    return mapping, descriptors, pixels


for size in (16, 24):
    mapping, descriptors, pixels = font_data(fonts / f"td_font_{size}.c")
    missing = required - set(mapping)
    assert not missing, "Missing: " + "".join(chr(c) for c in sorted(missing))
    for char in "句芒配置热点标定90°网络设置语音测试播放中播报失败":
        glyph = descriptors[mapping[ord(char)]]
        assert glyph["box_w"] > 0 and glyph["box_h"] > 0
    print(f"font {size}px: {len(required)} required code points covered; missing=0")

if __name__ == "__main__" and len(__import__("sys").argv) > 1:
    from PIL import Image
    image = Image.new("RGB", (520, 240), (13, 28, 21))
    for y, size, line in [(16, 24, "句芒"), (65, 16, "配置热点  标定90°  网络设置"),
                           (105, 16, "语音测试  播放中  播报失败"),
                           (145, 16, "距离：--  电池：12285 mV"),
                           (185, 16, "移动巡检机器人 · ESP32-S31")]:
        mapping, descriptors, pixels = font_data(fonts / f"td_font_{size}.c")
        x = 20
        for char in line:
            glyph = descriptors[mapping[ord(char)]]
            width, height = glyph["box_w"], glyph["box_h"]
            count = width * height
            if count:
                start = glyph["bitmap_index"]
                raw = pixels[start:start + (count + 1) // 2]
                alpha = bytes((raw[i // 2] >> (4 if i % 2 == 0 else 0) & 15) * 17 for i in range(count))
                mask = Image.frombytes("L", (width, height), alpha)
                image.paste((232, 247, 238), (x + glyph["ofs_x"], y + size - glyph["ofs_y"] - height), mask)
            x += (glyph["adv_w"] + 8) // 16
    image.resize((1040, 480)).save(__import__("sys").argv[1])
