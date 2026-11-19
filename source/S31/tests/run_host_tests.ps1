$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$main = Join-Path $root "main"
$gcc = (Get-Command gcc -ErrorAction Stop).Source

& $gcc -std=c99 -Wall -Wextra -Werror -O0 -I $main `
    (Join-Path $PSScriptRoot "test_core.c") `
    (Join-Path $main "diagnostics/c30d_monitor.c") `
    (Join-Path $main "control/line_follower.c") `
    (Join-Path $main "protocol/robot_protocol.c") `
    -o (Join-Path $PSScriptRoot "test_core_v393.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $gcc -std=c99 -Wall -Wextra -Werror -O0 -I $main `
    (Join-Path $PSScriptRoot "test_agent_policy.c") `
    (Join-Path $main "agent/agent_policy.c") `
    -o (Join-Path $PSScriptRoot "test_agent_policy.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $gcc -std=c99 -Wall -Wextra -Werror -O0 -I $main `
    (Join-Path $PSScriptRoot "test_device_auth.c") `
    (Join-Path $main "network/device_auth.c") `
    -o (Join-Path $PSScriptRoot "test_device_auth.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& (Join-Path $PSScriptRoot "test_core_v393.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $PSScriptRoot "test_agent_policy.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $PSScriptRoot "test_device_auth.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $gcc -std=c99 -Wall -Wextra -Werror -O0 -I $main `
    (Join-Path $PSScriptRoot "test_selftest_policy.c") `
    (Join-Path $main "diagnostics/selftest_policy.c") `
    -o (Join-Path $PSScriptRoot "test_selftest_policy.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $PSScriptRoot "test_selftest_policy.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# robot_bus: host build must NOT link robot_bus_port.c; test_bus.c provides
# the counting port mock and the controllable clock.
& $gcc -std=c99 -Wall -Wextra -Werror -O0 -I $main `
    (Join-Path $PSScriptRoot "test_bus.c") `
    (Join-Path $main "bus/robot_bus.c") `
    -o (Join-Path $PSScriptRoot "test_bus.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $PSScriptRoot "test_bus.exe")
& $gcc -std=c99 -Wall -Wextra -Werror -O0 -DAGENT_RUNTIME_HOST_TEST -I $main `
    (Join-Path $PSScriptRoot "test_agent_runtime.c") `
    (Join-Path $main "agent/agent_runtime.c") `
    (Join-Path $main "protocol/robot_protocol.c") `
    (Join-Path $main "bus/robot_bus.c") `
    -o (Join-Path $PSScriptRoot "test_agent_runtime.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $PSScriptRoot "test_agent_runtime.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

exit $LASTEXITCODE
