$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$app = Join-Path $root "APP"
$gcc = (Get-Command gcc -ErrorAction Stop).Source

& $gcc -std=c99 -Wall -Wextra -Werror -O0 -I $app `
    (Join-Path $PSScriptRoot "test_line_fusion.c") `
    (Join-Path $app "line/line_fusion.c") `
    -o (Join-Path $PSScriptRoot "test_line_fusion.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $gcc -std=c99 -Wall -Wextra -Werror -O0 -I $app `
    (Join-Path $PSScriptRoot "test_bluetooth.c") `
    (Join-Path $app "bluetooth/bluetooth_control.c") `
    -o (Join-Path $PSScriptRoot "test_bluetooth_v393.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $gcc -std=c99 -Wall -Wextra -Werror -O0 -I $app `
    (Join-Path $PSScriptRoot "test_pan_param.c") `
    -o (Join-Path $PSScriptRoot "test_pan_param.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& (Join-Path $PSScriptRoot "test_line_fusion.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $PSScriptRoot "test_bluetooth_v393.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $PSScriptRoot "test_pan_param.exe")
exit $LASTEXITCODE

