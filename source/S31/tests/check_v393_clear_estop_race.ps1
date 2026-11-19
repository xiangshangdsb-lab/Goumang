$ErrorActionPreference = 'Stop'

$controlPath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\control\control.c'))
$control = Get-Content -LiteralPath $controlPath -Raw
$checks = 0
$failures = 0

function Check([bool]$condition, [string]$name) {
    $script:checks++
    if ($condition) {
        Write-Host "PASS  $name"
    } else {
        $script:failures++
        Write-Host "FAIL  $name"
    }
}

$clear = [regex]::Match(
    $control,
    'bool control_clear_estop\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$estop = [regex]::Match(
    $control,
    'bool control_estop\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check ($control -match 's_motion\.generation') `
    'E-STOP generation counter is tracked'
Check $clear.Success `
    'E-STOP clear implementation is present'
Check $estop.Success `
    'E-STOP implementation is present'

if ($estop.Success) {
    Check ($estop.Groups['body'].Value -match
        'enter_mode_locked\(TD_MODE_ESTOP\)') `
        'a new E-STOP advances the generation counter'
}

if ($clear.Success) {
    $body = $clear.Groups['body'].Value
    $sendIndex = $body.IndexOf('robot_link_clear_estop')
    $captureIndex = $body.IndexOf('s_motion.generation')
    Check ($captureIndex -ge 0 -and
           $captureIndex -lt $sendIndex) `
        'clear captures the generation before transmitting'
    Check ($body -match
        '(?s)generation\s*!=\s*s_motion\.generation.*?robot_link_emergency_stop') `
        'a racing E-STOP is reasserted after clear'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
