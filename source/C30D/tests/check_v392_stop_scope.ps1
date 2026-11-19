$ErrorActionPreference = 'Stop'

$sourcePath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\APP\control\s31_link.c'))
$source = Get-Content -LiteralPath $sourcePath -Raw
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

$stopMatch = [regex]::Match(
    $source,
    'case S31_CMD_STOP_ALL:(?<body>.*?)(?=\bcase S31_CMD_)',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$estopMatch = [regex]::Match(
    $source,
    'case S31_CMD_ESTOP:(?<body>.*?)break;',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check $stopMatch.Success 'STOP_ALL handler is present'
Check $estopMatch.Success 'ESTOP handler is present'

if ($stopMatch.Success) {
    Check ($source -match 'static void stop_chassis_now\s*\(\s*void\s*\)') `
        'a chassis-only stop helper exists'
    Check ($stopMatch.Groups['body'].Value -match 'stop_chassis_now\s*\(\s*\)') `
        'STOP_ALL uses the chassis-only stop helper'
    Check ($stopMatch.Groups['body'].Value -notmatch 'stop_motors_now\s*\(\s*\)') `
        'STOP_ALL does not stop the gimbal'
}

if ($estopMatch.Success) {
    Check ($estopMatch.Groups['body'].Value -match 'stop_motors_now\s*\(\s*\)') `
        'ESTOP still stops chassis and gimbal'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
