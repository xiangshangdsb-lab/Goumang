$ErrorActionPreference = 'Stop'

$cameraPath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\perception\camera_app.c'))
$camera = Get-Content -LiteralPath $cameraPath -Raw
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

$getter = [regex]::Match(
    $camera,
    'esp_err_t camera_app_get_bmp\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$fresh = [regex]::Match(
    $camera,
    'static bool camera_snapshot_is_fresh\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check ($camera -match '#define\s+CAMERA_FRAME_STALE_MS\s+1000u') `
    'camera frames expire after one second'
Check $fresh.Success `
    'camera freshness helper exists'
Check $getter.Success `
    'BMP endpoint implementation is present'

if ($fresh.Success) {
    $body = $fresh.Groups['body'].Value
    Check ($body -match 's_streaming') `
        'freshness requires an active stream'
    Check ($body -match 's_last_frame_ms') `
        'freshness checks the last frame timestamp'
    Check ($body -match 'CAMERA_FRAME_STALE_MS') `
        'freshness applies the stale timeout'
}

if ($getter.Success) {
    Check ($getter.Groups['body'].Value -match
        'camera_snapshot_is_fresh\s*\(\s*\)') `
        'BMP endpoint rejects black or stale snapshots'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
