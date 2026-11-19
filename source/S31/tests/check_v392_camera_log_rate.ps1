$ErrorActionPreference = 'Stop'

$videoPath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\perception\app_video.c'))
$video = Get-Content -LiteralPath $videoPath -Raw
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

$receive = [regex]::Match(
    $video,
    'static inline esp_err_t video_receive_video_frame\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$throttle = [regex]::Match(
    $video,
    'static void log_video_error_throttled\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check ($video -match '#define\s+VIDEO_ERROR_LOG_INTERVAL_MS\s+5000u') `
    'video error logging has a five second window'
Check $throttle.Success `
    'video error log throttling helper exists'
Check $receive.Success `
    'video receive implementation is present'

if ($receive.Success) {
    Check ($receive.Groups['body'].Value -match
        'log_video_error_throttled\s*\(') `
        'receive failures use the throttled logger'
    Check ($receive.Groups['body'].Value -notmatch
        'ESP_LOGE\s*\(\s*TAG,\s*"failed to receive video frame"') `
        'receive failures do not bypass the throttled logger'
}

if ($throttle.Success) {
    $body = $throttle.Groups['body'].Value
    Check ($body -match 'VIDEO_ERROR_LOG_INTERVAL_MS') `
        'throttled logger enforces the interval'
    Check ($body -match 'suppressed') `
        'throttled logger reports suppressed failures'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
