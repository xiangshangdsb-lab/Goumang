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

$freeFrame = [regex]::Match(
    $video,
    'static inline esp_err_t video_free_video_frame\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check $freeFrame.Success `
    'video buffer release implementation is present'

if ($freeFrame.Success) {
    $body = $freeFrame.Groups['body'].Value
    Check ($body -match 'log_video_error_throttled\s*\(') `
        'QBUF failures use the throttled logger'
    Check ($body -notmatch 'ESP_LOGE\s*\(') `
        'QBUF failures do not bypass throttling'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
