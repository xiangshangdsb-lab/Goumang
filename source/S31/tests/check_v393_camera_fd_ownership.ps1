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

$setBufs = [regex]::Match(
    $video,
    'esp_err_t app_video_set_bufs\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check $setBufs.Success `
    'video buffer setup implementation is present'

if ($setBufs.Success) {
    $body = $setBufs.Groups['body'].Value
    Check ($body -notmatch 'close\s*\(\s*video_fd\s*\)') `
        'buffer setup leaves fd ownership with the caller'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
