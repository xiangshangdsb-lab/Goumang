$ErrorActionPreference = 'Stop'

$cameraPath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\perception\camera_app.c'))
$camera = Get-Content -LiteralPath $cameraPath -Raw
$uiPath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\interface\panel\ui.c'))
$ui = Get-Content -LiteralPath $uiPath -Raw
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

$cleanup = [regex]::Match(
    $camera,
    'static void camera_release_resources\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$start = [regex]::Match(
    $camera,
    'esp_err_t camera_app_start\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check $cleanup.Success `
    'camera resource release helper exists'
Check $start.Success `
    'camera startup implementation is present'
Check ($ui -match 'void ui_detach_camera\s*\(\s*void\s*\)') `
    'UI can detach a deleted camera canvas'

if ($cleanup.Success) {
    $body = $cleanup.Groups['body'].Value
    Check ($body -match 'app_video_stream_task_stop') `
        'cleanup stops the video task'
    Check ($body -match 'app_video_wait_video_stop') `
        'cleanup waits for the video task'
    Check ($body -match 'app_video_close') `
        'cleanup closes the video device'
    Check ($body -match 'ui_detach_camera') `
        'cleanup detaches the canvas from the UI'
    Check ($body -match 'lv_obj_delete') `
        'cleanup deletes the LVGL canvas'
    Check ($body -match 'heap_caps_free') `
        'cleanup frees camera buffers'
    Check ($body -match 'ppa_unregister_client') `
        'cleanup unregisters the PPA client'
    Check ($body -match 'esp_video_deinit_with_flags') `
        'cleanup deinitializes the video device layer'
}

if ($start.Success) {
    $gotoCount = [regex]::Matches(
        $start.Groups['body'].Value, 'goto\s+cleanup\s*;').Count
    Check ($gotoCount -ge 7) `
        "startup routes failure paths through cleanup ($gotoCount)"
}

Check ($video -match 'vSemaphoreDelete\s*\(\s*app_camera_video\.video_stop_sem\s*\)') `
    'video close releases its stop semaphore'

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
