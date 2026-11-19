$ErrorActionPreference = 'Stop'

$wifiPath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\network\wifi_mgr.c'))
$wifi = Get-Content -LiteralPath $wifiPath -Raw
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

$disconnected = [regex]::Match(
    $wifi,
    'WIFI_EVENT_STA_DISCONNECTED\)(?<body>.*?)(?:else if|else)',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$supervisor = [regex]::Match(
    $wifi,
    'static void wifi_supervisor_task(?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check ($wifi -match 'WIFI_STA_RETRY_INTERVAL_US') `
    'STA retry interval is configured'
Check ($wifi -match 's_sta_retry_due_us') `
    'STA retry deadline is tracked'
Check $disconnected.Success `
    'STA disconnected handler is present'
Check $supervisor.Success `
    'Wi-Fi supervisor is present'

if ($disconnected.Success) {
    Check ($disconnected.Groups['body'].Value -match
        's_sta_retry_due_us\s*=') `
        'disconnect schedules a retry deadline'
}

if ($supervisor.Success) {
    $body = $supervisor.Groups['body'].Value
    Check ($body -match 'retry_station') `
        'supervisor evaluates a STA retry'
    Check ($body -match 'esp_wifi_connect\s*\(\s*\)') `
        'supervisor actively reconnects the station'
    Check ($body -match 's_sta_retry_due_us\s*=') `
        'supervisor advances the retry deadline'
    Check ($body -match 'WIFI_STA_FALLBACK_US') `
        '20 second AP fallback remains present'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
