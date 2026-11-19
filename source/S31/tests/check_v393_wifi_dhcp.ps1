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

$connected = [regex]::Match(
    $wifi,
    'WIFI_EVENT_STA_CONNECTED\)(?<body>.*?)else if',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$gotIp = [regex]::Match(
    $wifi,
    'IP_EVENT_STA_GOT_IP\)(?<body>.*?)else if',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$supervisor = [regex]::Match(
    $wifi,
    'static void wifi_supervisor_task\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check ($wifi -match 's_sta_has_ip') `
    'STA IP acquisition state is tracked'
Check $connected.Success `
    'STA connected event is present'
Check $gotIp.Success `
    'STA got-IP event is present'
Check $supervisor.Success `
    'Wi-Fi supervisor is present'

if ($connected.Success) {
    Check ($connected.Groups['body'].Value -notmatch
        's_fallback_pending\s*=\s*false') `
        'link association does not cancel the fallback deadline'
}

if ($gotIp.Success) {
    Check ($gotIp.Groups['body'].Value -match
        's_sta_has_ip\s*=\s*true') `
        'got-IP marks the station usable'
}

if ($supervisor.Success) {
    $body = $supervisor.Groups['body'].Value
    Check ($body -match
        's_fallback_pending\s*&&\s*!s_sta_has_ip') `
        'fallback waits for IP rather than link association'
    Check ($body -match
        's_sta_has_ip\s*&&\s*s_ap_active') `
        'AP closes only after station IP is available'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
