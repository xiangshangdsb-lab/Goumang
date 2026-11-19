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

$setIp = [regex]::Match(
    $wifi,
    'static void set_ip_state\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$startStation = [regex]::Match(
    $wifi,
    'static esp_err_t start_station\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check $setIp.Success 'IP state implementation is present'
Check $startStation.Success 'station start implementation is present'

if ($setIp.Success) {
    Check ($setIp.Groups['body'].Value -match
        'strcmp\s*\(\s*ip\s*,\s*WIFI_AP_IP\s*\)') `
        'access-point URL is selected only for the AP address'
}

if ($startStation.Success) {
    Check ($startStation.Groups['body'].Value -match
        'set_ip_state\s*\(\s*s_ap_active\s*\?') `
        'starting STA clears a stale AP address'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
