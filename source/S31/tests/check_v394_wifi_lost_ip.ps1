$ErrorActionPreference = 'Stop'

$sourcePath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\network\wifi_mgr.c'))
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

$lostIpMatch = [regex]::Match(
    $source,
    'id == IP_EVENT_STA_LOST_IP(?<body>.*?)\}\s*\}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check $lostIpMatch.Success 'STA lost-IP handler is present'
if ($lostIpMatch.Success) {
    $body = $lostIpMatch.Groups['body'].Value
    Check ($body -match 's_sta_has_ip\s*=\s*false') `
        'lost IP clears the station IP state'
    Check ($body -match 'set_ip_state\s*\(') `
        'lost IP clears or replaces the advertised URL'
    Check ($body -match 's_ap_active\s*\?\s*WIFI_AP_IP\s*:\s*""') `
        'lost IP keeps the AP address only while an AP is active'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
