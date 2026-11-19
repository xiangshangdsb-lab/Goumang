$ErrorActionPreference = 'Stop'

$sourcePath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\interface\web\dashboard.html'))
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

Check ($source -match 'moveLeaseInFlight') `
    'manual move leases track an in-flight request'
Check ($source -match 'if\s*\(\s*moveLeaseInFlight\s*\)\s*return') `
    'a new lease is skipped while one request is in flight'
Check ($source -notmatch 'moveRequestQueue\s*=\s*moveRequestQueue') `
    'move leases are not accumulated in an unbounded Promise chain'
Check ($source -notmatch "enqueueMoveRequest\s*\(\s*['""]/api/stop") `
    'stop bypasses the move-lease queue'
Check ($source -match 'function\s+sendStopRequest\s*\(\s*\)') `
    'stop has a direct request helper'
Check ($source -match 'sendStopRequest\s*\(\s*\)') `
    'release sends stop immediately'

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
