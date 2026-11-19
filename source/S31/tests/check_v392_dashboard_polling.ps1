$ErrorActionPreference = 'Stop'

$htmlPath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\interface\web\dashboard.html'))
$html = Get-Content -LiteralPath $htmlPath -Raw
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

$agent = [regex]::Match(
    $html,
    'async function refreshAgentStatus\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$frames = [regex]::Match(
    $html,
    'function refreshFrames\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check ($html -match '<link\s+rel="icon"\s+href="data:,">') `
    'dashboard embeds an empty favicon'
Check ($html -match 'let\s+agentPollTimer') `
    'dashboard stores the Agent poll timer'
Check ($html -match 'function\s+startAgentPolling\s*\(') `
    'dashboard starts Agent polling explicitly'
Check ($html -notmatch
    '(?m)^\s*setInterval\(refreshAgentStatus,\s*900\)') `
    'dashboard has no unconditional Agent poll'
Check ($html -match 'let\s+cameraReady\s*=\s*false') `
    'dashboard tracks camera readiness'

if ($agent.Success) {
    $body = $agent.Groups['body'].Value
    Check ($body -match 'if\s*\(!controlToken\)' -and
        $body -match 'return\s*;') `
        'Agent status returns before fetch when unauthenticated'
}

if ($frames.Success) {
    $body = $frames.Groups['body'].Value
    Check ($body -match 'if\s*\(!cameraReady\)' -and
        $body -match 'return\s*;') `
        'camera frames are not requested before readiness'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
