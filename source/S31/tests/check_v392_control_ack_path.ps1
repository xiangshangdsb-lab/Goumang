$ErrorActionPreference = 'Stop'

$controlPath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\control\control.c'))
$control = Get-Content -LiteralPath $controlPath -Raw
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

$task = [regex]::Match(
    $control,
    'static void control_task\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check $task.Success 'control task is present'
Check ($control -notmatch
    'robot_link_send_line_mode\(\s*false\s*,') `
    'manual and inspection drive paths avoid a redundant Line ACK wait'
Check ($control -match 'uint32_t\s+stop_ack_generation') `
    'idle stop acknowledgement belongs to a motion generation'
Check ($control -match 'stop_ack_generation\s*!=\s*s_motion\.generation') `
    'idle STOP_ALL is sent only until the current stop is acknowledged'
Check ($control -match '(?s)static uint32_t enter_mode_locked.*?stop_ack_generation\s*=\s*0') `
    'entering a new mode rearms the stop'
Check ($control -match '(?s)if\s*\(ok && current.*?stop_ack_generation\s*=\s*generation') `
    'a current stop ACK is remembered'

$link = Get-Content -LiteralPath (Join-Path $PSScriptRoot '../main/transport/robot_link.c') -Raw
$rxHandler = [regex]::Match($link, '(?s)static void handle_frame\([^\r\n]+\)\r?\n\{.*?\r?\n\}')
Check ($rxHandler.Success -and $rxHandler.Value -notmatch 'publish_topic|rb_port_lock') `
    'the USB receiver can deliver an ACK without entering the robot bus'

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
