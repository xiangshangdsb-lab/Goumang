$ErrorActionPreference = 'Stop'

$linkPath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\transport\robot_link.c'))
$controlPath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\control\control.c'))
$link = Get-Content -LiteralPath $linkPath -Raw
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

$estop = [regex]::Match(
    $link,
    'bool robot_link_emergency_stop\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$priority = [regex]::Match(
    $link,
    'static bool send_frame_priority\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$manual = [regex]::Match(
    $control,
    'bool control_manual\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)
$line = [regex]::Match(
    $control,
    'bool control_start_line_speed\((?<body>.*?)\r?\n}',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check $estop.Success `
    'emergency-stop link function is present'
Check $priority.Success `
    'priority frame sender exists'
Check $manual.Success `
    'manual control implementation is present'
Check $line.Success `
    'line-start implementation is present'

if ($estop.Success) {
    Check ($estop.Groups['body'].Value -match
        'send_frame_priority\s*\(') `
        'E-STOP bypasses the ordinary command mutex'
    Check ($estop.Groups['body'].Value -notmatch
        'send_frame_wait_ack\s*\(') `
        'E-STOP does not wait for an ordinary ACK transaction'
}

Check ($manual.Groups['body'].Value -match 'SEND_DRIVE' -and
       $manual.Groups['body'].Value -match 'begin_driving') 'manual requests use the ordered sender'
Check ($line.Groups['body'].Value -match 'SEND_LINE' -and
       $line.Groups['body'].Value -match 'begin_driving') 'line requests use the ordered sender'

$begin = [regex]::Match($control, '(?s)static bool begin_driving\(.*?\r?\n\}').Value
$ordered = [regex]::Match($control, '(?s)static bool send_ordered\(.*?\r?\n\}').Value
Check ($begin -match '(?s)manual_deadline_us\s*=.*?unlock_motion\(\).*?send_ordered') `
    'manual control arms its deadline and releases the mode lock before USB ACK waiting'
Check ($begin -match '(?s)line_speed\s*=.*?unlock_motion\(\).*?send_ordered') `
    'line start records its speed and releases the mode lock before USB ACK waiting'
Check ($ordered -match '(?s)generation\s*==\s*generation.*?unlock_motion\(\).*?send_command') `
    'the ordered sender checks the generation before releasing the mode lock and sending'
$localEstop = [regex]::Match($control, '(?s)bool control_estop\(.*?\r?\n\}').Value
Check ($localEstop -match 'robot_link_emergency_stop\(\)' -and $localEstop -notmatch 'send_ordered') `
    'E-STOP bypasses the motion transaction lock'

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
