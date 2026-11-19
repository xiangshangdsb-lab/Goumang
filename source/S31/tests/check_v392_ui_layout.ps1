$ErrorActionPreference = 'Stop'

$uiPath = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\main\interface\panel\ui.c'))
$ui = Get-Content -LiteralPath $uiPath -Raw
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

$summary = [regex]::Match(
    $ui,
    'lv_obj_set_size\(s_network_summary_label,\s*(?<w>\d+),\s*(?<h>\d+)\)')
$lineSpace = [regex]::Match(
    $ui,
    'lv_obj_set_style_text_line_space\(s_network_summary_label,\s*(?<space>\d+),\s*0\)')
$summaryStart = [regex]::Match(
    $ui,
    '"网络状态读取中",\s*20,\s*(?<y>\d+)')
$button = [regex]::Match(
    $ui,
    'make_button\(s_network_panel,\s*"作为终端联网",\s*20,\s*(?<y>\d+)')
$help = [regex]::Match(
    $ui,
    '"路由器名称和密码请在浏览器中填写.*?",\s*20,\s*(?<y>\d+)',
    [System.Text.RegularExpressions.RegexOptions]::Singleline)

Check $summary.Success 'network summary size is declared'
Check $lineSpace.Success 'network summary line spacing is declared'
Check $summaryStart.Success 'network summary position is declared'
Check $button.Success 'network buttons position is declared'
Check $help.Success 'network help position is declared'

if ($summary.Success -and $lineSpace.Success -and
    $summaryStart.Success -and $button.Success -and $help.Success) {
    $summaryY = [int]$summaryStart.Groups['y'].Value
    $summaryH = [int]$summary.Groups['h'].Value
    $space = [int]$lineSpace.Groups['space'].Value
    $buttonY = [int]$button.Groups['y'].Value
    $helpY = [int]$help.Groups['y'].Value
    $required = 8 * 20 + 7 * $space
    $secretsFirst =
        $ui.IndexOf('"网页控制码：%s') -lt
        $ui.IndexOf('"首选模式：%s')

    Check ($summaryH -ge $required) `
        "summary height $summaryH covers $required px"
    Check ($buttonY -ge ($summaryY + $required + 4)) `
        'network buttons start below the complete summary'
    Check ($helpY -ge ($buttonY + 44 + 4)) `
        'network help starts below the buttons'
    Check (($helpY + 5 * 20) -le 470) `
        'network help remains inside the 800x480 panel'
    Check $secretsFirst `
        'control code and hotspot password are the first two lines'
}

Write-Host "checks=$checks failures=$failures"
exit $(if ($failures -eq 0) { 0 } else { 1 })
