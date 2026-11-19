$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
function Read-Normalized([string]$Path) {
    return (Get-Content -Raw $Path).Replace("`r`n", "`n")
}

$sdkconfig = Read-Normalized (Join-Path $root "sdkconfig")
$defaults = Read-Normalized (Join-Path $root "sdkconfig.defaults")
$bspConfig = Read-Normalized (Join-Path $root "sdkconfig.bsp.esp32_s31_korvo_1")
$headerPath = Join-Path $root "components\esp32_s31_korvo_1\include\bsp\esp32_s31_korvo_1.h"
$header = Read-Normalized $headerPath
$appConfig = Read-Normalized (Join-Path $root "main\app\app_config.h")

$checks = [System.Collections.Generic.List[object]]::new()

function Add-Check {
    param(
        [string]$Name,
        [bool]$Condition
    )
    $checks.Add([pscustomobject]@{ Name = $Name; Passed = $Condition })
}

Add-Check "active target is esp32s31" `
    ($sdkconfig -match '(?m)^CONFIG_IDF_TARGET="esp32s31"$')
Add-Check "default target is esp32s31" `
    ($defaults -match '(?m)^CONFIG_IDF_TARGET="esp32s31"$')
Add-Check "PSRAM runs at 250 MHz" `
    ($sdkconfig -match '(?m)^CONFIG_SPIRAM_SPEED_250M=y$' -and
     $sdkconfig -match '(?m)^CONFIG_SPIRAM_SPEED=250$')
Add-Check "RGB panel uses 26 MHz" `
    ($header -match '(?m)^\s*\.pclk_hz\s*=\s*26\s*\*\s*1000\s*\*\s*1000')
Add-Check "RGB horizontal timing is 1/40/20" `
    ($header -match '(?m)^\s*\.hsync_pulse_width\s*=\s*1' -and
     $header -match '(?m)^\s*\.hsync_back_porch\s*=\s*40' -and
     $header -match '(?m)^\s*\.hsync_front_porch\s*=\s*20')
Add-Check "RGB vertical timing is 1/10/5" `
    ($header -match '(?m)^\s*\.vsync_pulse_width\s*=\s*1' -and
     $header -match '(?m)^\s*\.vsync_back_porch\s*=\s*10' -and
     $header -match '(?m)^\s*\.vsync_front_porch\s*=\s*5')
Add-Check "bounce-buffer mode is disabled" `
    ($sdkconfig -match '(?m)^# CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE is not set$')
Add-Check "SC101IOT preview rotation is 0" `
    ($header -match '(?m)^#define BSP_CAMERA_ROTATION\s+\(0\)$')
Add-Check "camera XCLK source enum stays available" `
    ($sdkconfig -match '(?m)^CONFIG_CAMERA_XCLK_USE_LEDC=y$')
Add-Check "LVGL performance overlay is disabled" `
    ($sdkconfig -notmatch '(?m)^CONFIG_LV_USE_PERF_MONITOR=y$' -and
     $sdkconfig -match '(?m)^# CONFIG_LV_USE_SYSMON is not set$')
Add-Check "S31 preset has no ESP32-S3 cache setting" `
    ($bspConfig -notmatch '(?m)^CONFIG_ESP32S3_DATA_CACHE_LINE_64B=y$')
Add-Check "S31 preset does not force 80 MHz PSRAM" `
    ($bspConfig -notmatch '(?m)^CONFIG_SPIRAM_SPEED_80M=y$')
Add-Check "manual AP SSID uses Goumang branding" `
    ($appConfig -match '(?m)^#define TD_AP_SSID\s+"Goumang-S31"$')
Add-Check "setup AP SSID uses Goumang branding" `
    ($appConfig -match '(?m)^#define TD_AP_SETUP_SSID\s+"Goumang-S31-Setup"$')

$failed = @($checks | Where-Object { -not $_.Passed })
$checks | ForEach-Object {
    $state = if ($_.Passed) { "PASS" } else { "FAIL" }
    Write-Host ("{0}  {1}" -f $state, $_.Name)
}
Write-Host ("checks={0} failures={1}" -f $checks.Count, $failed.Count)
exit ([int]$failed.Count -gt 0)
