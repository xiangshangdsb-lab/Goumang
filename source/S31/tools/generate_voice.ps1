param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$Clip = "",
    [switch]$GreetingOnly
)
$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Speech
$voiceDir = Join-Path $ProjectRoot "voice"
$prompts = [ordered]@{
    boot_ok = "句芒启动，上电自检开始。"
    say_test = "你好，我是句芒。语音播报测试正常。"
    cam_missing = "句芒提示，摄像头未连接，请检查排线。"
    face_found = "句芒提示，人脸识别完成。"
    inspect = "句芒开始巡检。"
    line_follow = "句芒开始巡线。"
    low_battery = "句芒提示，电池电压偏低，请及时充电。"
    obstacle = "句芒提示，前方有障碍，请注意安全。"
    greet_hello = "你好"
    greet_wish = "号，希望你天天开心快乐，总是无忧。"
}
$digits = @("零", "一", "二", "三", "四", "五", "六", "七", "八", "九")
for ($i = 0; $i -lt $digits.Length; $i++) { $prompts["greet_d$i"] = $digits[$i] }
# The same table drives the runtime result names and the recorded phrase bank.
$policy = Get-Content -LiteralPath (Join-Path $ProjectRoot "main\diagnostics\selftest_policy.c") -Raw -Encoding UTF8
foreach ($match in [regex]::Matches($policy, '\{"[^"\r\n]+", "([^"\r\n]+)", "(st_[a-z_]+)"\}')) {
    $prompts[$match.Groups[2].Value] = $match.Groups[1].Value
}
$prompts["st_done"] = "上电自检完成。"
$prompts["st_done_warn"] = "上电自检完成，请检查提示项目。"
$synth = [System.Speech.Synthesis.SpeechSynthesizer]::new()
try {
    if ($Clip -and -not $prompts.Contains($Clip)) { throw "Unknown voice clip: $Clip" }
    $synth.SelectVoice("Microsoft Huihui Desktop")
    $synth.Rate = 0
    $synth.Volume = 100
    foreach ($entry in $prompts.GetEnumerator()) {
        if ($Clip -and $entry.Key -ne $Clip) { continue }
        if ($GreetingOnly -and -not $entry.Key.StartsWith("greet_")) { continue }
        $path = Join-Path $voiceDir ($entry.Key + ".wav")
        $isSelftest = $entry.Key.StartsWith("st_")
        $synth.Rate = $(if ($isSelftest) { 2 } else { 0 })
        $sampleRate = $(if ($isSelftest -or $entry.Key.StartsWith("greet_")) { 8000 } else { 16000 })
        $format = [System.Speech.AudioFormat.SpeechAudioFormatInfo]::new(
            $sampleRate, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen,
            [System.Speech.AudioFormat.AudioChannel]::Mono)
        # Keep the displayed name correct while disambiguating the polyphonic 句.
        $speech = [System.Security.SecurityElement]::Escape($entry.Value).
            Replace("句芒", '<sub alias="勾芒">句芒</sub>')
        $ssml = '<speak version="1.0" xmlns="http://www.w3.org/2001/10/synthesis" xml:lang="zh-CN">' + $speech + '</speak>'
        $synth.SetOutputToWaveFile($path, $format)
        $synth.SpeakSsml($ssml)
        $synth.SetOutputToNull()
        Write-Host ("Generated " + $entry.Key)
    }
    $manifest = $prompts | ConvertTo-Json
    [IO.File]::WriteAllText((Join-Path $voiceDir "prompts.json"), $manifest,
        [Text.UTF8Encoding]::new($false))
    $clipPattern = $(if ($GreetingOnly) { "greet_*.wav" } elseif ($Clip) { "$Clip.wav" } else { "" })
    & python (Join-Path $PSScriptRoot "trim_selftest_voice.py") $voiceDir $clipPattern
    if ($LASTEXITCODE -ne 0) { throw "Voice boundary processing failed" }
}
finally { $synth.Dispose() }
