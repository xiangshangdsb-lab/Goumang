param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = "Stop"
$mainDir = Join-Path $ProjectRoot "main"
$fontDir = Join-Path $mainDir "interface/panel/fonts"
$glyphs = [System.Collections.Generic.SortedSet[char]]::new()

foreach ($file in (Get-ChildItem -LiteralPath $mainDir -File -Recurse |
    Where-Object { $_.Extension -in @(".c", ".h", ".cpp") -and $_.Name -notlike "td_font_*" })) {
    $path = $file.FullName
    $text = Get-Content -LiteralPath $path -Raw -Encoding UTF8
    foreach ($ch in $text.ToCharArray()) {
        if ([int][char]$ch -gt 127 -and
            -not [char]::IsSurrogate($ch) -and
            -not [char]::IsControl($ch)) {
            [void]$glyphs.Add($ch)
        }
    }
}

$symbols = -join $glyphs
$manifest = Join-Path $fontDir "font_symbols.txt"
[System.IO.File]::WriteAllText($manifest, $symbols, [System.Text.UTF8Encoding]::new($false))
$candidate = Get-ChildItem -LiteralPath (
    Join-Path $env:LOCALAPPDATA "npm-cache\_npx"
) -Recurse -Filter "lv_font_conv.js" -ErrorAction SilentlyContinue |
    Select-Object -First 1
if ($null -eq $candidate) {
    throw "lv_font_conv.js was not found in the local npx cache"
}

$node = (Get-Command node.exe -ErrorAction Stop).Source
$font = "C:\Windows\Fonts\Noto Sans SC (TrueType).otf"
foreach ($size in @(16, 24)) {
    $output = Join-Path $fontDir "td_font_$size.c"
    & $node $candidate.FullName `
        --size $size `
        --bpp 4 `
        --format lvgl `
        --lv-include lvgl.h `
        --font $font `
        --range "0x20-0x7E" `
        --symbols $symbols `
        --no-compress `
        --no-kerning `
        --lv-font-name "td_font_$size" `
        -o $output
    if ($LASTEXITCODE -ne 0) {
        throw "lv_font_conv failed for size $size"
    }
}

Write-Host "Generated td_font_16.c and td_font_24.c with $($glyphs.Count) non-ASCII symbols"
