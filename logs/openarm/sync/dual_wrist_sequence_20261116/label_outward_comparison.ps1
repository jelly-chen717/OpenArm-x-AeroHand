$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$dir = Split-Path -Parent $MyInvocation.MyCommand.Path
$raw = [System.Drawing.Image]::FromFile((Join-Path $dir 'outward_direction_comparison_raw.png'))
$canvas = New-Object System.Drawing.Bitmap 1800,1000
$g = [System.Drawing.Graphics]::FromImage($canvas)
$g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$g.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::AntiAliasGridFit
$g.Clear([System.Drawing.Color]::FromArgb(245,248,252))
$g.DrawImage($raw,0,100,1800,850)
$title = [System.Drawing.Font]::new('Microsoft YaHei',25,[System.Drawing.FontStyle]::Bold,[System.Drawing.GraphicsUnit]::Pixel)
$normal = [System.Drawing.Font]::new('Microsoft YaHei',19,[System.Drawing.FontStyle]::Regular,[System.Drawing.GraphicsUnit]::Pixel)
$label = [System.Drawing.Font]::new('Microsoft YaHei',23,[System.Drawing.FontStyle]::Bold,[System.Drawing.GraphicsUnit]::Pixel)
$ink = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(23,42,62))
$white = [System.Drawing.Brushes]::White
$outward = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(97,255,196))
$inward = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(255,187,94))
$g.DrawString('原同号：左 J5 −88° / 右 J5 −88°',$title,$ink,40,16)
$g.DrawString('双侧朝外：左 J5 −88° / 右 J5 +88°',$title,$ink,940,16)
$g.DrawString('第 2 步末 · 两臂 J7 +88° · 最新复位基准',$normal,$ink,40,61)
$g.DrawString('第 2 步末 · 两臂 J7 +88° · 最新复位基准',$normal,$ink,940,61)
foreach ($offset in @(0,900)) {
  $g.DrawString('机器人右臂',$normal,$white,(275+$offset),213)
  $g.DrawString('机器人左臂',$normal,$white,(515+$offset),213)
}
function Draw-Arrow([int]$x1,[int]$x2,[int]$y,[System.Drawing.SolidBrush]$brush,[string]$text,[int]$textx) {
  $pen = New-Object System.Drawing.Pen $brush.Color,5
  $pen.CustomEndCap = New-Object System.Drawing.Drawing2D.AdjustableArrowCap 5,6
  $g.DrawLine($pen,$x1,$y,$x2,$y)
  $g.DrawString($text,$label,$brush,$textx,($y+12))
  $pen.Dispose()
}
Draw-Arrow 330 425 695 $inward '向内' 350
Draw-Arrow 565 690 695 $outward '向外' 605
Draw-Arrow 1240 1115 695 $outward '向外' 1150
Draw-Arrow 1465 1590 695 $outward '向外' 1505
$divider = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(210,221,231)),2
$g.DrawLine($divider,900,0,900,950)
$g.DrawString('正面看机器人：画面左侧是机器人右臂；画面右侧是机器人左臂。此图为模型姿态对比，未执行实机。',$normal,$ink,40,962)
$canvas.Save((Join-Path $dir 'outward_direction_comparison.png'),[System.Drawing.Imaging.ImageFormat]::Png)
$divider.Dispose();$inward.Dispose();$outward.Dispose();$ink.Dispose();$label.Dispose();$normal.Dispose();$title.Dispose();$g.Dispose();$canvas.Dispose();$raw.Dispose()
