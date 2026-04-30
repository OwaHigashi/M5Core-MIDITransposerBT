param(
  [string]$Port = 'COM10',
  [string]$OutDir = 'screenshots'
)

$ErrorActionPreference = 'Stop'

function Read-LineFromSerial {
  param([System.IO.Ports.SerialPort]$SerialPort)
  $builder = New-Object System.Text.StringBuilder
  while ($true) {
    $byte = $SerialPort.ReadByte()
    if ($byte -lt 0) { throw "Serial read returned EOF" }
    if ($byte -eq 10) { break }
    if ($byte -ne 13) { [void]$builder.Append([char]$byte) }
  }
  return $builder.ToString()
}

function Send-Command {
  param([System.IO.Ports.SerialPort]$SerialPort,[string]$Command,[int]$DelayMs = 250)
  $SerialPort.Write($Command + "`n")
  Start-Sleep -Milliseconds $DelayMs
}

function Wait-ForOkLine {
  param([System.IO.Ports.SerialPort]$SerialPort)
  while ($true) {
    $line = Read-LineFromSerial -SerialPort $SerialPort
    if ([string]::IsNullOrWhiteSpace($line)) { continue }
    if ($line.StartsWith('OK ')) { return $line }
  }
}

function Save-ScreenshotPpm {
  param([System.IO.Ports.SerialPort]$SerialPort,[string]$Path)
  $SerialPort.DiscardInBuffer()
  $SerialPort.Write("SCREENSHOT PPM`n")
  $headerLine = Wait-ForOkLine -SerialPort $SerialPort
  if ($headerLine -notmatch 'bytes=(\d+)') { throw "Unexpected: $headerLine" }
  $byteCount = [int]$Matches[1]
  $buffer = New-Object byte[] $byteCount
  $offset = 0
  while ($offset -lt $byteCount) {
    $read = $SerialPort.Read($buffer, $offset, $byteCount - $offset)
    if ($read -le 0) { throw "Timeout reading payload" }
    $offset += $read
  }
  [System.IO.File]::WriteAllBytes($Path, $buffer)
  while ($true) {
    $line = Read-LineFromSerial -SerialPort $SerialPort
    if ([string]::IsNullOrWhiteSpace($line)) { continue }
    if ($line -eq 'OK SCREENSHOT_DONE') { break }
  }
}

function Prepare-Screen {
  param([System.IO.Ports.SerialPort]$SerialPort,[string[]]$Commands)
  foreach ($command in $Commands) {
    Send-Command -SerialPort $SerialPort -Command $command -DelayMs 350
  }
  Send-Command -SerialPort $SerialPort -Command 'REDRAW' -DelayMs 400
}

function Convert-PpmToPng {
  param([string]$PpmPath, [string]$PngPath)
  Add-Type -AssemblyName System.Drawing
  $bytes = [System.IO.File]::ReadAllBytes($PpmPath)
  # Parse PPM(P6) header: "P6\n<w> <h>\n<maxval>\n<binary RGB>"
  $idx = 0
  function Read-Token {
    param([byte[]]$Bytes, [ref]$Index)
    # skip whitespace and comments
    while ($Index.Value -lt $Bytes.Length) {
      $c = $Bytes[$Index.Value]
      if ($c -eq 0x23) {
        while ($Index.Value -lt $Bytes.Length -and $Bytes[$Index.Value] -ne 0x0A) { $Index.Value++ }
      } elseif ($c -eq 0x20 -or $c -eq 0x09 -or $c -eq 0x0A -or $c -eq 0x0D) {
        $Index.Value++
      } else { break }
    }
    $sb = New-Object System.Text.StringBuilder
    while ($Index.Value -lt $Bytes.Length) {
      $c = $Bytes[$Index.Value]
      if ($c -eq 0x20 -or $c -eq 0x09 -or $c -eq 0x0A -or $c -eq 0x0D) { break }
      [void]$sb.Append([char]$c)
      $Index.Value++
    }
    return $sb.ToString()
  }
  $idxRef = [ref]0
  $magic  = Read-Token $bytes $idxRef
  if ($magic -ne 'P6') { throw "Not a P6 PPM: $PpmPath" }
  $w = [int](Read-Token $bytes $idxRef)
  $h = [int](Read-Token $bytes $idxRef)
  $mx = [int](Read-Token $bytes $idxRef)
  $idxRef.Value++  # consume single whitespace after maxval
  if ($mx -ne 255) { throw "Only maxval=255 supported, got $mx" }

  $bmp = New-Object System.Drawing.Bitmap $w, $h
  $rect = New-Object System.Drawing.Rectangle 0,0,$w,$h
  $data = $bmp.LockBits($rect,[System.Drawing.Imaging.ImageLockMode]::WriteOnly,[System.Drawing.Imaging.PixelFormat]::Format24bppRgb)
  $stride = $data.Stride
  $rowBytes = New-Object byte[] $stride
  for ($y = 0; $y -lt $h; $y++) {
    for ($x = 0; $x -lt $w; $x++) {
      $r = $bytes[$idxRef.Value]; $idxRef.Value++
      $g = $bytes[$idxRef.Value]; $idxRef.Value++
      $b = $bytes[$idxRef.Value]; $idxRef.Value++
      $off = $x * 3
      $rowBytes[$off]     = $b
      $rowBytes[$off + 1] = $g
      $rowBytes[$off + 2] = $r
    }
    $rowPtr = [System.IntPtr]::Add($data.Scan0, $stride * $y)
    [System.Runtime.InteropServices.Marshal]::Copy($rowBytes, 0, $rowPtr, $stride)
  }
  $bmp.UnlockBits($data)
  $bmp.Save($PngPath, [System.Drawing.Imaging.ImageFormat]::Png)
  $bmp.Dispose()
}

if (!(Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir | Out-Null }

# 00-play.ppm        : PLAY main screen with VOL/PRG/PB/SUS/INIT and TEST PHRASE
# 00-play-picker.ppm : program picker dialog (tap on the program-name bar at y=82..100)
$screens = @(
  @{ Name = '00-play.ppm';        Commands = @('MODE PLAY') },
  @{ Name = '00-play-picker.ppm'; Commands = @('MODE PLAY', 'TOUCH 160 108') }
)

$serialPort = New-Object System.IO.Ports.SerialPort $Port,115200,'None',8,'one'
$serialPort.ReadTimeout = 8000
$serialPort.WriteTimeout = 3000
$serialPort.NewLine = "`n"
$serialPort.Open()

try {
  Start-Sleep -Seconds 2
  $serialPort.DiscardInBuffer()
  $serialPort.DiscardOutBuffer()

  Send-Command -SerialPort $serialPort -Command 'HELP' -DelayMs 400
  [void](Wait-ForOkLine -SerialPort $serialPort)

  foreach ($screen in $screens) {
    Prepare-Screen -SerialPort $serialPort -Commands $screen.Commands
    $targetPath = Join-Path $OutDir $screen.Name
    Save-ScreenshotPpm -SerialPort $serialPort -Path $targetPath
    $pngPath = [System.IO.Path]::ChangeExtension($targetPath, '.png')
    Convert-PpmToPng -PpmPath $targetPath -PngPath $pngPath
    Write-Host "saved $targetPath / $pngPath"
    Start-Sleep -Milliseconds 300
  }
}
finally {
  $serialPort.Close()
}
