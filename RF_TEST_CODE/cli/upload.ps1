param(
  [Parameter(Mandatory = $true)]
  [string]$Sketch,

  [Parameter(Mandatory = $true)]
  [string]$Port,

  [ValidateSet("serialupdi", "serialupdi57k", "serialupdi230k_wd1", "jtag2updi", "atmelice_updi", "pickit4_updi", "snap_updi")]
  [string]$Programmer = "serialupdi",

  [ValidateSet("20internal", "16internal", "10internal", "8internal", "5internal", "4internal", "2internal", "1internal")]
  [string]$Clock = "10internal"
)

$ErrorActionPreference = "Stop"

$repo = Resolve-Path "$PSScriptRoot\..\.."
$sketchPath = Join-Path $repo $Sketch
$buildName = ($Sketch -replace '[\\/:*?"<>| ]', '_') + "_$Clock"
$buildPath = Join-Path $repo ".arduino-cli\build\$buildName"
$fqbn = "megaTinyCore:megaavr:atxy2:chip=402,clock=$Clock,millis=enabled"

Write-Host "Uploading:  $Sketch"
Write-Host "Port:       $Port"
Write-Host "Programmer: $Programmer"
Write-Host "Clock:      $Clock"
Write-Host "FQBN:       $fqbn"

Remove-Item -LiteralPath $buildPath -Recurse -Force -ErrorAction SilentlyContinue
arduino-cli compile --fqbn $fqbn --build-path $buildPath $sketchPath
if ($LASTEXITCODE -ne 0) {
  throw "arduino-cli compile failed with exit code $LASTEXITCODE"
}
arduino-cli upload -p $Port --fqbn $fqbn --programmer $Programmer --input-dir $buildPath $sketchPath
if ($LASTEXITCODE -ne 0) {
  throw "arduino-cli upload failed with exit code $LASTEXITCODE"
}
