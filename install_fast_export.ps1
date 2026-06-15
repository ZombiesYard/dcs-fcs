$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$dcsScripts = Join-Path $env:USERPROFILE "Saved Games\DCS\Scripts"
$targetDir = Join-Path $dcsScripts "AH64DAutoRudder"
$exportLua = Join-Path $dcsScripts "Export.lua"
$sourceLua = Join-Path $root "lua\ah64d_auto_rudder_export.lua"
$loadLine = 'dofile(lfs.writedir() .. [[Scripts\AH64DAutoRudder\ah64d_auto_rudder_export.lua]])'
$needle = 'Scripts\AH64DAutoRudder\ah64d_auto_rudder_export.lua'

if (-not (Test-Path $dcsScripts)) {
    throw "DCS Scripts directory not found: $dcsScripts"
}

New-Item -ItemType Directory -Force -Path $targetDir | Out-Null
Copy-Item -Force -Path $sourceLua -Destination (Join-Path $targetDir "ah64d_auto_rudder_export.lua")

if (-not (Test-Path $exportLua)) {
    New-Item -ItemType File -Force -Path $exportLua | Out-Null
}

$content = Get-Content -Raw -Path $exportLua
if ($content -notlike "*$needle*") {
    Add-Content -Path $exportLua -Value ""
    Add-Content -Path $exportLua -Value $loadLine
    Write-Host "Added fast export loader to: $exportLua"
} else {
    Write-Host "Fast export loader already present in: $exportLua"
}

Write-Host "Installed fast export script to: $targetDir"
