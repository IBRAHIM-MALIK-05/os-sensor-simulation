# Builds sensor_sim.exe from every .cpp file in src/
# Usage:  .\build.ps1
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$sources = Get-ChildItem -Path src -Recurse -Filter *.cpp | ForEach-Object { $_.FullName }
g++ -std=c++17 -O2 -Wall -Wextra -static $sources -o sensor_sim.exe

if ($LASTEXITCODE -eq 0) { Write-Host "Built sensor_sim.exe" } else { exit 1 }
