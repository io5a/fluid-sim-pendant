param(
  [string]$Port = "COM8",
  [switch]$CheckMapping
)

$ErrorActionPreference = "Stop"
$platformio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\platformio.exe"
if (-not (Test-Path $platformio)) {
  throw "PlatformIO executable not found: $platformio"
}

$serial = $null
$restoreProductionFirmware = $false
$testsPassed = $false
$mappingPassed = $true

try {
  $restoreProductionFirmware = $true
  & $platformio run -e fluid_sim_self_test -t upload --upload-port $Port
  if ($LASTEXITCODE -ne 0) {
    throw "Failed to upload the simulation test firmware."
  }

  $serial = [System.IO.Ports.SerialPort]::new($Port, 115200)
  $serial.ReadTimeout = 500
  $serial.DtrEnable = $true
  $portDeadline = [DateTime]::UtcNow.AddSeconds(20)
  $lastOpenError = $null
  while (-not $serial.IsOpen -and [DateTime]::UtcNow -lt $portDeadline) {
    try {
      $serial.Open()
    } catch {
      $lastOpenError = $_
      $null = [System.IO.Ports.SerialPort]::GetPortNames()
      $null = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue
    }
  }
  if (-not $serial.IsOpen) {
    throw "Serial port $Port did not return after firmware upload: $lastOpenError"
  }

  $serial.Write($(if ($CheckMapping) { "M" } else { "T" }))
  $deadline = [DateTime]::UtcNow.AddSeconds(30)
  while ([DateTime]::UtcNow -lt $deadline) {
    try {
      $line = $serial.ReadLine().Trim()
      if ($line) {
        Write-Output $line
      }
      if ($line -eq "SIMTEST_PASS") {
        $testsPassed = $true
        break
      }
      if ($line -eq "SIMTEST_FAIL") {
        break
      }
      if ($line -eq "MAP_EXPECT_BOTTOM" -or $line -eq "MAP_EXPECT_LEFT") {
        $expectedEdge = if ($line -eq "MAP_EXPECT_BOTTOM") { "bottom" } else { "left" }
        $observedEdge = (Read-Host "Which physical edge is illuminated? Enter top, bottom, left, or right").Trim().ToLowerInvariant()
        Write-Output "MAP_OBSERVED expected=$expectedEdge observed=$observedEdge"
        $mappingPassed = $mappingPassed -and ($observedEdge -eq $expectedEdge)
        $serial.Write("N")
        $deadline = [DateTime]::UtcNow.AddSeconds(30)
      }
      if ($line -eq "MAP_DONE") {
        $testsPassed = $mappingPassed
        break
      }
    } catch [System.TimeoutException] {
    }
  }

  if (-not $testsPassed) {
    if ($CheckMapping) {
      throw "Physical LED edges do not match logical display axes."
    }
    throw "Simulation self-tests did not report SIMTEST_PASS."
  }
} finally {
  if ($serial -and $serial.IsOpen) {
    $serial.Close()
  }
  if ($restoreProductionFirmware) {
    & $platformio run -e seeed_xiao_nrf52840 -t upload --upload-port $Port
    if ($LASTEXITCODE -ne 0) {
      throw "Failed to restore the regular pendant firmware."
    }
  }
}