$ErrorActionPreference = 'Stop'
foreach ($name in @('run_eight_tests.ps1', 'run_tests.ps1', 'run_display_tests.ps1', 'run_maneuver_tests.ps1', 'run_course_tests.ps1', 'run_ring_white_tests.ps1', 'run_startup_tests.ps1', 'run_buzzer_tests.ps1', 'run_smooth_corner_tests.ps1')) {
    & (Join-Path $PSScriptRoot "Calibration/$name")
    if ($LASTEXITCODE -ne 0) { throw "Failed: $name" }
}
Write-Host 'All nine suites passed; six single-feature modes, separated breaks, automatic corners and crossing beep checked.'
