$ErrorActionPreference = 'Stop'
$ProjectRoot = (Resolve-Path -LiteralPath "$PSScriptRoot\..\..").Path
$sources = @(
    'test_calibration.c',
    "$ProjectRoot\Hardware\Calibration.c",
    "$ProjectRoot\Hardware\robot.c",
    "$ProjectRoot\Hardware\Irtracking.c",
    "$ProjectRoot\Hardware\Crossing.c",
    "$ProjectRoot\Hardware\Buzzer.c",
    "$ProjectRoot\Hardware\Corner.c",
    "$ProjectRoot\Hardware\Roundabout.c",
    "$ProjectRoot\Hardware\Gap.c",
    "$ProjectRoot\Hardware\Course.c"
)
# Prefer the MSVC toolset used originally; fall back to any gcc on PATH so the
# host tests also run on machines without Visual Studio.
$vc = 'D:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Tools\MSVC\14.44.35207'
$cl = "$vc\bin\Hostx64\x64\cl.exe"
Push-Location $PSScriptRoot
try {
    if (Test-Path -LiteralPath $cl) {
        Write-Host "host compiler: MSVC ($cl)"
        $sdk = (Get-ItemProperty -LiteralPath 'HKLM:\SOFTWARE\Microsoft\Windows Kits\Installed Roots').KitsRoot10.TrimEnd('\')
        $version = (Get-ChildItem -LiteralPath "$sdk\Include" -Directory | Sort-Object Name -Descending | Select-Object -First 1).Name
        $env:INCLUDE = "$vc\include;$sdk\Include\$version\ucrt;$sdk\Include\$version\shared;$sdk\Include\$version\um"
        $env:LIB = "$vc\lib\x64;$sdk\Lib\$version\ucrt\x64;$sdk\Lib\$version\um\x64"
        & $cl /nologo /W4 /WX /TC /I. "/I$ProjectRoot\Hardware" "/I$ProjectRoot\System" @sources /Fe:test_calibration.exe
        if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed (MSVC)' }
    }
    else {
        $gcc = (Get-Command gcc -ErrorAction SilentlyContinue).Source
        if (-not $gcc) { throw 'No host compiler found: install MSVC or put gcc on PATH' }
        Write-Host "host compiler: gcc ($gcc)"
        # -Wno-sign-compare: ramp() compares promoted int against the unsigned
        # CAL_PWM_STEP; the MSVC /W4 build this harness mirrors accepts that.
        & $gcc -std=c99 -Wall -Wextra -Werror -Wno-sign-compare -I. "-I$ProjectRoot\Hardware" "-I$ProjectRoot\System" @sources -o test_calibration.exe
        if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed (gcc)' }
    }
    & '.\test_calibration.exe'
    $testExit = $LASTEXITCODE
}
finally { Pop-Location }
exit $testExit
