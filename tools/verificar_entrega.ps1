$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $projectDir
try {
    $pioExe = Join-Path $env:USERPROFILE '.platformio/penv/Scripts/pio.exe'
    $pythonExe = Join-Path $env:USERPROFILE '.platformio/penv/Scripts/python.exe'
    & $pythonExe tests/verificar_base.py
    if ($LASTEXITCODE -ne 0) { throw 'Fallo conservacion de adquisicion' }
    & $pioExe run -e genericSTM32F401RC -e solo_medicion
    if ($LASTEXITCODE -ne 0) { throw 'Fallo compilacion firmware' }
    $gpp = Get-Command g++.exe -ErrorAction SilentlyContinue
    if ($gpp) {
        New-Item -ItemType Directory -Force -Path .pio/host_tests | Out-Null
        foreach ($testName in @('test_state_math','test_motor_safety','test_command_parse')) {
            $binary = Join-Path $projectDir ".pio/host_tests/$testName.exe"
            & $gpp.Source -std=c++11 -O2 -Wall -Wextra -pedantic -Iinclude "tests/$testName.cpp" -o $binary
            if ($LASTEXITCODE -ne 0) { throw "Fallo compilando $testName" }
            & $binary
            if ($LASTEXITCODE -ne 0) { throw "Fallo $testName" }
        }
    } else { Write-Warning 'Pruebas C++ host OMITIDAS: g++ no disponible.' }
    $matlabExe = Get-Command matlab.exe -ErrorAction SilentlyContinue
    if ($matlabExe) {
        & $matlabExe.Source -batch "addpath('matlab'); prueba_estados; prueba_protocolo_manual; salida=evalc('prueba_analizar_pulsos'); disp('PASS analisis de pulsos, rollover y CSV/MAT'); modelo=preparar_lqr_fuerza(); files=dir('matlab/*.m'); for k=1:numel(files), a=checkcode(fullfile(files(k).folder,files(k).name)); assert(isempty(a),files(k).name); end"
        if ($LASTEXITCODE -ne 0) { throw 'Fallo verificaciones MATLAB' }
    } else { Write-Warning 'Pruebas MATLAB OMITIDAS: matlab.exe no disponible.' }
    Write-Output 'Verificacion de software terminada. No se abrio COM ni se cargaron/activaron motores.'
} finally { Pop-Location }
