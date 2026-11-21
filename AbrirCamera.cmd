@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if not errorlevel 1 (
    py -3 -c "import tkinter" >nul 2>nul
    if not errorlevel 1 (
        py -3 "%~dp0monitor_camera.py"
        goto finished
    )
)
where python >nul 2>nul
if not errorlevel 1 (
    python -c "import tkinter" >nul 2>nul
    if not errorlevel 1 (
        python "%~dp0monitor_camera.py"
        goto finished
    )
)
set "CAMERA_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%CAMERA_PYTHON%" (
    "%CAMERA_PYTHON%" "%~dp0monitor_camera.py"
    goto finished
)
echo Python com Tkinter nao encontrado.
echo Instale Python 3.9 ou superior pelo instalador completo do python.org.
pause
exit /b 1
:finished
if errorlevel 1 (
    echo O monitor encontrou um problema. Veja a mensagem acima.
    pause
)
endlocal
