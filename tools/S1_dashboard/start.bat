@echo off
rem start.bat
rem Subsystem 1: starts the Mosquitto broker in its own window, then opens
rem the dashboard. Double-click to run. Close the broker window to stop it.

rem ---- Settings ----------------------------------------------------------
set "MOSQ=C:\Program Files\Mosquitto\mosquitto.exe"
rem Must match MQTT_BROKER_IP in config/wifi_secrets.h (built into the car)
set "BROKER_IP=172.20.10.3"

rem ---- Checks ------------------------------------------------------------
if not exist "%MOSQ%" (
    echo Mosquitto not found at "%MOSQ%".
    echo Install it, or fix MOSQ at the top of this file.
    pause
    exit /b 1
)

netstat -ano | findstr /r /c:":1883 .*LISTENING" >nul
if not errorlevel 1 (
    echo A broker is already running on port 1883.
    echo If it is the Mosquitto Windows service, stop it first:
    echo     Services, Mosquitto Broker, Stop
    pause
    exit /b 1
)

ipconfig | findstr /c:"%BROKER_IP%" >nul
if errorlevel 1 (
    echo WARNING: this laptop's address is not %BROKER_IP%.
    echo The car will not connect. Check the hotspot connection, or
    echo update MQTT_BROKER_IP and BROKER_IP and rebuild the car.
    echo.
    pause
)

rem ---- Start -------------------------------------------------------------
start "S1 broker" "%MOSQ%" -c "%~dp0mosquitto.conf" -v
timeout /t 2 /nobreak >nul
start "" "%~dp0index.html"