# S1 dashboard

PC dashboard for subsystem 1: live telemetry, PAUSE / RESUME / ABORT,
map entry, event log and CSV export.

| File | What it is |
|---|---|
| `start.bat` | Starts the broker in its own window, then opens the page |
| `mosquitto.conf` | Broker config: port 1883 for the car, 9001 (WebSockets) for the page |
| `index.html` | The dashboard |
| `codec.js` | Frame decoder and encoder. Mirrors `app_program/subsystem1/telem_layout.h`: change both together |
| `mqtt.min.js` | MQTT.js 5.10.4 (MIT licence), kept here so the page works offline |

## Run (Windows)

1. Install Mosquitto in `C:\Program Files\Mosquitto`. Stop and disable its
   Windows service, so only this broker runs.
2. Copy this folder to Windows (the batch file can't run from the WSL path):
   `cp -r tools/S1_dashboard /mnt/c/Users/<you>/S1_dashboard`
3. Connect the laptop to the hotspot and flash the car with
   `make WIFI=cyw43 NET=lwip MQTT=1`.
4. Double-click `start.bat`.

`start.bat` warns if port 1883 is already in use, or if the laptop's address
isn't the broker address built into the car (`MQTT_BROKER_IP` in
`config/wifi_secrets.h`). Close the broker window to stop the broker.