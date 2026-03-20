# Arduino GIGA R1 WiFi PID Webserver

This repo contains a minimal Arduino sketch for the Arduino GIGA R1 WiFi (M7 core) that:

- Hosts a webpage on the local WiFi network
- Lets you change `Kp`, `Ki`, `Kd`, and `Setpoint`
- Reads `analogRead(A0)` and shows it live in the browser
- Prints PID updates to `Serial`

Main Arduino sketch: [WEB_page/WEB_page.ino](/Users/annie/Documents/GitHub/AdvRobotics_AnnieSam/WEB_page/WEB_page.ino)

Webpage only: [WEB_page/webpage.h](/Users/annie/Documents/GitHub/AdvRobotics_AnnieSam/WEB_page/webpage.h)

## How to use

1. Open [WEB_page/WEB_page.ino](/Users/annie/Documents/GitHub/AdvRobotics_AnnieSam/WEB_page/WEB_page.ino) in Arduino IDE.
2. Replace `YOUR_WIFI_SSID` and `YOUR_WIFI_PASSWORD` with your network credentials.
3. Select board: `Arduino GIGA R1 WiFi`.
4. Make sure you are targeting the `M7` core.
5. Upload the sketch.
6. Open Serial Monitor at `115200` baud.
7. Wait for the board to print its IP address.
8. On a device connected to the same WiFi network, open a browser and go to `http://<board-ip>/`

## Web interface

The HTML page is stored separately in `webpage.h` as a raw string. It provides:

- Inputs for `Kp`, `Ki`, `Kd`, and `Setpoint`
- A `Send Updates` button
- A live sensor display that polls `/data`

## HTTP endpoints

- `/` serves the HTML page
- `/data` returns the current `A0` reading as plain text
- `/update?kp=...&ki=...&kd=...&sp=...` updates the global PID values
