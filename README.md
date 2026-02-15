# 📡 ESP8266 Ultrasonic WiFi Radar (CRT-style)

Turn a cheap ultrasonic sensor into a **browser-based radar** with a glorious **CRT / phosphor** look.

- **ESP8266 (Wemos D1 R2 recommended)**
- **HC-SR04** ultrasonic sensor
- **Servo** (e.g. SG90)
- Real-time visualization in your browser (no external JS libraries)

<p align="center">
  <img src="assets/preview.png" width="900" alt="Radar UI preview">
</p>

## ✨ Features

- Live **sweep line** (always visible)
- **Phosphor persistence** + per-sweep fading
- **Scanlines**, bloom, vignette, slight “curvature”
- Fixed-size **DOS/CRT readout** with leading zeros
- Filtering options: **median / mean** aggregation + optional spike filter
- Self-contained: **single `.ino`** file serves UI + data endpoint

---

## 🧰 Hardware

### Bill of materials
- ESP8266 board (Wemos D1 R2 / NodeMCU)
- HC-SR04
- Servo SG90 (or similar)
- Jumper wires
- Optional: voltage divider for ECHO (recommended)

### Pinout (default)
| Signal | ESP8266 |
|---|---|
| Servo | D5 |
| TRIG  | D6 |
| ECHO  | D7 |

> ⚠️ HC-SR04 **ECHO is 5V**. ESP8266 GPIO are 3.3V.  
> For long-term safety use a simple divider to bring 5V → ~3.3V.

---

## 🚀 Quick start (Arduino IDE)

1. Install **ESP8266 board support** in Arduino IDE.
2. Copy `secrets.h.example` → `secrets.h`
3. Edit WiFi credentials.
4. Flash the sketch.
5. Open Serial Monitor (115200) to read IP.
6. Browse to: `http://<esp-ip>/`

If WiFi STA fails, it falls back to AP mode:
- SSID: `ESP8266-Radar` (default)
- PASS: `12345678` (default)

---

## 🎛 Tuning (the fun knobs)

### Sensor / scan settings (C++ constants)
Open `ESP8266_radar_wifi.ino` and tweak:

- `MAX_CM` (full scale)
- `ANGLE_STEP` and `LOOP_PERIOD_MS` (scan speed)
- `SONAR_SAMPLES` (1..9)
- `AGGREGATION_MODE` (median vs mean)
- `ENABLE_SPIKE_FILTER` + `SPIKE_MAX_JUMP_CM`

### Visual persistence (inside the embedded HTML/JS)
Search for these in the HTML:

- `FADE_TICKS` – how long points live
- `PHOSPHOR_FADE` – how quickly the phosphor buffer fades
- `SWEEP_FADE_BOOST` – extra aging each full sweep

Example “long persistence” preset:
```js
const FADE_TICKS = 450;
const PHOSPHOR_FADE = 0.06;
const SWEEP_FADE_BOOST = 4;
```

---

## 📡 Endpoints

- `/` → UI
- `/data` → JSON data:
  ```json
  {"a":42,"d":123.4,"mode":"STA","ip":"192.168.1.50","max_cm":200,"echo_to":11890}
  ```

---

## 🗺️ Roadmap ideas
- Color/intensity by distance
- WebSocket streaming instead of polling
- ESP32 port + higher FPS

---

## 📜 License
MIT (see `LICENSE`)
