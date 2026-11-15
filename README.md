# DECLIVOLT
### Downhill Energy Recovery & Smart Vehicle Control System
**Tagline:** *Turning Downhill Motion into Electrical Energy.*

DECLIVOLT is an embedded telemetry and regenerative energy recovery project combining an **ESP32 microcontroller**, an **L298N Dual H-Bridge Motor Driver**, an **MPU6050 6-Axis Motion Sensor**, and a **DC Voltage Sensor** with a responsive web dashboard utilizing the **Web Serial API**.

---

## 📁 Project Structure

```
DECLIVOLT/
│
├── index.html                  # Dashboard structure & visual layouts
├── style.css                   # Engineering-grade responsive styling
├── script.js                   # Web Serial communication & telemetry parsing
│
└── ESP32/
    └── DECLIVOLT_ESP32.ino     # Complete upload-ready Arduino IDE sketch
```

---

## ⚡ Hardware Pinout Reference

| Module | Pin Name | ESP32 GPIO | Description |
|---|---|---|---|
| **L298N Motor Driver** | ENA | GPIO 25 | Motor 1 Speed / Enable |
| | IN1 | GPIO 26 | Motor 1 Direction Control |
| | IN2 | GPIO 27 | Motor 1 Direction Control |
| | ENB | GPIO 33 | Motor 2 Speed / Enable |
| | IN3 | GPIO 14 | Motor 2 Direction Control |
| | IN4 | GPIO 13 | Motor 2 Direction Control |
| **MPU6050 Sensor** | SDA | GPIO 21 | I2C Data (Address: `0x68`) |
| | SCL | GPIO 22 | I2C Clock |
| **DC Voltage Sensor** | Signal Out | GPIO 34 | Analog ADC input (Generator recovery) |
| **LM393 IR Speed Sensor** | D0 (Digital) | GPIO 32 | Digital pulse count interrupt (Wheel RPM) |
| | VCC | 3.3V | Power supply |
| | GND | GND | Common ground |
| | A0 (Analog) | Not connected | Unused (Digital D0 used) |

---

## 🚀 How to Upload Firmware to ESP32

1. Open **Arduino IDE** (v1.8+ or v2.x).
2. Select **Tools > Board > ESP32 Arduino > ESP32 Dev Module** (or your specific ESP32 board).
3. Connect your ESP32 to your PC via USB and select the correct COM port in **Tools > Port**.
4. Open `ESP32/DECLIVOLT_ESP32.ino`.
5. Ensure the vehicle is placed on a flat, level surface during boot so the flat-surface calibration initializes accurately.
6. Click **Upload**.
7. Close the Arduino IDE Serial Monitor before using the web dashboard (only one application can access the serial port at a time).

---

## 🌐 How to Use the Web Dashboard

1. Open `index.html` in a Web Serial compatible browser (**Google Chrome** or **Microsoft Edge**).
2. Click the **CONNECT** button in the header.
3. Select your ESP32 serial port in the browser prompt and click **Connect**.
4. The status badge will change to **CONNECTED**, and real-time telemetry (voltage, pitch angle, slope classification, acceleration axes) will begin streaming.
5. Control vehicle drive motors using the **FORWARD**, **BACKWARD**, and **STOP** buttons, or via keyboard shortcuts (`W`/`↑`, `S`/`↓`, `Space`/`X`).

---

## ☁️ Deploying to Vercel

Since the frontend is built entirely with pure HTML, CSS, and Vanilla JavaScript with no frameworks:
1. Push this repository to GitHub or run `vercel` in the project root directory.
2. Vercel automatically detects the static site and deploys it immediately with HTTPS.
3. Once deployed, open the live URL in Chrome or Edge and connect to your ESP32 via USB.
