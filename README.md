# 🔭 ESP32 Smart Astro Box

**Ultimate Edition v6.3**

An All-in-One observatory controller powered by ESP32. This device combines a Focuser, Dual Dew Heater Controller, Weather Station, and Status Display into a single unit, fully compatible with **ASCOM Alpaca** and **N.I.N.A.**

## ✨ Features

* **📡 ASCOM Alpaca Wireless:** No USB cables needed! Connects via WiFi (Static IP).
* **⚙️ Stepper Motor Focuser:** Absolute positioning with memory (saves position on power loss).
* **🌡️ Environmental Monitor:** Measures Temperature, Humidity, Pressure, and Dew Point (BME280).
* **🔥 Dual Smart Dew Heater:** * 2x PWM Channels (RCA outputs).
    * **Auto-Dew Algorithm:** Automatically adjusts power based on the Dew Point delta.
* **🖥️ OLED Status Display:** 1.5" Screen (128x128) showing IP, Weather Data, and Focuser Position.
* **🛡️ Crash-Proof:** Dual-Core processing and "Lazy Memory" writing ensures stable WiFi connection.

## 🛠️ Hardware & Pinout

**Microcontroller:** ESP32 Dev Kit V1

| Component | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **Stepper Step** | GPIO 14 | To Driver (DIR) |
| **Stepper Dir** | GPIO 12 | To Driver (STEP) |
| **Stepper Enable**| GPIO 13 | To Driver (EN) |
| **Heater 1** | GPIO 25 | PWM Output (MOSFET) |
| **Heater 2** | GPIO 26 | PWM Output (MOSFET) |
| **I2C SDA** | GPIO 21 | BME280 Sensor & OLED |
| **I2C SCL** | GPIO 22 | BME280 Sensor & OLED |

## 📦 Required Libraries

To compile this project in Arduino IDE, install the following libraries:

1.  **AccelStepper** (by Mike McCauley)
2.  **ArduinoJson** (by Benoit Blanchon)
3.  **Adafruit BME280 Library**
4.  **Adafruit SH110X** (for the 1.5" OLED)
5.  **Adafruit GFX Library**

## 🚀 Setup Instructions

1.  **WiFi Configuration:** Edit the `ssid`, `password`, and `local_IP` variables in `main.ino` to match your network.
2.  **Upload:** Use Arduino IDE to upload the sketch to your ESP32.
3.  **Connect:** Open N.I.N.A. (or any ASCOM software).
    * **Focuser:** Select "Alpaca Focuser" -> Enter IP `192.168.8.10` Port `4567`.
    * **Switch:** Select "Alpaca Switch" -> Same IP/Port (Controls Heaters).
    * **Weather:** Select "Alpaca Observing Conditions" -> Same IP/Port.

## 📊 Status Display

The OLED screen provides real-time feedback:
* **IP Address:** Never lose your device on the network.
* **Weather:** Temp (C), Humidity (%), Pressure (hPa), Dew Point (C).
* **Focus:** Current absolute position.

---
*Created by Jan*
