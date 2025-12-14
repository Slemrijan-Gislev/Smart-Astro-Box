# Smart-Astro-Box
ESP32 ASCOM Alpaca Smart Astro Box
# ESP32 ASCOM Alpaca Astro Box

A DIY "Smart Astro Box" that controls a telescope focuser, dew heaters, and provides weather data via WiFi. 

This project uses the **ASCOM Alpaca** protocol, meaning no drivers are required on the PC. It works natively with N.I.N.A., SGP, and other astrophotography software.

## Features

* **Wireless Focuser:** Controls a stepper motor (NEMA 17) for precise focusing.
* **Dual Dew Controller:**
    * **Channel 1 (Primary):** Smart Automatic Dew Control using the BME280 sensor (maintains mirror temp just above dew point).
    * **Channel 2 (Aux):** Manual slider control via software (e.g., for Guide Scope).
* **Weather Station:** Reports Temperature, Humidity, and Dew Point to your imaging software (stores environment data in FITS headers).
* **Driverless:** Connects via WiFi (ASCOM Alpaca).

## Hardware Required

* **ESP32 Development Board** (e.g., ESP32-WROOM-32)
* **Stepper Driver:** DRV8825 or A4988
* **Stepper Motor:** NEMA 17
* **MOSFET Modules:** 2x IRLZ44N or similar logic-level MOSFET modules (for heater control)
* **Sensor:** BME280 (I2C version)
* **Power:** 12V DC power supply + LM2596 Buck Converter (12V -> 5V for ESP32)

## Wiring / Pinout

| Component | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **Stepper Step** | GPIO 14 | |
| **Stepper Dir** | GPIO 12 | |
| **Stepper Enable** | GPIO 13 | |
| **Heater 1 (Main)**| GPIO 25 | PWM output to MOSFET Gate |
| **Heater 2 (Aux)** | GPIO 26 | PWM output to MOSFET Gate |
| **BME280 SDA** | GPIO 21 | I2C Data |
| **BME280 SCL** | GPIO 22 | I2C Clock |
| **BME280 VCC** | 3.3V | **IMPORTANT:** Do not use 5V! |

*Note: Ensure common ground between the 12V power source and the ESP32.*

## Installation

1.  Open the project in **Arduino IDE** or **PlatformIO**.
2.  Install the required libraries:
    * `WiFi`
    * `ArduinoJson`
    * `AccelStepper`
    * `Adafruit BME280 Library`
    * `Adafruit Unified Sensor`
3.  **Configuration:**
    * Edit the `ssid` and `password` variables in the code to match your WiFi network.
    * (Optional) Set a static IP in the `local_IP` configuration to ensure the device address remains constant.
4.  Upload the code to your ESP32.

## Connecting to N.I.N.A. / ASCOM

1.  Make sure your PC and ESP32 are on the same WiFi network.
2.  Open **N.I.N.A.**
3.  **Focuser:** Go to Equipment -> Focuser, select "Alpaca Driver", and input the IP address (Port: 4567).
4.  **Switch:** Go to Equipment -> Switch, select "Alpaca Driver" to control the dew heaters.
5.  **Weather:** Go to Equipment -> Weather, select "Alpaca Driver" to get environmental data.

## License

This project is open source. Feel free to modify and share!
