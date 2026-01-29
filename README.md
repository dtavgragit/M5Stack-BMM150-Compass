# M5Stack Advanced Compass (BMM150 Magnetometer) 🧭

A collection of visual compass implementations for the M5Stack (ESP32) platform, utilizing the BMM150 3-axis digital geomagnetic sensor.

## 🚀 Overview
This project demonstrates how to interface with the BMM150 sensor via I2C to create various visual styles of digital compasses. It includes advanced features like data smoothing through circular buffers and NVS (Non-Volatile Storage) for calibration offsets.

## 🛠️ Key Features
- **Sensor Fusion:** Raw data processing from BMM150 via I2C (`Wire.h`).
- **Data Smoothing:** Implementation of a **Circular Buffer** to average readings and reduce needle jitter.
- **Persistent Calibration:** Saves and loads magnetometer offsets using the `Preferences.h` library to ensure accuracy after reboot.
- **Trigonometric Rendering:** Real-time calculation of heading and UI elements using `atan2`, `sinf`, and `cosf`.

## 🎨 Compass Styles
The repository includes three distinct visualization modes:
1. **Advanced:** A professional-grade dial with degree markers and cardinal points.
2. **Circle:** A minimalist design featuring a rotating red dot on a white disc.
3. **Triangle:** A classic navigation style with a solid red triangular needle.

## ⚙️ Hardware Requirements
- **M5Stack Core** (Gray/Basic) with built-in or external BMM150.
- **Arduino IDE** or **PlatformIO**.
- **Libraries:** `M5Stack`, `M5_BMM150`.

## 🕹️ Usage
1. Upload the desired `.ino` file to your M5Stack.
2. **Calibration:** Press **Button A** to start the 10-second calibration process. Rotate the device in all directions until finished.
3. The offsets will be saved automatically for future use.

## 🎓 Context
Project developed as part of the *Digital Technology and Multimedia* degree at the **Polytechnic University of Valencia (UPV)**.
