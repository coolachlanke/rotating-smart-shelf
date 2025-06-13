# Rotating Smart Shelf

Motorized display shelf for Rubik’s Cubes and other collectibles, powered by ESP32-C3 and WS2812B RGB LEDs.  
Designed for showpieces, timed rotations and eye-catching animations.

---

## 🧰 Tech Stack

- **Microcontroller:** ESP32-C3 (BLE, WiFi)
- **PCBs:** Custom power distrubution/driver/logic board + annular 48-pixel WS21812B carrier board. All designed in KiCAD 8.
- **Motors:** NEMA17 bipolar stepper motors with A4988 drivers
- **Power:** 9V barrel input with onboard 5V & 3V3 voltage regulation
- **Firmware:** Custom C++ firmware utiliting FreeRTOS built with PlatformIO (Arduino framework)
- **Lighting:** WS2812B addressable RGB LED rings for reactive effects
- **Control App:** Web-app interface for wireless control and presets
- **Features:**
  - Smooth timed rotation of platforms
  - Colour animations, brightness modulation, visual syncing
  - [UNDER CONSTRUCTION] Optional music-reactive visuals (via onboard mic or wireless data stream)

---

> Designed by Lachlan Cooke in late 2024.
> Check out [lachlancooke.com/projects](https://www.lachlancooke.com/projects) für mehr details.
