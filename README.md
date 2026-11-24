# VRXFILE-FW-TWatch-S3 (v2.0.0)

Alternative single-file Arduino firmware (`VRXFILE-FW-TWatch-S3.ino`) for the **LILYGO T-Watch S3 Plus** smartwatch. It provides a flicker-free 240×240 graphical interface with **36 built-in applications** across 4 main menu pages, covering timekeeping, GNSS navigation, RF/network diagnostics, audio and vibration FFT spectrometry, offline references, weather, and emergency tools.

---

## Build & Installation

### Requirements

- **Arduino IDE** with the **ESP32** board support package (compatible with both ESP32 Arduino Core 3.x / ESP-IDF v5 and Core 2.x).
- Two external libraries installed via the Arduino IDE **Library Manager**:
  1. **LovyanGFX** (by `lovyan03`)
  2. **TinyGPSPlus** (by `Mikal Hart`)

### Pre-Build Configuration

> **WeatherAPI Key Required:** For the **Weather** application to work properly, generate your own API key on WeatherAPI and set it in the `WEATHER_API_KEY` constant inside `VRXFILE-FW-TWatch-S3.ino` before compiling:
> 
> ```cpp
> #define WEATHER_API_KEY "YOUR_WEATHERAPI_KEY"
> ```

### First Boot

On startup, the firmware loads saved settings from Flash, initializes the AXP2101 PMU rails, buzzes the DRV2605L haptic motor for 500 ms, configures the BMA423 accelerometer, FT6236/FT6336U touch controller, MIA-M10Q GPS (38400 baud), ST7789V display, and IR LEDC transmitter, then opens Page 1 of the main menu.

---

## Hardware Pinout (T-Watch S3 Plus)

| Component                     | Bus / Address        | GPIO Pins                                  | Notes                                             |
| ----------------------------- | -------------------- | ------------------------------------------ | ------------------------------------------------- |
| **ST7789V** Display (240×240) | SPI2 (3-wire DMA)    | MOSI: 13, SCLK: 18, DC: 38, CS: 12, BL: 45 | 40 MHz write, 12 kHz PWM backlight                |
| **AXP2101** PMU               | I2C0 (`0x34`)        | SDA: 10, SCL: 11                           | 3.3 V LDO rails, charge current, fuel gauge       |
| **PCF8563** RTC               | I2C0 (`0x51`)        | SDA: 10, SCL: 11                           | 1 Hz polling                                      |
| **BMA423** Accelerometer      | I2C0 (`0x18`/`0x19`) | SDA: 10, SCL: 11                           | 50 Hz default / 200 Hz vibration FFT, temp sensor |
| **DRV2605L** Haptic Driver    | I2C0 (`0x5A`)        | SDA: 10, SCL: 11                           | 3.0 V ERM 0827 motor (Open-Loop RTP)              |
| **FT6236 / FT6336U** Touch    | I2C1 (`0x38`)        | SDA: 39, SCL: 40, INT: 16                  | 60 Hz scan, reg. `0x80` threshold (30–80)         |
| **MIA-M10Q** GNSS             | UART1 (`8N1`)        | RX: 41, TX: 42, PPS: 0                     | Fixed 38400 baud, NMEA 0183                       |
| **IR12-21C** IR Emitter       | LEDC (T1 / Ch2)      | TX: 2                                      | 33 / 36 / 38 kHz carrier (NEC, RC-5, RC-6)        |
| **SPM1423HM4H-B** PDM Mic     | I2S0 (PDM RX)        | CLK: 44, DATA: 47                          | 16 kHz, 16-bit mono                               |
| **MAX98357A** Speaker Amp     | I2S1 (STD TX)        | BCK: 48, WS: 15, DOUT: 46                  | 22050 Hz, 16-bit stereo (Core 0 task)             |

---

## Navigation, Power & Flash Storage

- **Menu Navigation:** 4 pages of 9 hexagonal icons (3×3). Top-left/top-right arrows switch pages; inside apps, the top-left red **X** returns to the main menu.
- **Backlight & Wake:** Active brightness (5%–100%, default 94%) dims by 50% after 10 s and turns off after the configured timeout (default 30 s; `0 s` = always on). Wake with a **double tap** (`2x TAP`, default) or **triple tap** (`3x TAP`).
- **Background Tasks:** Continuous 1 Hz RTC/battery/temp polling, 50 Hz step counting, GPS parsing, 2 alarms + countdown timer monitoring, and 60 s Wi-Fi auto-reconnect.
- **Flash Persistence (`Preferences`):** `watch_cfg` (system settings, step count saved every 10 min, weather mode/city, alarms/timer), `wifi_creds` (up to 64 networks in LIFO order), and `paint_img` (120×86 4-bit canvas).

---

## Built-in Applications (36 Total)

### Page 1 — Utilities & Navigation

- **Calendar:** 2000–2099 monthly grid with weekend and current-date highlights.
- **Calc:** 4-page scientific calculator (12-digit precision, `±∞`, complex roots, factorials, trig/hyperbolic/inverse hyperbolic functions, `rnd`).
- **Clock:** RTC viewer/editor (`UTC-12`..`UTC+14`) with one-tap **GPS** and **NTP** (`pool.ntp.org` / `time.nist.gov`) sync.
- **GPS:** 3-page GNSS monitor — telemetry + HDOP, polar satellite skyplot (GPS/GLONASS/Galileo/BeiDou), and SNR bar chart.
- **IR remote control:** Hex code keypad (up to 8 digits), camera-visible **Test LED**, protocol (`NEC`/`RC-5`/`RC-6`) and carrier (`33`/`36`/`38 kHz`) settings.
- **Step counter:** Background BMA423 pedometer with Flash persistence and zero reset.
- **Inclinometer:** 20 Hz dual-axis digital bubble level with ±1.5° level indicator.
- **Earth / Satellites:** Interactive 3D globe with lighting, GPS pin, and live J2 orbital tracking for **Kinosputnik** and the **ISS** (NORAD 25544).
- **Settings:** Wi-Fi auto-connect, display brightness, screen timeout, touch vibration, touch sensitivity (`0x80`) & wake gesture (`2x`/`3x`), AXP2101 charge current (`100`–`400 mA`), and independent **GPS (`BLDO1`) / LoRa (`ALDO4`)** power rail toggles.

### Page 2 — Networking, RF & Audio

- **Wi-Fi connect:** Scans up to 32 APs, 4-page on-screen keyboard, LIFO Flash save.
- **Wi-Fi list:** View saved SSIDs/passwords, reconnect, or delete entries.
- **Wi-Fi analyzer:** Real-time 2.4 GHz channel load monitor (Ch 1–11).
- **Wi-Fi info:** 14 connection metrics including internet probe, PHY modes, RSSI, OUI vendor, auth, WPS, and ciphers.
- **Wi-Fi jam detect:** Promiscuous-mode Deauth/Disassoc frame rate monitor (0–250 pkt/s, Ch 1–11).
- **Hosts scan:** Non-blocking `/24` LAN ARP scanner with NetBIOS lookup, OUI vendor, RTT, and TCP port probes (`22, 53, 80, 443, 8080`).
- **Bluetooth scan:** Active 3-second BLE 5.0 scanner with RSSI, TX power, Company ID/OUI vendor, appearance, and UUID details.
- **Sound spectrometer:** PDM mic 256-pt FFT (62.5 Hz–8 kHz) with Peak Hold and Spectroid waterfall.
- **Sound generator:** 20 Hz–12 kHz synthesizer (`Sine`, `Square`, `Tri`, `Saw`, `White`, `Pink`) with volume and step controls.

### Page 3 — References, Telemetry & Weather

- **ASCII table:** Full 0–255 ASCII + Windows-1251 Cyrillic table (`DEC`, `HEX`, `CHR`).
- **Periodic table:** All 118 elements across 4 pages with physical/chemical property cards.
- **Food e-additives:** 100 additives (`E100`–`E967`) with hazard ratings and notes.
- **Temp. sensor:** Dual live chart (ESP32 die + BMA423) with selectable `°C`, `K`, and `°F` units.
- **Accel. sensor:** Live 3-axis (`X`, `Y`, `Z`) acceleration chart and readouts in `m/s²`.
- **Sun activity:** Live NOAA SWPC space weather (`Kp`, `R/S/G` scales, solar wind, IMF `Bt/Bz`, 3-day forecast, 30-day `F10.7` flux, and 24-hour Kyoto `Dst`).
- **Weather:** WeatherAPI current conditions, astronomy (sunrise/sunset/moon phase), 7-day forecast, `AUTO` (Wi-Fi IP with GPS fallback) or `MANUAL` mode (53 world capitals). *Requires your own `WEATHER_API_KEY`.*
- **System info:** 20 live parameters (ESP32 CPU/Heap/PSRAM/Flash/Serial, temperatures, battery stage, and AXP2101 `VBAT`/`VBUS`/`VSYS`/rails).
- **About firmware:** Firmware version, target device, and author details.

### Page 4 — Timekeeping, Signaling, Games & Vibration

- **Alarm:** 2 independent alarms with `Beep`/`Bell` ringtones, configurable Snooze (5–30 min), and wake-on-alert modal.
- **Stopwatch:** 0.01 s precision with background counting.
- **Timer:** Countdown up to `23:59:59` with wake-on-finish alert modal.
- **Flashlight:** 8-color screen floodlight with independent 5%–100% brightness slider (keeps screen awake).
- **SOS signal:** Continuous ITU `... --- ...` beacon using max-volume 2400 Hz tone, 100% screen strobe, and DRV2605L vibration.
- **Morse code:** 59-character EN (`A–Z`) + RU (`А–Я`, `Ё`) table with tap-to-play 800 Hz CW audio.
- **Tetris:** 10×20 well, wall-kick rotation, ghost piece, next preview, levels 1–15, and pause.
- **Paint:** 120×86 canvas (2×2 scaled), 10 brush colors, 6 backgrounds, 4 widths, 4 brush types (`RND`/`SQR`/`SPR`/`DSH`), and Flash save/auto-load.
- **Accel. spectrometer:** 200 Hz BMA423 128-pt vibration FFT (1.56–100 Hz) with Peak Hold, waterfall, and selectable `MAG`/`X`/`Y`/`Z` axis.

---

## Troubleshooting

- **Screen won't wake on a single tap:** Use a quick double tap (`2x TAP`) or triple tap (`3x TAP`) spaced 60–450 ms apart.
- **GPS shows `Error` / `No GPS UART`:** Enable `GPS power (BLDO1)` in *Settings → Power settings* and test outdoors.
- **Weather / Sun activity / NTP fails:** Verify Wi-Fi connection and internet access in *Wi-Fi info*, and confirm your personal `WEATHER_API_KEY` is configured for *Weather*.
- **Wi-Fi disconnects in *Wi-Fi jam detect*:** Normal behavior during promiscuous channel hopping; exits automatically reconnect.

---

## Author & Support

- **Author:** Rostislav Varzar
- **Email:** `vrxfile@mail.ru` / `vrxfile1987@gmail.com`