# ESP32 Quadcopter Flight Controller with GPS Hold

**Full flight controller firmware for ESP32-based quadcopter with MPU6050, BME280, GPS NEO-6M, and iBUS receiver support.**

---

## 📌 What is This?

This is the code I wrote for my ESP32-based quadcopter. It's not a commercial product – just my own flight controller that actually flies.

### What it does:
- **Reads MPU6050** (gyro + accelerometer) to figure out Roll, Pitch, and Yaw attitude.
- **Mahony complementary filter** for smooth, drift-free orientation estimation.
- **Reads BME280** for temperature, air pressure, and relative altitude (to know how high it is).
- **GPS Hold** – when I flip a switch on my transmitter, the drone tries to stay in one spot using GPS.
- **Cascaded PID** (angle loop + rate loop) for Roll and Pitch, and a separate PID for Yaw.
- **Air-density adaptive gains** – automatically adjust PID for altitude changes (optional).
- **Dynamic notch filter** – removes motor vibration noise from gyro signals.
- **Safety stuff** – won't arm unless it's level and throttle is down. Also limits how fast motor signals change.

---

## 🛠️ Main Features

- ✅ **Mahony AHRS** – SO(3) complementary filter for quaternion-based attitude estimation  
- ✅ **In-flight gyro bias estimation** – automatically compensates for temperature drift  
- ✅ **Dynamic Notch Filter** – 4-stage bank tracks motor RPM to remove vibrations  
- ✅ **PT1 Low-Pass Filter** – additional smoothing on gyro signals (80 Hz cutoff)  
- ✅ **Air-Density Adaptive PID** – scales gains with atmospheric density (CH6 selectable)  
- ✅ **Cascaded PID** – outer angle loop + inner rate loop for responsive control  
- ✅ **D-on-measurement** – reduces derivative kick from noisy setpoints  
- ✅ **Anti-windup** – conditional integration with back-calculation  
- ✅ **GPS Hold** – position hold using GPS coordinates (NEO-6M)  
- ✅ **Altitude Hold** – barometric altitude hold via BME280  
- ✅ **iBUS Receiver** – works with FlySky/Turnigy iBUS protocol  
- ✅ **Bluetooth Telemetry** – stream live flight data to phone  
- ✅ **Ground Effect Compensation** – reduces PID aggression near the ground  
- ✅ **Gyro Calibration** – automatic at boot  

---

## 🔌 Wiring (Quick Reference)

| Component   | ESP32 Pin | Notes                          |
|-------------|-----------|--------------------------------|
| **MPU6050** | SDA=21, SCL=22 | I²C bus (shared with BME280) |
| **BME280**  | SDA=21, SCL=22 | I²C bus (shared with MPU6050) |
| **GPS**     | RX=17, TX=5  | UART1 serial connection       |
| **iBUS RX** | RX=16        | UART2 serial connection       |
| **ESC M1**  | 13           | Rear-Right (CCW)              |
| **ESC M2**  | 14           | Front-Right (CW)              |
| **ESC M3**  | 25           | Rear-Left (CW)                |
| **ESC M4**  | 26           | Front-Left (CCW)              |

> **Note:** PWM range = 1000–2000 µs, idle at 1050 µs.

---

## 📦 Bill of Materials (BOM)

| Component              | Model / Spec              | Quantity |
|------------------------|---------------------------|----------|
| Flight Controller      | ESP32 Dev Module          | 1        |
| IMU                    | MPU6050                   | 1        |
| Barometer              | BME280                    | 1        |
| GPS                    | NEO-6M (or NEO-M8N)       | 1        |
| Receiver               | FlySky iBUS compatible    | 1        |
| ESCs                   | 30A BLHeli_S (DShot or PWM) | 4      |
| Motors                 | 2212 930KV                | 4        |
| Propellers             | 1045 or 1245 (2-blade)    | 2 CW + 2 CCW |
| Battery                | 3S 2200mAh LiPo           | 1        |
| Frame                  | 500mm X-frame             | 1        |

---

## 📝 Channel Mapping (FlySky iBUS)

| Channel | Function                | Range       |
|---------|-------------------------|-------------|
| CH0     | Roll (Aileron)          | 1000–2000   |
| CH1     | Pitch (Elevator)        | 1000–2000   |
| CH2     | Throttle                | 1000–2000   |
| CH3     | Yaw (Rudder)            | 1000–2000   |
| CH4     | Arm / Disarm            | <1300 = Disarm, >1700 = Arm |
| CH5     | PID Mode                | <1300 = FIX, >1300 = ADAPTIVE |

---

## 🚀 Getting Started

### What You Need
- ESP32 Dev Board
- MPU6050 (I²C)
- BME280 (I²C)
- GPS module (NEO‑6M or similar, UART)
- iBUS receiver (UART)
- 4x ESCs + motors (X configuration)
- LiPo battery (3S or 4S)

### Software Setup

1. **Install Arduino IDE** (2.0 or newer).
2. **Add ESP32 support**:
   - Tools → Board → Boards Manager → Search "ESP32" → Install
3. **Install these libraries** (all from Library Manager):
   - Adafruit MPU6050
   - Adafruit BME280
   - Adafruit Sensor
   - ESP32Servo
   - TinyGPSPlus
4. **Download this repo** (clone or zip).
5. Open `quadcopter-code-adptive-pid.ino` in Arduino IDE.
6. Select board: **ESP32 Dev Module**.
7. Upload!

### Arming & Calibration

1. **Gyro calibrates itself** in the first few seconds – **keep the drone still** on a level surface.
2. **AHRS zeroing** – after calibration, the current orientation becomes "level" (Roll/Pitch/Yaw = 0°).
3. **To arm**:
   - Throttle stick at minimum.
   - Flip CH4 (ARM switch) to **high** (1700+ µs).
   - Motors will spin at idle speed (1050 µs).
4. **To disarm**: Flip CH4 back to low.
5. **Tilt safety** – if Roll or Pitch exceeds 50°, motors disarm immediately.

> **WARNING:** Always remove propellers when testing on the ground!

---

## 🎮 RC Transmitter Setup

- **Mode 2** (Throttle + Yaw on left, Roll + Pitch on right)
- **Throttle trim** – set to neutral (center)
- **Endpoints** – 1000–2000 µs for all channels
- **Arm switch** – CH5 (or CH4 depending on your receiver) assigned to a 2-position switch
- **Mode switch** – CH6 assigned to a 2-position switch:
  - **Low (≤1300)** = FIX gains
  - **High (>1300)** = ADAPTIVE gains (air-density compensated)

---

## 🧪 PID Tuning (My Current Gains)

I tuned these for a typical 500mm frame with 2212 930KV motors and 1245 props. Feel free to adjust:

| Parameter                  | Variable in Code       | Value    | Description                          |
|----------------------------|------------------------|----------|--------------------------------------|
| Angle Roll Kp              | `cfgAngR.Kp`           | 0.45     | Outer loop – angle response         |
| Angle Pitch Kp             | `cfgAngP.Kp`           | 0.45     | Outer loop – angle response         |
| Angle Roll/Pitch Kd        | `cfgAngR.Kd` / `cfgAngP.Kd` | 0.005 | Damping on angle loop             |
| Rate Roll Kp               | `cfgRateR.Kp`          | 0.105    | Inner loop – rate response          |
| Rate Pitch Kp              | `cfgRateP.Kp`          | 0.105    | Inner loop – rate response          |
| Rate Roll Kd               | `cfgRateR.Kd`          | 0.00730  | Damping on rate loop                |
| Rate Pitch Kd              | `cfgRateP.Kd`          | 0.00753  | Damping on rate loop                |
| Rate Roll/Pitch Ki         | `cfgRateR.Ki` / `cfgRateP.Ki` | 0.000   | Integral (start at 0, add later) |
| Yaw Rate Kp                | `cfgYaw.Kp`            | 1.20     | Yaw rate response                   |
| Altitude Kp                | `cfgAlt.Kp`            | 1.50     | Altitude hold Kp                    |
| Altitude Ki                | `cfgAlt.Ki`            | 0.080    | Altitude hold integral              |
| Altitude Kd                | `cfgAlt.Kd`            | 0.800    | Altitude hold damping               |
| GPS Hold Kp                | `cfgGps.Kp`            | 0.60     | Position hold Kp                    |
| GPS Hold Ki                | `cfgGps.Ki`            | 0.010    | Position hold integral              |
| GPS Hold Kd                | `cfgGps.Kd`            | 0.080    | Position hold damping               |

### My PID Tuning Process

1. **Start with Ki = 0** everywhere until flight is stable.
2. **KpRate** → increase in small steps (0.02) until wobble starts, then back off 20%.
3. **KdRate** → add gradually in 0.002 steps to damp oscillation.
4. **KpAng** → 0.45 gives good response; KdAng = 0.005 prevents overshoot.
5. **Finally add KiRate** → start at 0.005, increase slowly if stable.

---

## 📡 Telemetry & Debug

The firmware streams live telemetry over both Serial (USB) and Bluetooth:

### Serial Output (115200 baud)

```
ARM=ON MODE=ADAPT R/P/Y=2.3/-1.1/45.6 gxyz=12.5/-8.3/3.2 THR=1450 M=1250,1230,1245,1260 uRPY=15,-12,0 T=28.5C P=1012.3 Alt=0.45m H=55.2% rho=1.193 ratio=0.974 RPM=6200,6100,6150,6250 ...
```

### Bluetooth Telemetry

- **SSID:** `Quadcopter_Adam`
- **App:** Serial Bluetooth Terminal (Android)
- **Baud:** 115200 (auto)

---

## ⚙️ Configuration Options

### Physical Constants (Adjust for Your Build)

```cpp
static const float PROP_DIAM_M  = 0.3048f;   // 12 inch prop diameter
static const float V_BATT       = 11.1f;     // 3S LiPo nominal voltage
static const float MASS_KG      = 1.4f;      // total weight
static const float KV_RPM       = 930.0f;    // motor KV rating
```

### Filter Settings

```cpp
static const float GYRO_LPF_HZ = 80.0f;   // Gyro low-pass cutoff
static const float NOTCH_Q     = 5.0f;    // Notch filter quality factor
static const int   PROP_BLADES = 2;       // Propeller blade count
```

### PID Mode Switching

- **CH6 ≤ 1300** → FIX gains (base values)
- **CH6 > 1300** → ADAPTIVE (scales by 1/ρ_ratio)

---

## 🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| **MPU6050 not found** | Check I²C wiring (SDA/SCL), pull-ups, and power (3.3V) |
| **Gyro not calibrating** | Keep drone perfectly still during first 4 seconds |
| **Motors won't arm** | Throttle must be at minimum, Roll/Pitch within ±25°, CH4 > 1700 |
| **Oscillations / wobble** | Lower `GYRO_LPF_HZ` to 60 Hz, reduce KpRate, increase KdRate |
| **Drift during hover** | In-flight bias estimator works at low rates (<4°/s) – wait 10 seconds |
| **Altitude hold unstable** | Reduce Alt Kp, increase Alt Kd (damping), ensure BME280 is reading correctly |
| **GPS not getting fix** | Wait outdoors, ensure antenna faces sky, check baud rate (9600) |
| **iBUS not receiving** | Check RX pin (16), baud rate (115200), ground connection |

---

## 📈 What I Want to Add Next

- [ ] Log flight data to SD card (CSV format)
- [ ] Waypoint navigation with GPS
- [ ] Return-to-Home (RTH) failsafe
- [ ] Support for DShot ESCs (instead of PWM)
- [ ] OLED display for real-time status
- [ ] Web-based PID tuning interface over WiFi
- [ ] Blackbox logging for post-flight analysis

---

## 📄 License

This project is licensed under **GNU General Public License v3.0** – see the [LICENSE](LICENSE) file.

**In short:** You can use, modify, and share this code freely, but any modified version must also be open source under GPLv3.

---

## 📚 Citation (If You Use This in Your Work)

```bibtex
@software{Adam_ESP32_QuadFlightController_2026,
  author = {Adam Thear Abdel Nabi},
  title = {ESP32 Quadcopter Flight Controller with GPS Hold},
  year = {2026},
  url = {https://github.com/adamtheareng22/ESP32_QuadFlightController}
}
```

---

## 🤝 Contributing

Pull requests and issue reports are welcome! If you find a bug or have an improvement:

1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/amazing-feature`).
3. Commit your changes (`git commit -m 'Add amazing feature'`).
4. Push to the branch (`git push origin feature/amazing-feature`).
5. Open a Pull Request.

---

## 📞 Contact

- **GitHub:** [@adamthaereng22-ops](https://github.com/adamthaereng22-ops)
- **Email:** [adam.thear@example.com](mailto:adam.thear@example.com)

## 🙏 Acknowledgments

- [Mahony et al. (2008)](https://ieeexplore.ieee.org/document/4608934) – Complementary filter theory
- [Betaflight](https://github.com/betaflight/betaflight) – PID tuning inspiration
- [Adafruit](https://www.adafruit.com/) – MPU6050 & BME280 libraries
- [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus) – GPS parsing library
- ESP32 community – for making this possible!

---

*Happy flying! 🚁*

*Built with ❤️ in Iraq*
