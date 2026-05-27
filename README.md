# ESP-NOW Wireless Multi-IMU Platform

Open-source firmware and PC tools for a wireless multi-IMU system built on ESP32. A **Master** node collects time-synchronized IMU data from multiple **Slave** nodes over ESP-NOW and streams it to a PC over USB Serial. Python applications provide live monitoring, CSV logging, post-processing plots, and accelerometer/gyro calibration.

## Hardware overview

Custom carrier PCB with ESP32 module (CAD):

![ESP32 carrier PCB — 3D render](https://raw.githubusercontent.com/caonam-tugraz/MultiIMU-GaitSync/main/3D.png)

Five IMU slave nodes and Master with TFT (5/5 nodes connected):

![Five IMU nodes and Master recorder](https://raw.githubusercontent.com/caonam-tugraz/MultiIMU-GaitSync/main/ss.png)

## Programs included

### Python applications

| Program | Description |
|---------|-------------|
| **`imu_logger_gui.py`** | Main GUI: connect to Master over COM, live IMU plots (pyqtgraph), record IMU/VL53 CSV logs, gap detection, and plot saved CSV files. |
| **`imu_calib.py`** | 6-face accelerometer calibration (+ optional gyro bias and temperature fit). Auto-detects COM port and IMU MAC; sends calibration to slaves via Master. |

### Python support modules (imported by the apps above)

| Module | Role |
|--------|------|
| `imu_serial_codec.py` | Binary/text Serial protocol (58-byte IMU packet + CRC), COM auto-detect, ESP32-friendly port open. |
| `imu_logger.py` | CSV logging helpers and shared constants. |
| `imu_live_plot_core.py` | Live multi-slave IMU plot widget used inside the logger GUI. |
| `imu_log_plotter.py` | Matplotlib CSV plotter (used from the logger GUI “Plot CSV” tab). |

### Firmware

| Folder | Target | Description |
|--------|--------|-------------|
| `firmware/master_tft/` | ESP32 + ST7789 TFT | Master with on-board display (node count, RSSI, battery). |
| `firmware/master_esp32s/` | ESP32-S | Master without TFT; Serial logger only. |
| `firmware/slave_bno055/` | ESP32 + BNO055 | Slave IMU node (accel, gyro, magnetometer, temperature). |
| `firmware/slave_icm42688p/` | ESP32 + ICM42688P | Slave IMU node (raw accel/gyro; mag fields sent as zero). |
| `firmware/common/` | — | Shared `esp_now_time_sync_types.h` (ESP-NOW packet definitions). |

## Directory structure

```
.
├── README.md
├── firmware/
│   ├── common/
│   ├── master_tft/
│   ├── master_esp32s/
│   ├── slave_bno055/
│   └── slave_icm42688p/
└── python/
    ├── imu_logger_gui.py      # main logger GUI
    ├── imu_calib.py           # calibration tool
    ├── imu_serial_codec.py
    ├── imu_logger.py
    ├── imu_live_plot_core.py
    ├── imu_log_plotter.py
    └── requirements.txt
```

## Requirements

**Hardware**

- ESP32 board(s) for Master and one or more Slaves
- IMU: Adafruit BNO055 **or** ICM42688P (plus external ICM42688 library for the ICM42688P sketch)
- Master TFT variant: ST7789 display + Adafruit GFX/ST7789 libraries
- USB cable from Master to PC

**Software**

- [Arduino IDE](https://www.arduino.cc/en/software) 2.x (or PlatformIO) with ESP32 board support
- Python 3.9+ (64-bit recommended on Windows)
- Arduino libraries (install via Library Manager where applicable):
  - Master TFT: Adafruit GFX, Adafruit ST7789
  - Slave BNO055: Adafruit Unified Sensor, Adafruit BNO055
  - Slave ICM42688P: ICM42688 driver used by the sketch (see sketch comments)

**Serial**

- Default baud rate: **921600**
- Master streams binary IMU frames (`A5 5A A5 5A` + 58-byte payload + CRC) when binary mode is enabled on firmware

## Installation

### 1. Flash firmware

1. Open the matching sketch folder in Arduino IDE, e.g. `firmware/master_tft/`.
2. Select the correct ESP32 board and COM port.
3. Install required libraries, then upload.
4. Repeat for each Slave node (`firmware/slave_bno055/` or `firmware/slave_icm42688p/`).
5. Power Slaves; Master should discover them over ESP-NOW (TFT master shows node count / RSSI).

Each firmware folder is a self-contained sketch (`.ino` + local headers).

### 2. Install Python tools

```bash
cd python
python -m pip install -r requirements.txt
```

On Windows, use `py -3` instead of `python` if needed.

## Usage

Connect the **Master** board to the PC via USB. Slaves communicate wirelessly with the Master; the PC only talks to the Master over Serial.

### IMU Logger GUI

```bash
cd python
python imu_logger_gui.py
```

1. Choose the Master COM port (or use auto-detect if available) and connect.
2. **Live IMU** tab — real-time plots per slave MAC.
3. **Record** — saves `imu_log_*.csv` and optional `vl53_log_*.csv` under a `recorded/` folder next to the script.
4. **Plot CSV** tab — open and plot saved IMU logs; optional gap markers for missing samples.

<p align="center">
  <img src="https://raw.githubusercontent.com/caonam-tugraz/MultiIMU-GaitSync/main/Screenshot%202026-05-27%20124552.jpg" width="80%" alt="IMU Logger GUI — Live IMU tab" />
  <br /><em>Live IMU — real-time accel, gyro, and |a| plots for multiple slaves</em>
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/caonam-tugraz/MultiIMU-GaitSync/main/Screenshot%202026-05-27%20124622.jpg" width="80%" alt="IMU Logger GUI — Serial logger tab" />
  <br /><em>Serial logger — COM connect, log, GAP detection, per-MAC stats (battery, RSSI)</em>
</p>

### IMU Calibration

```bash
cd python
python imu_calib.py
# or specify port explicitly:
python imu_calib.py COM18 921600
```

1. Connect to the Master COM port (auto-scan if no port is given).
2. Select the slave MAC to calibrate.
3. Follow the 6-face procedure (+X, −X, +Y, −Y, +Z, −Z); keep the module still on each face.
4. Review computed bias/scale, then **Send Calib To Module** to apply on the slave via Master.

### Typical workflow

1. Flash Master + Slaves → verify ESP-NOW link on Master.
2. Run `imu_calib.py` → calibrate each slave.
3. Run `imu_logger_gui.py` → live check → record CSV during experiment.
4. Plot and analyze CSV from the GUI or export for external tools.

## License

This project is licensed under the [MIT License](LICENSE).
