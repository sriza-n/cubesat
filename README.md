# CubeSat Telemetry System

A wireless telemetry system for collecting and transmitting sensor data from a CubeSat to a ground station using RF communication.

## Project Overview

This project consists of two main components:
- **CubeSat.cpp**: Satellite-side code that collects sensor data and transmits it via RF
- **Ground.cpp**: Ground station code that receives and displays telemetry data

## 📁 Project Structure

```
cubesat/
├── CubeSat.cpp              # Satellite transmitter code
├── Ground.cpp               # Ground station receiver code
├── README.md                # This file
├── docs/                    # Documentation and guides
│   ├── circuit-diagram.png  # Wiring diagrams
│   ├── pcb-layout.pdf       # PCB design files
│   └── setup-guide.pdf      # Detailed setup instructions
├── libraries/               # Required Arduino libraries
│   ├── VirtualWire.zip
│   ├── TinyGPS.zip
│   └── MPU6050.zip
└── hardware/               # Hardware specifications
    ├── bom.csv             # Bill of materials
    └── 3d-models/          # 3D printable parts
```

## 📥 Downloads

### Quick Start Files
- 📄 [CubeSat Code](./CubeSat.cpp) - Download satellite code
- 📄 [Ground Station Code](./Ground.cpp) - Download receiver code
- 📋 [Bill of Materials](./hardware/bom.csv) - Component list with part numbers

### Documentation
- 📖 [Setup Guide PDF](./docs/setup-guide.pdf) - Detailed installation instructions
- 🔌 [Circuit Diagram](./docs/circuit-diagram.png) - Wiring schematic
- 🖨️ [PCB Layout](./docs/pcb-layout.pdf) - PCB design files

### Libraries Package
- 📦 [All Required Libraries](./libraries/) - Complete library collection
- 🔗 [MiniCore Bootloader](https://github.com/MCUdude/MiniCore/releases/latest) - External link

## 🖼️ Project Images

### Circuit Diagrams
![CubeSat Wiring Diagram](./docs/circuit-diagram.png)
*Complete wiring schematic for CubeSat sensors and RF transmitter*

### Project Photos
![CubeSat Assembly](./docs/cubesat-assembly.jpg)
*Assembled CubeSat with all sensors mounted*

![Ground Station Setup](./docs/ground-station.jpg)
*Ground station with RF receiver and Arduino Nano*

## Hardware Requirements

### CubeSat (Transmitter)
- **Microcontroller**: ATmega328P
- **Sensors**:
  - LM35 Temperature Sensor (Pin A0)
  - LDR Light Sensor (Pin A1)
  - BMP180 Barometric Pressure Sensor (I2C)
  - MPU6050 IMU with DMP (I2C)
  - NEO-6M GPS Module (SoftwareSerial, Pins 4-5)
- **RF Transmitter**: 433MHz (Pin 10)
- **Power**: 3.3V/5V compatible

### Ground Station (Receiver)
- **Microcontroller**: Arduino Nano (ATmega328P)
- **RF Receiver**: 433MHz (Pin 12)
- **Communication**: USB Serial (9600 baud)

## Software Setup

### ATmega328P Bootloader Installation

The ATmega328P requires a compatible bootloader for Arduino IDE programming:

1. Download MiniCore from: [https://github.com/MCUdude/MiniCore](https://github.com/MCUdude/MiniCore)
2. Install MiniCore in Arduino IDE:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json`
   - Tools → Board Manager → Search "MiniCore" → Install
3. Select board: Tools → Board → MiniCore → ATmega328
4. Configure settings:
    - Everything default except (LTO disabled)

### Quick Library Installation

**Option 1: Download Library Pack**
1. Download [libraries folder](./libraries/) from this repository
2. Extract to your Arduino libraries folder (`~/Documents/Arduino/libraries/`)

**Option 2: Manual Installation**
Install the following libraries through Arduino IDE Library Manager:

**For CubeSat:**
- VirtualWire
- TinyGPS
- I2Cdev
- MPU6050 (by Jeff Rowberg)
- Adafruit BMP085 Library
- Adafruit Unified Sensor

**For Ground Station:**
- VirtualWire

## Pin Configuration

### CubeSat Connections
```
ATmega328P    |  Component
--------------|------------------
A0            |  LM35 (Temp)
A1            |  LDR (Light)
Pin 4         |  GPS RX
Pin 5         |  GPS TX
Pin 10        |  RF Transmitter
SDA (A4)      |  BMP180 & MPU6050 SDA
SCL (A5)      |  BMP180 & MPU6050 SCL
LED_BUILTIN   |  Status LED
```

### Ground Station Connections
```
Arduino Nano  |  Component
--------------|------------------
Pin 12        |  RF Receiver
```

## Data Format

The system transmits 11 float values (44 bytes total):

| Index | Parameter | Unit | Description |
|-------|-----------|------|-------------|
| 0     | Temperature | °C | LM35 sensor reading |
| 1     | Light | % | LDR sensor (0-100%) |
| 2     | Pressure | PSI | BMP180 barometric pressure |
| 3     | Altitude | m | Calculated altitude |
| 4     | Quat_w | - | Quaternion W component |
| 5     | Quat_x | - | Quaternion X component |
| 6     | Quat_y | - | Quaternion Y component |
| 7     | Quat_z | - | Quaternion Z component |
| 8     | Latitude | degrees | GPS coordinate |
| 9     | Longitude | degrees | GPS coordinate |
| 10    | Marker | - | Packet validation (0xAAAA) |

## Usage

### Uploading Code

1. **CubeSat (ATmega328P)**:
   - Select "ATmega328" from MiniCore boards
   - Upload CubeSat.cpp using ISP programmer
   
2. **Ground Station (Arduino Nano)**:
   - Select "Arduino Nano" board
   - Upload Ground.cpp via USB

### Operation

1. Power on both systems
2. CubeSat will initialize sensors and begin transmitting data
3. Ground station will receive and display formatted telemetry
4. Monitor Serial output on ground station (9600 baud)

### Expected Output Format
```
{Temp:25.3,Light:67,Press:14.7,Alt:1400.5,Quat_w:0.98,Quat_x:0.12,Quat_y:0.05,Quat_z:0.15,Lat:27.656908,Lon:85.327476}
```

## Features

- **Real-time telemetry**: Continuous sensor data transmission
- **GPS tracking**: Location monitoring with fallback coordinates
- **Orientation sensing**: Quaternion-based attitude determination
- **Environmental monitoring**: Temperature, pressure, altitude, light
- **Packet validation**: Built-in data integrity checking
- **Status indication**: LED feedback for transmission status

## Troubleshooting

### GPS Issues
- Check wiring (RX/TX may need swapping)
- Allow 2-3 minutes for GPS fix outdoors
- Default coordinates (27.656908, 85.327476) used when no fix

### RF Communication
- Verify transmitter/receiver frequency match (433MHz)
- Check antenna connections
- Ensure adequate power supply
- Maximum range: ~100m line-of-sight

### Sensor Calibration
- MPU6050 auto-calibrates on startup
- Keep device stationary during initialization
- Temperature readings may need offset adjustment

## 🔗 External Resources

- [MiniCore GitHub Repository](https://github.com/MCUdude/MiniCore) - ATmega328P bootloader
- [Arduino IDE Downloads](https://www.arduino.cc/en/software) - Development environment

## 📋 Downloads

- 🚀 [Complete Project Package](https://github.com/sriza-n/cubesat/releases/download/1.1/cubesat.zip) - Full project with code, libraries, and documentation



