# Complete Quadcopter Flight Controller Architecture
## Raspberry Pi Pico W + MPU6050 + 4 Motors

---

## 📋 Table of Contents
1. [Hardware Components](#hardware-components)
2. [Pin Connections](#pin-connections)
3. [Power System Architecture](#power-system-architecture)
4. [Circuit Diagram Description](#circuit-diagram-description)
5. [Software Architecture](#software-architecture)
6. [Code Implementation](#code-implementation)
7. [WiFi Control Interface](#wifi-control-interface)
8. [Calibration & Testing](#calibration--testing)

---

## 1. Hardware Components

### Required Components:
- **Raspberry Pi Pico W** (1x) - Main flight controller
- **MPU6050** (1x) - 6-axis IMU (gyroscope + accelerometer)
- **Brushed DC Motors** (4x) - 8520 or similar (or brushless with ESCs)
- **Motor Drivers** - L298N (2x) or 4-channel driver like L293D
- **LiPo Battery** - 7.4V 2S (1000-2200mAh recommended)
- **Voltage Regulator** - 5V BEC or LM7805
- **Propellers** (4x) - 2 CW, 2 CCW
- **Frame** - Quadcopter frame (F450 or smaller)
- **Connectors & Wires**
- **XT60 Connector** (for battery)
- **Power Distribution Board** (optional but recommended)

### Motor Configuration:
```
     FRONT
   M1    M2
    \    /
     \  /
      \/
      /\
     /  \
    /    \
   M4    M3
     BACK

M1 (Front Left)  - CCW (Counter-Clockwise)
M2 (Front Right) - CW  (Clockwise)
M3 (Back Right)  - CCW
M4 (Back Left)   - CW
```

---

## 2. Pin Connections

### A. MPU6050 to Pico W (I2C Connection)
```
MPU6050          Pico W
---------        --------
VCC       -----> 3.3V (Pin 36)
GND       -----> GND  (Pin 38)
SDA       -----> GP4  (Pin 6)  - I2C0 SDA
SCL       -----> GP5  (Pin 7)  - I2C0 SCL
INT       -----> GP3  (Pin 5)  - Optional interrupt
```

### B. Motor Drivers to Pico W (PWM Connections)

**Option 1: Using L298N (2 modules for 4 motors)**

**L298N Module #1 (M1 & M2):**
```
L298N #1         Pico W          Motor
---------        --------        ------
IN1       -----> GP6  (Pin 9)    
IN2       -----> GP7  (Pin 10)   --> M1 (Front Left)
IN3       -----> GP8  (Pin 11)   
IN4       -----> GP9  (Pin 12)   --> M2 (Front Right)
ENA       -----> GP10 (Pin 14)   PWM for M1
ENB       -----> GP11 (Pin 15)   PWM for M2
VCC       -----> 7.4V (Battery)
GND       -----> Common GND
```

**L298N Module #2 (M3 & M4):**
```
L298N #2         Pico W          Motor
---------        --------        ------
IN1       -----> GP12 (Pin 16)   
IN2       -----> GP13 (Pin 17)   --> M3 (Back Right)
IN3       -----> GP14 (Pin 19)   
IN4       -----> GP15 (Pin 20)   --> M4 (Back Left)
ENA       -----> GP16 (Pin 21)   PWM for M3
ENB       -----> GP17 (Pin 22)   PWM for M4
VCC       -----> 7.4V (Battery)
GND       -----> Common GND
```

**Option 2: Using 4-Channel Motor Driver (L293D or similar)**
```
Driver Pin       Pico W          Motor
-----------      --------        ------
M1 PWM    -----> GP10 (Pin 14)   
M1 DIR1   -----> GP6  (Pin 9)    --> M1
M1 DIR2   -----> GP7  (Pin 10)   

M2 PWM    -----> GP11 (Pin 15)   
M2 DIR1   -----> GP8  (Pin 11)   --> M2
M2 DIR2   -----> GP9  (Pin 12)   

M3 PWM    -----> GP16 (Pin 21)   
M3 DIR1   -----> GP12 (Pin 16)   --> M3
M3 DIR2   -----> GP13 (Pin 17)   

M4 PWM    -----> GP17 (Pin 22)   
M4 DIR1   -----> GP14 (Pin 19)   --> M4
M4 DIR2   -----> GP15 (Pin 20)   

VCC       -----> 7.4V (Battery)
GND       -----> Common GND
```

### C. Power Connections
```
Battery (7.4V 2S LiPo)
    |
    +----> Motor Driver VCC (7.4V)
    |
    +----> 5V Voltage Regulator (LM7805 or BEC)
              |
              +----> Pico W VSYS (Pin 39)
              +----> GND (Pin 38)
```

### D. Complete Pin Summary for Pico W
```
Pin #   GPIO    Function            Connection
-----   ----    --------            ----------
5       GP3     MPU6050 INT         MPU6050 INT (optional)
6       GP4     I2C0 SDA            MPU6050 SDA
7       GP5     I2C0 SCL            MPU6050 SCL
9       GP6     Motor Control       M1 Direction 1
10      GP7     Motor Control       M1 Direction 2
11      GP8     Motor Control       M2 Direction 1
12      GP9     Motor Control       M2 Direction 2
14      GP10    PWM                 M1 Speed (ENA)
15      GP11    PWM                 M2 Speed (ENB)
16      GP12    Motor Control       M3 Direction 1
17      GP13    Motor Control       M3 Direction 2
19      GP14    Motor Control       M4 Direction 1
20      GP15    Motor Control       M4 Direction 2
21      GP16    PWM                 M3 Speed (ENA)
22      GP17    PWM                 M4 Speed (ENB)
36      3.3V    Power               MPU6050 VCC
38      GND     Ground              Common Ground
39      VSYS    Power Input         5V from regulator
```

---

## 3. Power System Architecture

### Power Distribution Diagram:
```
                    7.4V LiPo Battery (2S)
                    [1000-2200mAh]
                           |
                    XT60 Connector
                           |
              +------------+------------+
              |                         |
        Power Switch              Fuse (5-10A)
              |                         |
              |            +------------+
              |            |
              |      Motor Drivers
              |      (7.4V Direct)
              |            |
              |         Motors x4
              |
        5V Regulator
        (LM7805/BEC)
         Max 1A
              |
        +-----+-----+
        |           |
    Pico W      MPU6050
    (5V)        (3.3V via Pico)
```

### Power Budget:
```
Component           Voltage    Current      Power
---------           -------    -------      -----
Pico W              5V         80-150mA     0.4-0.75W
MPU6050             3.3V       3-5mA        0.01-0.02W
Motors (each)       7.4V       500-2000mA   3.7-14.8W
Motors (total)      7.4V       2-8A         14.8-59.2W
Motor Drivers       7.4V       100mA        0.74W
-------------------------------------------------
Total Maximum                  8-10A        ~60W
```

**Recommended Battery:** 7.4V 2S LiPo, 1500mAh, 25C (37.5A burst capability)

---

## 4. Circuit Diagram Description

### Wiring Steps:

**Step 1: Power System**
1. Connect battery positive (+) to power switch
2. From switch, connect to fuse (5-10A)
3. From fuse:
   - Branch 1: To motor driver VCC
   - Branch 2: To 5V regulator input
4. Connect 5V regulator output to Pico W VSYS
5. Connect all grounds together (battery, drivers, regulator, Pico)

**Step 2: MPU6050**
1. Connect MPU6050 VCC to Pico 3.3V
2. Connect MPU6050 GND to Pico GND
3. Connect MPU6050 SDA to Pico GP4
4. Connect MPU6050 SCL to Pico GP5

**Step 3: Motor Drivers**
1. Connect driver power (VCC) to battery positive (via switch/fuse)
2. Connect driver GND to common ground
3. Connect each motor to driver outputs (A+, A-, B+, B-)
4. Connect PWM pins from Pico to driver enable pins
5. Connect direction pins from Pico to driver input pins

**Step 4: Safety**
1. Add capacitors (100µF) across motor terminals
2. Add capacitor (1000µF) at battery connection
3. Ensure all solder joints are secure
4. Use heat shrink on all connections
5. Mount components securely on frame

---

## 5. Software Architecture

### System Block Diagram:
```
┌─────────────────────────────────────────────────────────┐
│                    FLIGHT CONTROLLER                     │
│                    (Raspberry Pi Pico W)                 │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌─────────────┐      ┌──────────────┐                  │
│  │ WiFi Module │◄────►│ Web Server   │                  │
│  │  (Station)  │      │ (Commands)   │                  │
│  └─────────────┘      └──────────────┘                  │
│         │                     │                          │
│         ▼                     ▼                          │
│  ┌─────────────────────────────────┐                    │
│  │     Command Interpreter         │                    │
│  │  (Throttle, Pitch, Roll, Yaw)   │                    │
│  └─────────────────────────────────┘                    │
│                  │                                       │
│                  ▼                                       │
│  ┌──────────────────────────────────────┐               │
│  │         PID Controllers               │               │
│  │  ┌──────┐ ┌──────┐ ┌──────┐         │               │
│  │  │ Roll │ │ Pitch│ │ Yaw  │         │               │
│  │  │ PID  │ │ PID  │ │ PID  │         │               │
│  │  └──────┘ └──────┘ └──────┘         │               │
│  └──────────────────────────────────────┘               │
│         ▲                     │                          │
│         │                     ▼                          │
│  ┌─────────────┐      ┌──────────────┐                  │
│  │   MPU6050   │      │ Motor Mixer  │                  │
│  │ (I2C Sensor)│      │  (M1-M4 PWM) │                  │
│  └─────────────┘      └──────────────┘                  │
│         │                     │                          │
└─────────┼─────────────────────┼──────────────────────────┘
          │                     │
    ┌─────▼──────┐      ┌───────▼────────┐
    │ Gyroscope  │      │  4x DC Motors  │
    │Accelerometer│      │   + Drivers    │
    └────────────┘      └────────────────┘
```

### Software Components:

1. **Sensor Reading Module**
   - Read MPU6050 via I2C (gyro + accel)
   - Complementary filter for angle estimation
   - Sampling rate: 100-250 Hz

2. **PID Controllers**
   - Separate PID for Roll, Pitch, Yaw
   - Tunable gains (Kp, Ki, Kd)
   - Anti-windup protection

3. **Motor Mixer**
   - Convert PID outputs to motor speeds
   - Quadcopter X configuration
   - Motor speed limits (0-100%)

4. **WiFi Control**
   - Access Point or Station mode
   - Web interface for control
   - UDP/WebSocket for low latency
   - Command structure: throttle, pitch, roll, yaw

5. **Safety Module**
   - Low battery detection
   - Failsafe (kill motors on signal loss)
   - Angle limits
   - Emergency stop

---
