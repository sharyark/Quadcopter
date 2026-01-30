# 🚁 Micro Quadcopter Architecture Guide
## Ultra-Lightweight Design: Pico W + MPU6050 + Coreless Motors

---

## 📐 System Overview

### Design Goals:
- **Total Weight**: 50-70g (all-up weight)
- **Frame Size**: 100-150mm diagonal
- **Flight Time**: 6-8 minutes
- **Control**: WiFi via smartphone/laptop
- **Cost**: ~$35-40 total

### Why This Matters:
Lightweight = Longer flight time + Better maneuverability + Safer crashes!

---

## 🛍️ Complete Parts List

| Component | Specification | Weight | Price | Where to Buy |
|-----------|---------------|--------|-------|--------------|
| **Pico W** | RP2040 + WiFi | 3g | $6 | Adafruit, Amazon |
| **MPU6050** | 6-axis IMU | 1g | $2 | AliExpress, Amazon |
| **Coreless Motors** | 7mm/8.5mm, 4pcs | 8g | $8 | Banggood, AliExpress |
| **N-FET MOSFETs** | AO3400 or SI2302, 4pcs | 1g | $2 | DigiKey, LCSC |
| **Resistors** | 1kΩ (4x), 10kΩ (4x) | <1g | $1 | Local electronics |
| **1S LiPo Battery** | 3.7V 400-500mAh, 25C | 15g | $8 | Hobbyking, Amazon |
| **Propellers** | 55mm or 65mm, 4pcs | 4g | $2 | Banggood |
| **Frame Material** | Carbon fiber rods/PCB | 10g | $5 | DIY |
| **JST Connector** | 1.25mm or 2.0mm | 1g | $1 | Local |
| **Wires** | 28AWG silicone | 2g | $2 | Amazon |
| **Misc** | Heat shrink, glue | 2g | $2 | Local |
| **TOTAL** | | **47g** | **~$39** | |

**Final weight with battery: 62g** ✅ Perfect for micro drone!

---

## 🔌 Pin Connections - Ultra-Simplified

### Complete Wiring Map:

```
┌─────────────────────────────────────────────────┐
│           RASPBERRY PI PICO W                    │
│                                                  │
│  Pin 1  GP0  ─────► MPU6050 SDA (Blue wire)    │
│  Pin 2  GP1  ─────► MPU6050 SCL (Yellow wire)  │
│  Pin 4  GP2  ─────► Motor 1 MOSFET Gate        │
│  Pin 5  GP3  ─────► Motor 2 MOSFET Gate        │
│  Pin 6  GP4  ─────► Motor 3 MOSFET Gate        │
│  Pin 7  GP5  ─────► Motor 4 MOSFET Gate        │
│  Pin 36 3.3V ─────► MPU6050 VCC (Red wire)     │
│  Pin 38 GND  ─────► Common Ground (Black wire) │
│  Pin 39 VSYS ─────► Battery + (3.7V Red wire)  │
│                                                  │
└─────────────────────────────────────────────────┘

         ▼ Total wires: Only 10 connections! ▼
```

### Motor Connection Diagram:

```
For each motor (×4):

    Battery 3.7V (+)
          │
          ├──────► Motor Positive (+)
          │              │
          │         Motor Body
          │              │
          │        Motor Negative (-)
          │              │
          │         [MOSFET Drain]
          │              │
          │         [MOSFET Source]
          │              │
          └─────────► GND

    Pico GPIO ──[1kΩ]──► [MOSFET Gate]
                            │
                      [10kΩ to GND]
```

### Complete System Wiring:

```
                    3.7V LiPo Battery
                    (400mAh, 15g)
                          │
                    ┌─────┴─────┐
                    │    JST    │
                    │ Connector │
                    └─────┬─────┘
                          │
              ┌───────────┼──────────┬───────────┐
              │           │          │           │
         To Motor1   To Motor2  To Motor3   To Motor4
         via MOSFET  via MOSFET via MOSFET  via MOSFET
              │           │          │           │
              │           │          │    ┌──────┴─────┐
              │           │          │    │  Pico W    │
              │           │          │    │   VSYS     │◄─── 3.7V
              │           │          │    │            │
              │           │          │    │   3.3V     │◄─┐
              │           │          │    └────────────┘  │
              │           │          │                     │
              │           │          │                  ┌──┴──┐
              │           │          │                  │ MPU │
              │           │          │                  │6050 │
              │           │          │                  └─────┘
              │           │          │
              └───────────┴──────────┴──► Common GND
```

---

## ⚡ Power System Architecture

### Battery Selection Chart:

```
Battery Size    Weight    Flight Time    Best For
────────────    ──────    ───────────    ────────
200mAh          8g        3-4 min        Ultra-light (<50g)
300mAh          12g       5-6 min        Light (50-60g)
400mAh          15g       6-7 min        ★ RECOMMENDED ★
500mAh          18g       7-8 min        Heavy (70-80g)
600mAh+         20g+      Not worth it   Too heavy!
```

**Recommendation**: **400mAh 1S LiPo** is the sweet spot!

### Voltage Ranges:

```
LiPo Cell Voltage States:
───────────────────────────
4.2V  ████████████  Fully charged (stop charging!)
3.7V  ████████      Normal operating voltage
3.5V  ████          Warning - land soon!
3.3V  ██            CRITICAL - land immediately!
3.0V  ░░            DAMAGE ZONE - never discharge this low!

Safe flying range: 3.5V - 4.1V
```

### Power Distribution Layout:

```
Top View of Drone:

                Battery (center, bottom)
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        │         ┌─────┴─────┐        │
        │         │  Pico W   │        │
        │         │ MPU6050   │        │
        │         └───────────┘        │
        │                               │
   [Motor 1]                       [Motor 2]
      │                                 │
    [FET 1]                          [FET 2]
        │                               │
        └───────────┬───────────────────┘
                    │
        ┌───────────┴───────────────┐
        │                           │
   [Motor 4]                   [Motor 3]
        │                           │
     [FET 4]                     [FET 3]
        │                           │
        └───────────┬───────────────┘
                    │
                  [GND]
```

---

## 🏗️ Frame Design Options

### Option 1: Carbon Fiber X-Frame (BEST)

**Pros**: Strongest, lightest, professional look
**Cons**: Requires cutting and gluing
**Weight**: 10-12g
**Cost**: $5-8

```
Top View:
                 
       [Tube]    [Tube]
          \        /
           \  [C] /      C = Center plate (40×40mm)
            \ │ /        Tubes = 2mm carbon fiber, 70-90mm
             \│/
          ────┼────      
             /│\
            / │ \
           /  [C] \
          /        \
     [Tube]      [Tube]
     
Assembly:
1. Cut 4 carbon tubes (80mm each)
2. Cut center plate from 1.5mm carbon sheet
3. Arrange in X pattern
4. Glue with CA glue + accelerator
5. Reinforce with epoxy on joints
```

### Option 2: PCB Frame (INNOVATIVE)

**Pros**: Lightest, integrated wiring, cool!
**Cons**: Need to design and order
**Weight**: 8-10g
**Cost**: $2-3 (5 pieces from JLCPCB)

```
Integrated PCB Design:

┌──────────┐        ┌──────────┐
│  Motor   │        │  Motor   │
│  Mount   │        │  Mount   │
└────┬─────┘        └─────┬────┘
     │                    │
     │   ┌──────────┐    │
     │   │  Center  │    │
     └───┤  PCB     ├────┘
         │  (Power  │
         │  traces) │
     ┌───┤          ├────┐
     │   └──────────┘    │
     │                    │
┌────┴─────┐        ┌─────┴────┐
│  Motor   │        │  Motor   │
│  Mount   │        │  Mount   │
└──────────┘        └──────────┘

Features:
- Copper traces for power distribution
- Mounting holes for components
- Silkscreen labels for wiring
- Can add LED pads
```

### Option 3: 3D Printed Frame

**Pros**: Easiest to make, customizable
**Cons**: Heaviest option
**Weight**: 12-15g
**Cost**: $1 (filament)

```
Design in Tinkercad (free, browser-based):
1. Central hub (30mm diameter, 5mm thick)
2. 4 arms (10×5mm cross-section, 70mm long)
3. Motor mounts at ends (8mm hole)
4. 15-20% infill for weight savings

Print settings:
- Layer: 0.2mm
- Infill: 15%
- Material: PLA or PETG
- Time: ~2 hours
```

### Motor Mounting:

```
Side view of motor mount:

    ╔═════════╗
    ║ MOTOR   ║ ←── Coreless motor (7-8.5mm)
    ╚════╤════╝
         │
    [Hot Glue] ←── Secure with hot glue
         │
    ─────┴───── ←── Frame arm
```

---

## 🎯 Motor Configuration Guide

### X-Configuration Explained:

```
FRONT OF DRONE (Direction of flight)
         ▲
         │
         
   M1 ⟲      M2 ⟳
    [●]        [●]
      \        /
       \      /
        \    /
    ────  ╳  ────  ← Center (Pico W + MPU6050)
        /    \
       /      \
      /        \
    [●]        [●]
   M4 ⟳      M3 ⟲

Motor Spins:
M1 (Front Left)  = CCW ⟲  [Counterclockwise]
M2 (Front Right) = CW  ⟳  [Clockwise]
M3 (Back Right)  = CCW ⟲  [Counterclockwise]
M4 (Back Left)   = CW  ⟳  [Clockwise]

Why this matters:
- Opposite motors spin opposite ways
- Cancels out rotational torque
- Enables yaw control
```

### Propeller Direction:

```
CCW Propeller (M1, M3):    CW Propeller (M2, M4):
                           
    Leading edge             Leading edge
         ↓                        ↓
        ╱╲                       ╱╲
       ╱  ╲                     ╱  ╲
      ╱    ╲                   ╱    ╲
     ╱      ╲                 ╱      ╲
    (Spins ⟲)               (Spins ⟳)

Test: Blow on prop - should feel resistance
      (like it's trying to fly up)
```

---

## 🧠 Software Development Roadmap

### Phase 1: Hardware Testing (Week 1)
```
Goals:
├─ Test Pico W basics (LED blink)
├─ Test I2C communication with MPU6050
├─ Read and display IMU data
├─ Test individual motor control (NO PROPS!)
└─ Verify all connections

Tools needed:
- USB cable
- Computer with Thonny IDE
- Multimeter (optional)
```

### Phase 2: Sensor Integration (Week 1-2)
```
Goals:
├─ Read accelerometer data (angles)
├─ Read gyroscope data (rotation rates)
├─ Implement complementary filter
├─ Display real-time angles on serial
└─ Calibrate IMU offsets

Key concepts:
- Sample rate: 250Hz
- Filter: 98% gyro + 2% accel
- Calibration: Measure gyro zero-point
```

### Phase 3: Motor Control (Week 2)
```
Goals:
├─ Implement PWM motor control
├─ Create motor mixing algorithm
├─ Test motor response to tilt
├─ Verify motor directions
└─ Test failsafe (emergency stop)

Testing WITHOUT propellers:
- Tilt drone left → M4, M2 should speed up
- Tilt drone right → M1, M3 should speed up
- Tilt drone forward → M3, M4 should speed up
- Tilt drone back → M1, M2 should speed up
```

### Phase 4: PID Implementation (Week 3)
```
Goals:
├─ Implement Roll PID controller
├─ Implement Pitch PID controller
├─ Implement Yaw PID controller
├─ Start tuning (Kp only first)
└─ Test stabilization on bench

PID Tuning Process:
1. Start: Kp=0.5, Ki=0, Kd=0
2. Increase Kp until oscillation
3. Reduce Kp by 30%
4. Add Kd (usually 0.3-0.5 × Kp)
5. Add Ki last (very small, 0.02-0.05 × Kp)
```

### Phase 5: WiFi Control (Week 3-4)
```
Goals:
├─ Create WiFi Access Point
├─ Build web server
├─ Design HTML control interface
├─ Implement command parsing
└─ Add telemetry display

Control commands:
- Throttle: 0-100%
- Pitch: -30 to +30 degrees
- Roll: -30 to +30 degrees
- Yaw: -50 to +50 deg/s
- Arm/Disarm button
```

### Phase 6: Safety Features (Week 4)
```
Goals:
├─ Implement failsafe (signal loss)
├─ Add angle limits (±25°)
├─ Add low battery warning
├─ Add emergency stop button
└─ Test all safety features

Safety checks:
- If no WiFi command for 500ms → Disarm
- If angle > 25° → Limit motor output
- If battery < 3.5V → Flash LED warning
- Emergency stop → Kill all motors instantly
```

### Phase 7: Flight Testing (Week 5-6)
```
Testing progression:
├─ Tethered test (string tied to drone)
├─ Low hover test (10cm height)
├─ Controlled hover (30cm height)
├─ Basic movements (forward/back/left/right)
└─ Advanced flight (circles, figure-8)

Safety checklist:
- Test in open grass field
- Have spare props ready
- Start with 50% max throttle limit
- Always test emergency stop first
- Never fly near people or animals
```

---

## 📊 System Block Diagram

```
┌─────────────────────────────────────────────────────┐
│                    USER INTERFACE                    │
│         (Smartphone/Laptop Web Browser)             │
│                                                      │
│  Sliders: [Throttle] [Pitch] [Roll] [Yaw]          │
│  Buttons: [ARM] [DISARM] [EMERGENCY STOP]          │
│  Display: Battery: 3.7V  |  Angle: 2.3°            │
└──────────────────┬──────────────────────────────────┘
                   │
                   │ WiFi (HTTP Commands)
                   │
┌──────────────────▼──────────────────────────────────┐
│              RASPBERRY PI PICO W                     │
│              (Flight Controller)                     │
│                                                      │
│  ┌────────────┐        ┌────────────┐              │
│  │  WiFi RX   │───────►│  Command   │              │
│  │  Module    │        │  Parser    │              │
│  └────────────┘        └──────┬─────┘              │
│                               │                      │
│                    ┌──────────▼────────┐            │
│          ┌─────────┤  PID Controllers  ├─────────┐  │
│          │         │  (Roll,Pitch,Yaw) │         │  │
│          │         └──────────┬────────┘         │  │
│          │                    │                   │  │
│     ┌────▼────┐          ┌───▼────┐         ┌────▼────┐
│     │ Sensor  │◄─────────┤ Motor  ├────────►│ Safety  │
│     │ Fusion  │   IMU    │ Mixer  │  PWM    │ Monitor │
│     └─────────┘   Data   └────────┘         └─────────┘
│                                │                      │
└────────────────────────────────┼──────────────────────┘
                                 │
                     ┌───────────┼───────────┐
                     │           │           │
              ┌──────▼──┐  ┌─────▼────┐  ┌─▼────────┐
              │         │  │          │  │          │
              │         │  │          │  │          │
┌─────────────▼──┐  ┌───▼──────────┐  ┌──▼──────────┴─┐  ┌────────────────┐
│                │  │              │  │                │  │                │
│   MPU6050      │  │  4x MOSFETs  │  │   4x Motors   │  │  4x Propellers │
│   (IMU)        │  │  (Drivers)   │  │  (Coreless)   │  │  (55-65mm)     │
│                │  │              │  │                │  │                │
│ ┌─Accel        │  │  Gate◄───────┼──┼────Pico GPIO  │  │   ┌─Motor      │
│ └─Gyro         │  │  Drain───────┼──┼───►Motor(-)   │  │   └─Prop       │
│                │  │  Source──────┼──┼────►GND        │  │                │
└────────────────┘  └──────────────┘  └────────────────┘  └────────────────┘
```

---

## 🔄 Control Loop Flow

```
Main Loop (runs at 250Hz = every 4ms):

START
  │
  ├─► 1. Read WiFi commands (if available)
  │      └─► Update: throttle, pitch, roll, yaw, arm status
  │
  ├─► 2. Read MPU6050 sensor data
  │      ├─► Accelerometer → Calculate angles
  │      ├─► Gyroscope → Get rotation rates
  │      └─► Complementary filter → Smooth angles
  │
  ├─► 3. Check safety conditions
  │      ├─► Signal timeout? → DISARM
  │      ├─► Angle > 25°? → LIMIT THROTTLE
  │      └─► Battery < 3.5V? → WARNING
  │
  ├─► 4. PID Controllers (if armed)
  │      ├─► Roll PID: (desired - actual) → correction
  │      ├─► Pitch PID: (desired - actual) → correction
  │      └─► Yaw PID: (desired - actual) → correction
  │
  ├─► 5. Motor Mixer
  │      ├─► M1 = throttle + pitch + roll - yaw
  │      ├─► M2 = throttle + pitch - roll + yaw
  │      ├─► M3 = throttle - pitch - roll - yaw
  │      └─► M4 = throttle - pitch + roll + yaw
  │
  ├─► 6. Apply motor speeds (PWM output)
  │      └─► Update 4 MOSFET gates
  │
  └─► 7. Wait for next loop (4ms timer)
       └─► REPEAT ↑
```

### PID Explained Visually:

```
Example: Roll Control

Desired Angle: 0° (level)
Current Angle: 5° (tilted right)
Error: -5°

PID Calculation:
├─► P (Proportional): -5° × 0.8 = -4.0
├─► I (Integral): sum of errors × 0.02 = -0.1
├─► D (Derivative): rate of change × 0.4 = -0.5
└─► Total correction: -4.6

Motor Mixer applies correction:
├─► Motors on left side (M1, M4): SPEED UP
└─► Motors on right side (M2, M3): SLOW DOWN

Result: Drone rolls back to level! ✓
```

---

## 🎮 WiFi Control Interface Design

### Web Page Layout:

```
╔═══════════════════════════════════════════╗
║  🚁 MICRO DRONE CONTROL                   ║
╠═══════════════════════════════════════════╣
║                                           ║
║  Status: ● ARMED          Battery: 3.8V  ║
║  Mode: STABILIZE         Signal: Strong  ║
║                                           ║
╟───────────────────────────────────────────╢
║  THROTTLE                                 ║
║  [░░░░░░░░░░█████░░░░] 45%              ║
║  ▼ Slide up for more power               ║
╟───────────────────────────────────────────╢
║  PITCH (Forward/Backward)                 ║
║  [░░░░░░░█░░░░░░░░] 0°                  ║
║  ◄─────────────────► -30° to +30°       ║
╟───────────────────────────────────────────╢
║  ROLL (Left/Right)                        ║
║  [░░░░░░░█░░░░░░░░] 0°                  ║
║  ◄─────────────────► -30° to +30°       ║
╟───────────────────────────────────────────╢
║  YAW (Rotation)                           ║
║  [░░░░░░░█░░░░░░░░] 0°/s                ║
║  ◄─────────────────► -50° to +50°       ║
╟───────────────────────────────────────────╢
║  📊 TELEMETRY                             ║
║  Current Angles:                          ║
║  Roll: +2.3°   Pitch: -1.1°              ║
║  Gyro: X=12  Y=-5  Z=3  (deg/s)          ║
╟───────────────────────────────────────────╢
║  [  ARM  ]          [ DISARM ]           ║
║  [🔴 EMERGENCY STOP - KILLS ALL MOTORS]  ║
║                                           ║
╚═══════════════════════════════════════════╝
```

### Connection Instructions:

```
Step 1: Power on drone
         └─► Pico W creates WiFi network

Step 2: On your phone/laptop
         ├─► Open WiFi settings
         ├─► Find network "MicroDrone"
         └─► Connect with password "fly12345"

Step 3: Open web browser
         └─► Go to: http://192.168.4.1

Step 4: Control your drone!
         └─► Use sliders to fly
```

---

## ⚙️ Component Assembly Guide

### Step 1: Solder MOSFET Circuit (×4)

```
For EACH motor:

Components needed:
- 1× N-Channel MOSFET (AO3400)
- 1× 1kΩ resistor
- 1× 10kΩ resistor
- Wire: 5cm red, 5cm black, 5cm signal

Circuit diagram:
                    
    Pico GPIO pin
         │
        [1kΩ]
         │
         ├──────► Gate (MOSFET)
         │             │
       [10kΩ]       Drain ──► Motor (-)
         │             │
        GND         Source ──► GND

Motor (+) connects to Battery (+)

Tips:
- Use heat shrink on all solder joints
- Keep wires short (<5cm)
- Test continuity with multimeter
```

### Step 2: Mount MPU6050

```
Placement:                Mounting method:
                         
    ┌─────────┐         1. Cut small piece of
    │ MPU6050 │            double-sided foam tape
    │   [●]   │         2. Stick to bottom of MPU6050
    └─────────┘         3. Press firmly onto center of frame
         │              4. Ensure MPU is LEVEL and
         │                 aligned with drone axes
    [Frame Center]      
    
Orientation check:
- X axis = Forward (nose of drone)
- Y axis = Right wing
- Z axis = Up (toward sky)
```

### Step 3: Mount Pico W

```
Position: On top of frame, near center

Method 1: Foam tape (easiest)
   [Pico W]
      │
  [Foam tape]
      │
    [Frame]

Method 2: Zip tie through holes
   [Pico W]
      │
  [Zip tie] ──► Pull tight
      │
    [Frame]

Tips:
- Keep weight balanced
- Solder wires BEFORE mounting
- Test fit before permanent mounting
```

### Step 4: Install Motors

```
Each motor mount:

1. Apply hot glue to frame arm tip
2. Press motor body into glue
3. Hold for 10 seconds
4. Let dry for 5 minutes
5. Test: Motor should be FIRM, not wobbly

   ╔═══════╗
   ║ MOTOR ║ ←── 7-8.5mm coreless
   ╚═══╤═══╝
   [Hot glue]
       │
   ────┴──── ←── Carbon fiber arm

Double-check motor placement:
[M1]        [M2]  ← Diagonal pairs should
    \      /        be same motor type
     \ ── /
      / \/\
     /    \
[M4]        [M3]
```

### Step 5: Battery Mounting

```
Battery position: Center bottom (under Pico W)

Method: Velcro strap (recommended)

Side view:
                ┌──────────┐
                │  Pico W  │
                ├──────────┤
                │  Frame   │
                └────┬─────┘
              [Velcro strap]
                     │
              ┌──────┴──────┐
              │   Battery   │
              │   400mAh    │
              └─────────────┘

Why velcro strap?
- Easy battery swap
- Adjust CG (center of gravity)
- Quick removal for charging
```

### Step 6: Propeller Installation

```
CRITICAL: Match prop type to motor spin direction!

CCW Props (M1, M3):        CW Props (M2, M4):
     A-type                     B-type
         or                        or
    Counter-CW                 Clockwise
    
Installation:
1. Check prop type (look for letter A or B)
2. Press onto motor shaft
3. Ensure seated fully
4. Should spin freely by hand

Test before flight:
- Spin each prop by hand
- Should spin smoothly
- No wobble or catching
```

---

## 🧪 Testing Protocol

### Pre-Flight Checklist:

```
HARDWARE CHECKS:
├─ [ ] All propellers secure and correct type
├─ [ ] Motor directions verified
├─ [ ] Battery fully charged (4.2V)
├─ [ ] All solder joints secure
├─ [ ] Frame has no cracks
├─ [ ] MPU6050 is level
├─ [ ] Wires not touching propellers
└─ [ ] Weight < 70g

SOFTWARE CHECKS:
├─ [ ] WiFi connects successfully
├─ [ ] Web interface loads
├─ [ ] All sliders respond
├─ [ ] Emergency stop works
├─ [ ] Telemetry displays correctly
└─ [ ] Failsafe triggers after 500ms

ENVIRONMENT:
├─ [ ] Open grass field
├─ [ ] No wind or light breeze
├─ [ ] No people/animals nearby
├─ [ ] Spare props ready
└─ [ ] Emergency landing area clear
```

### Testing Phases:

```
Phase 1: Bench Test (NO PROPELLERS)
│
├─► Test 1: Motor response
│   └─► Tilt drone, watch motor speeds change
│
├─► Test 2: PID stability
│   └─► Should try to level itself
│
└─► Test 3: Emergency stop
    └─► Should kill motors instantly
    
✓ Pass all tests? → Continue to Phase 2


Phase 2: Tethered Test (WITH PROPELLERS)
│
├─► Setup: Tie drone to ground with 1m string
│
├─► Test 1: Low throttle hover (30%)
│   └─► Should lift slightly off ground
│
├─► Test 2: Stability check
│   └─► Push gently, should return to level
│
└─► Test 3: Control response
    └─► Test pitch, roll, yaw commands
    
✓ Stable hover? → Continue to Phase 3


Phase 3: Free Flight (OPEN AREA)
│
├─► Flight 1: 10cm hover (5 seconds)
│   └─► Just lift off and land
│
├─► Flight 2: 30cm hover (10 seconds)
│   └─► Hold steady altitude
│
├─► Flight 3: Basic maneuvers
│   └─► Forward, back, left, right
│
└─► Flight 4: Full control
    └─► Circles, figure-8, etc.
```

---

## 🔧 Troubleshooting Guide

### Problem #1: Motors don't spin

```
Diagnostic steps:
├─► Check battery voltage (should be > 3.5V)
│   └─► Use multimeter on battery terminals
│
├─► Check PWM signals
│   └─► Upload simple motor test code
│
├─► Check MOSFET connections
│   ├─► Gate connected to Pico GPIO?
│   ├─► Drain connected to motor (-)?
│   └─► Source connected to GND?
│
└─► Check motor itself
    └─► Connect directly to battery (briefly!)
    
Most common cause: Loose ground connection
```

### Problem #2: Drone flips on takeoff

```
Likely causes:
├─► Motor spinning wrong direction
│   └─► Fix: Check motor wiring, swap if needed
│
├─► Wrong propeller type
│   └─► Fix: Match CW/CCW props to motor spin
│
├─► Motors in wrong positions
│   └─► Fix: Verify M1-M4 wiring matches diagram
│
└─► Props installed upside-down
    └─► Fix: Flip props (curved side up)
    
Quick test: Increase throttle slowly while holding
```

### Problem #3: Oscillates/shakes

```
PID tuning issue:

Fast oscillation (>5Hz):
└─► Kp too high → Reduce by 20-30%

Slow oscillation (<2Hz):
└─► Kd too low → Increase Kd

Drifts but stable:
└─► Ki too low → Increase Ki slightly

Always tune in this order:
1. Kp first
2. Then Kd
3. Finally Ki
```

### Problem #4: Drifts in one direction

```
Possible causes:
├─► IMU not calibrated
│   └─► Re-run calibration with drone level
│
├─► IMU not mounted level
│   └─► Check with spirit level, re-mount
│
├─► One motor weaker
│   └─► Test each motor individually
│
└─► Center of gravity off
    └─► Move battery to balance
```

### Problem #5: Short flight time

```
Check these:
├─► Battery health
│   └─► Old battery? Replace if puffed
│
├─► Motor efficiency
│   └─► Clean motor, check for damage
│
├─► Weight too high
│   └─► Remove unnecessary components
│
└─► Flying too aggressively
    └─► Smooth inputs = longer flight
    
Target: 6-7 minutes for 400mAh battery
```

---

## 💡 Pro Tips & Tricks

### Weight Optimization:

```
Every gram saved = Better performance!

Easy wins:
├─ Remove Pico pin headers: Save 5g
├─ Use 28AWG wire (not 24AWG): Save 2g
├─ Skip power switch: Save 2g
├─ SMD components: Save 1g
└─ Minimal frame design: Save 3-5g

Total potential savings: ~13g!
```

### Flight Time Extension:

```
Tips to fly longer:
├─ Limit max throttle to 75%
├─ Fly smoothly (no jerky moves)
├─ Land at 3.5V (don't drain fully)
├─ Use efficient props (test different sizes)
└─ Keep drone clean (dust adds weight!)

Expected improvement: +1-2 minutes
```

### Crash Prevention:

```
Safety practices:
├─ Always test emergency stop FIRST
├─ Start with low throttle limits (60%)
├─ Fly away from yourself (easier control)
├─ Keep altitude low initially (<1m)
├─ Practice in flight simulator first
└─ Have observer for first flights

Remember: Crashes are learning opportunities!
```

### Maintenance Schedule:

```
After every 10 flights:
├─ Check all solder joints
├─ Tighten motor mounts
├─ Clean props (remove debris)
├─ Check frame for cracks
└─ Re-calibrate IMU

After every crash:
├─ Replace damaged props
├─ Check motor shafts (bent?)
├─ Test all motors individually
└─ Verify frame integrity
```

---

## 🚀 Next Steps & Upgrades

### Phase 1 Upgrades (Easy):

```
1. LED Indicators (<1g)
   └─ Show armed status, battery level
   
2. Buzzer (<1g)
   └─ Low battery warning, lost drone finder
   
3. Better Battery (0g)
   └─ Upgrade to 500mAh for longer flight
   
4. Spare Props
   └─ Always have 5 sets ready!
```

### Phase 2 Upgrades (Medium):

```
1. Barometer (BMP280, 2g)
   └─ Altitude hold mode
   
2. Optical Flow Sensor (5g)
   └─ Position hold indoors
   
3. FPV Camera (10-15g)
   └─ First-person view flying
   
4. RC Receiver (3g)
   └─ Traditional remote control
```

### Phase 3 Upgrades (Advanced):

```
1. GPS Module (15g)
   └─ Return-to-home, waypoints
   
2. Brushless Motors
   └─ More power, efficiency
   
3. Larger Frame (150-250mm)
   └─ More stable, carry camera
   
4. Custom PCB Flight Controller
   └─ All components on one board
```

### Learning Resources:

```
Essential reading:
├─ Quadcopter dynamics
├─ PID control theory
├─ LiPo battery safety
├─ RF communication basics
└─ Sensor fusion algorithms

Useful tools:
├─ PID tuning simulator
├─ Flight time calculator
├─ Thrust calculator
└─ Center of gravity calculator

Communities:
├─ r/Multicopter (Reddit)
├─ RC Groups Forum
├─ DIY Drones
└─ Pico W Discord
```

---

## 🎓 Understanding Flight Physics

### How Quadcopters Fly:

```
HOVER (All motors equal speed):
   M1 ↑  M2 ↑
      \ | /
       \|/
        ●
       /|\
      / | \
   M4 ↑  M3 ↑
   
   Net force: UP
   Drone: Stays in place


ROLL LEFT (M1↓ M4↓, M2↑ M3↑):
   M1 ↓  M2 ↑
      \ | /
       \|/── Tilts left
        ●
       /|\
      / | \
   M4 ↓  M3 ↑
   
   Net force: UP + LEFT
   Drone: Moves left


PITCH FORWARD (M3↓ M4↓, M1↑ M2↑):
   M1 ↑  M2 ↑
      \ | /
       \|/── Tilts forward
        ●
       /|\
      / | \
   M4 ↓  M3 ↓
   
   Net force: UP + FORWARD
   Drone: Moves forward


YAW (M1↓ M3↓, M2↑ M4↑):
   M1 ↓  M2 ↑
      \ | /
       \|/── Rotates CW
        ●
       /|\
      / | \
   M4 ↑  M3 ↓
   
   Torque: Clockwise
   Drone: Spins right
```

---

## ✅ Final Checklist Before First Flight

```
HARDWARE:
├─ [✓] Frame assembled and strong
├─ [✓] Motors mounted securely
├─ [✓] MOSFETs soldered correctly
├─ [✓] MPU6050 mounted level
├─ [✓] Pico W powered and programmed
├─ [✓] Battery charged to 4.2V
├─ [✓] Propellers correct type and secure
├─ [✓] Total weight < 70g
└─ [✓] No loose wires

SOFTWARE:
├─ [✓] Code uploaded and running
├─ [✓] WiFi network visible
├─ [✓] Web interface accessible
├─ [✓] PID values set conservatively
├─ [✓] Safety features enabled
├─ [✓] Emergency stop tested
└─ [✓] Failsafe triggers properly

ENVIRONMENT:
├─ [✓] Open grass field (20m x 20m minimum)
├─ [✓] No obstacles or people
├─ [✓] Wind < 10 km/h
├─ [✓] Good visibility (daylight)
└─ [✓] Spare props ready

PERSONAL:
├─ [✓] Read all safety guidelines
├─ [✓] Understand controls
├─ [✓] Know how to disarm quickly
├─ [✓] Observer present (recommended)
└─ [✓] Phone charged (controller)
```

---

## 🎯 Success Criteria

### Beginner Level:
- ✅ Stable hover for 30 seconds
- ✅ Smooth takeoff and landing
- ✅ Can control height
- ✅ Flight time > 5 minutes

### Intermediate Level:
- ✅ Stable hover for 2+ minutes
- ✅ Forward/backward/left/right control
- ✅ Can fly figure-8 pattern
- ✅ Flight time > 6 minutes

### Advanced Level:
- ✅ Full 3D control
- ✅ Smooth cinematic movements
- ✅ Can handle light wind
- ✅ Multiple flight modes

---

## 💬 Final Words

```
┌──────────────────────────────────────────┐
│                                           │
│  Remember:                                │
│                                           │
│  🎯 Start simple, build up gradually     │
│  🔧 Test thoroughly at each step         │
│  📚 Learn from mistakes (crashes happen!)│
│  🛡️  Safety first, always                │
│  🎮 Practice makes perfect               │
│  🤝 Ask for help when stuck              │
│  🎉 Celebrate small wins                 │
│  🚁 Most importantly: HAVE FUN!          │
│                                           │
└──────────────────────────────────────────┘
```

**Good luck building your micro quadcopter!** 🚀

---

**Questions? Issues? Suggestions?**
- Review this guide step-by-step
- Check troubleshooting section
- Search online communities
- Don't give up - you got this!

---

*Document Version: 1.0*  
*Created: 2026*  
*For: DIY Micro Quadcopter Builders*