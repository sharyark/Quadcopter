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

## 6. Code Implementation

### Main Flight Controller Code (MicroPython)

```python
from machine import Pin, PWM, I2C
import network
import socket
import utime
import math
import _thread
import struct

# ==================== CONFIGURATION ====================
# Motor Pins
MOTOR_PINS = {
    'M1': {'pwm': 10, 'dir1': 6, 'dir2': 7},   # Front Left (CCW)
    'M2': {'pwm': 11, 'dir1': 8, 'dir2': 9},   # Front Right (CW)
    'M3': {'pwm': 16, 'dir1': 12, 'dir2': 13}, # Back Right (CCW)
    'M4': {'pwm': 17, 'dir1': 14, 'dir2': 15}  # Back Left (CW)
}

# I2C Configuration for MPU6050
I2C_SDA = 4
I2C_SCL = 5
MPU6050_ADDR = 0x68

# PID Tuning Parameters (ADJUST THESE!)
PID_ROLL = {'kp': 1.5, 'ki': 0.05, 'kd': 0.8}
PID_PITCH = {'kp': 1.5, 'ki': 0.05, 'kd': 0.8}
PID_YAW = {'kp': 2.0, 'ki': 0.1, 'kd': 0.5}

# WiFi Configuration
WIFI_SSID = "QuadcopterControl"
WIFI_PASSWORD = "drone1234"

# Safety Limits
MAX_ANGLE = 30  # degrees
MIN_BATTERY_VOLTAGE = 6.8  # volts (for 2S LiPo)
FAILSAFE_TIMEOUT = 1000  # milliseconds

# ==================== GLOBAL VARIABLES ====================
throttle = 0
pitch_cmd = 0
roll_cmd = 0
yaw_cmd = 0
armed = False
last_command_time = 0

# Sensor data
gyro_x, gyro_y, gyro_z = 0, 0, 0
accel_x, accel_y, accel_z = 0, 0, 0
angle_roll, angle_pitch = 0, 0

# PID state
roll_error_sum, pitch_error_sum, yaw_error_sum = 0, 0, 0
roll_last_error, pitch_last_error, yaw_last_error = 0, 0, 0

# ==================== MOTOR CONTROL CLASS ====================
class Motor:
    def __init__(self, name, pwm_pin, dir1_pin, dir2_pin):
        self.name = name
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)  # 1kHz PWM frequency
        self.dir1 = Pin(dir1_pin, Pin.OUT)
        self.dir2 = Pin(dir2_pin, Pin.OUT)
        self.speed = 0
    
    def set_speed(self, speed):
        """Set motor speed: 0-100 (forward only for quadcopter)"""
        speed = max(0, min(100, speed))  # Clamp 0-100
        self.speed = speed
        
        if speed == 0:
            self.dir1.value(0)
            self.dir2.value(0)
            self.pwm.duty_u16(0)
        else:
            # Forward direction
            self.dir1.value(1)
            self.dir2.value(0)
            duty = int((speed / 100) * 65535)
            self.pwm.duty_u16(duty)
    
    def stop(self):
        self.set_speed(0)

# ==================== MPU6050 CLASS ====================
class MPU6050:
    def __init__(self, i2c):
        self.i2c = i2c
        self.address = MPU6050_ADDR
        # Wake up MPU6050
        self.i2c.writeto_mem(self.address, 0x6B, b'\x00')
        utime.sleep_ms(100)
        # Configure gyro and accel ranges
        self.i2c.writeto_mem(self.address, 0x1B, b'\x08')  # Gyro ±500°/s
        self.i2c.writeto_mem(self.address, 0x1C, b'\x10')  # Accel ±8g
        
        # Calibration offsets (measure these during calibration)
        self.gyro_offset_x = 0
        self.gyro_offset_y = 0
        self.gyro_offset_z = 0
        
    def read_raw_data(self, addr):
        """Read 2 bytes and convert to signed value"""
        high = self.i2c.readfrom_mem(self.address, addr, 1)[0]
        low = self.i2c.readfrom_mem(self.address, addr+1, 1)[0]
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value
    
    def read_sensors(self):
        """Read gyro and accelerometer data"""
        # Read accelerometer
        accel_x = self.read_raw_data(0x3B) / 4096.0  # ±8g scale
        accel_y = self.read_raw_data(0x3D) / 4096.0
        accel_z = self.read_raw_data(0x3F) / 4096.0
        
        # Read gyroscope
        gyro_x = (self.read_raw_data(0x43) - self.gyro_offset_x) / 65.5  # ±500°/s scale
        gyro_y = (self.read_raw_data(0x45) - self.gyro_offset_y) / 65.5
        gyro_z = (self.read_raw_data(0x47) - self.gyro_offset_z) / 65.5
        
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    
    def calibrate(self, samples=200):
        """Calibrate gyroscope offsets"""
        print("Calibrating MPU6050... Keep drone still!")
        sum_x, sum_y, sum_z = 0, 0, 0
        
        for _ in range(samples):
            sum_x += self.read_raw_data(0x43)
            sum_y += self.read_raw_data(0x45)
            sum_z += self.read_raw_data(0x47)
            utime.sleep_ms(5)
        
        self.gyro_offset_x = sum_x / samples
        self.gyro_offset_y = sum_y / samples
        self.gyro_offset_z = sum_z / samples
        
        print(f"Calibration complete! Offsets: X={self.gyro_offset_x:.1f}, Y={self.gyro_offset_y:.1f}, Z={self.gyro_offset_z:.1f}")

# ==================== PID CONTROLLER ====================
class PIDController:
    def __init__(self, kp, ki, kd, output_limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral = 0
        self.last_error = 0
        
    def compute(self, setpoint, measured_value, dt):
        """Compute PID output"""
        error = setpoint - measured_value
        
        # Proportional
        p_term = self.kp * error
        
        # Integral with anti-windup
        self.integral += error * dt
        self.integral = max(-50, min(50, self.integral))  # Limit integral
        i_term = self.ki * self.integral
        
        # Derivative
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        d_term = self.kd * derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        output = max(-self.output_limit, min(self.output_limit, output))
        
        self.last_error = error
        return output
    
    def reset(self):
        """Reset PID state"""
        self.integral = 0
        self.last_error = 0

# ==================== INITIALIZE HARDWARE ====================
def init_hardware():
    """Initialize all hardware components"""
    global motors, mpu, pid_roll, pid_pitch, pid_yaw
    
    # Initialize motors
    motors = {
        'M1': Motor('M1', MOTOR_PINS['M1']['pwm'], MOTOR_PINS['M1']['dir1'], MOTOR_PINS['M1']['dir2']),
        'M2': Motor('M2', MOTOR_PINS['M2']['pwm'], MOTOR_PINS['M2']['dir1'], MOTOR_PINS['M2']['dir2']),
        'M3': Motor('M3', MOTOR_PINS['M3']['pwm'], MOTOR_PINS['M3']['dir1'], MOTOR_PINS['M3']['dir2']),
        'M4': Motor('M4', MOTOR_PINS['M4']['pwm'], MOTOR_PINS['M4']['dir1'], MOTOR_PINS['M4']['dir2'])
    }
    
    # Initialize I2C and MPU6050
    i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
    mpu = MPU6050(i2c)
    mpu.calibrate()
    
    # Initialize PID controllers
    pid_roll = PIDController(PID_ROLL['kp'], PID_ROLL['ki'], PID_ROLL['kd'])
    pid_pitch = PIDController(PID_PITCH['kp'], PID_PITCH['ki'], PID_PITCH['kd'])
    pid_yaw = PIDController(PID_YAW['kp'], PID_YAW['ki'], PID_YAW['kd'])
    
    print("Hardware initialized!")

# ==================== SENSOR UPDATE (COMPLEMENTARY FILTER) ====================
def update_sensors():
    """Read sensors and estimate angles using complementary filter"""
    global gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z
    global angle_roll, angle_pitch
    
    # Read sensors
    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = mpu.read_sensors()
    
    # Calculate angles from accelerometer
    accel_roll = math.atan2(accel_y, accel_z) * 57.2958  # Convert to degrees
    accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 57.2958
    
    # Complementary filter (98% gyro, 2% accel)
    dt = 0.004  # 250Hz loop = 4ms
    angle_roll = 0.98 * (angle_roll + gyro_x * dt) + 0.02 * accel_roll
    angle_pitch = 0.98 * (angle_pitch + gyro_y * dt) + 0.02 * accel_pitch

# ==================== MOTOR MIXER ====================
def update_motors():
    """Mix PID outputs and update motor speeds"""
    global throttle, armed
    
    if not armed or throttle < 5:
        # Stop all motors if not armed or throttle too low
        for motor in motors.values():
            motor.stop()
        return
    
    # Calculate PID corrections
    dt = 0.004  # 250Hz
    roll_correction = pid_roll.compute(roll_cmd, angle_roll, dt)
    pitch_correction = pid_pitch.compute(pitch_cmd, angle_pitch, dt)
    yaw_correction = pid_yaw.compute(yaw_cmd, gyro_z, dt)
    
    # Motor mixing for X configuration
    # M1 (Front Left CCW):  +pitch +roll -yaw
    # M2 (Front Right CW):  +pitch -roll +yaw
    # M3 (Back Right CCW):  -pitch -roll -yaw
    # M4 (Back Left CW):    -pitch +roll +yaw
    
    m1_speed = throttle + pitch_correction + roll_correction - yaw_correction
    m2_speed = throttle + pitch_correction - roll_correction + yaw_correction
    m3_speed = throttle - pitch_correction - roll_correction - yaw_correction
    m4_speed = throttle - pitch_correction + roll_correction + yaw_correction
    
    # Apply speeds
    motors['M1'].set_speed(m1_speed)
    motors['M2'].set_speed(m2_speed)
    motors['M3'].set_speed(m3_speed)
    motors['M4'].set_speed(m4_speed)

# ==================== WIFI & WEB SERVER ====================
def start_wifi():
    """Start WiFi Access Point"""
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=WIFI_SSID, password=WIFI_PASSWORD)
    
    while not ap.active():
        utime.sleep(0.1)
    
    print(f"WiFi AP started: {WIFI_SSID}")
    print(f"IP Address: {ap.ifconfig()[0]}")
    return ap.ifconfig()[0]

def web_server():
    """Simple web server for control"""
    global throttle, pitch_cmd, roll_cmd, yaw_cmd, armed, last_command_time
    
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)
    
    print("Web server listening on port 80")
    
    while True:
        try:
            cl, addr = s.accept()
            request = cl.recv(1024).decode('utf-8')
            
            # Parse command from request
            if '/cmd?' in request:
                params = request.split('/cmd?')[1].split(' ')[0]
                parse_command(params)
                last_command_time = utime.ticks_ms()
                response = "OK"
            else:
                response = get_html_page()
            
            cl.send('HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n')
            cl.send(response)
            cl.close()
            
        except Exception as e:
            print(f"Server error: {e}")

def parse_command(params):
    """Parse control commands"""
    global throttle, pitch_cmd, roll_cmd, yaw_cmd, armed
    
    for param in params.split('&'):
        if '=' in param:
            key, value = param.split('=')
            if key == 't':
                throttle = float(value)
            elif key == 'p':
                pitch_cmd = float(value)
            elif key == 'r':
                roll_cmd = float(value)
            elif key == 'y':
                yaw_cmd = float(value)
            elif key == 'arm':
                armed = value == '1'

def get_html_page():
    """Return HTML control interface"""
    return """
<!DOCTYPE html>
<html>
<head>
    <title>Quadcopter Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; text-align: center; padding: 20px; background: #1a1a1a; color: white; }
        .control { margin: 20px; }
        button { padding: 20px 40px; font-size: 18px; margin: 10px; border-radius: 10px; }
        .armed { background: #ff4444; }
        .disarmed { background: #44ff44; }
        input[type=range] { width: 80%; height: 30px; }
        .value { font-size: 24px; font-weight: bold; color: #00ff00; }
    </style>
</head>
<body>
    <h1>🚁 Quadcopter Control</h1>
    
    <div class="control">
        <button id="armBtn" class="disarmed" onclick="toggleArm()">ARM</button>
    </div>
    
    <div class="control">
        <label>Throttle: <span id="throttleVal" class="value">0</span>%</label><br>
        <input type="range" min="0" max="100" value="0" id="throttle" oninput="updateValue('throttle')">
    </div>
    
    <div class="control">
        <label>Pitch: <span id="pitchVal" class="value">0</span>°</label><br>
        <input type="range" min="-30" max="30" value="0" id="pitch" oninput="updateValue('pitch')">
    </div>
    
    <div class="control">
        <label>Roll: <span id="rollVal" class="value">0</span>°</label><br>
        <input type="range" min="-30" max="30" value="0" id="roll" oninput="updateValue('roll')">
    </div>
    
    <div class="control">
        <label>Yaw: <span id="yawVal" class="value">0</span>°/s</label><br>
        <input type="range" min="-50" max="50" value="0" id="yaw" oninput="updateValue('yaw')">
    </div>
    
    <div class="control">
        <button onclick="emergency()" style="background: red; color: white;">EMERGENCY STOP</button>
    </div>
    
    <script>
        let armed = false;
        
        function toggleArm() {
            armed = !armed;
            const btn = document.getElementById('armBtn');
            if (armed) {
                btn.textContent = 'DISARM';
                btn.className = 'armed';
            } else {
                btn.textContent = 'ARM';
                btn.className = 'disarmed';
                document.getElementById('throttle').value = 0;
                updateValue('throttle');
            }
            sendCommand();
        }
        
        function updateValue(ctrl) {
            const val = document.getElementById(ctrl).value;
            document.getElementById(ctrl + 'Val').textContent = val;
            sendCommand();
        }
        
        function sendCommand() {
            const t = document.getElementById('throttle').value;
            const p = document.getElementById('pitch').value;
            const r = document.getElementById('roll').value;
            const y = document.getElementById('yaw').value;
            const a = armed ? 1 : 0;
            
            fetch(`/cmd?t=${t}&p=${p}&r=${r}&y=${y}&arm=${a}`)
                .catch(e => console.error('Error:', e));
        }
        
        function emergency() {
            armed = false;
            document.getElementById('armBtn').textContent = 'ARM';
            document.getElementById('armBtn').className = 'disarmed';
            document.getElementById('throttle').value = 0;
            updateValue('throttle');
            alert('EMERGENCY STOP ACTIVATED!');
        }
        
        // Auto-update every 100ms when armed
        setInterval(() => {
            if (armed) sendCommand();
        }, 100);
    </script>
</body>
</html>
"""

# ==================== SAFETY CHECKS ====================
def check_safety():
    """Check safety conditions and failsafe"""
    global armed, throttle, last_command_time
    
    # Failsafe: Disarm if no command received in timeout period
    if armed and (utime.ticks_diff(utime.ticks_ms(), last_command_time) > FAILSAFE_TIMEOUT):
        print("FAILSAFE: Signal lost!")
        armed = False
        throttle = 0
        for motor in motors.values():
            motor.stop()
    
    # Angle limit check
    if armed and (abs(angle_roll) > MAX_ANGLE or abs(angle_pitch) > MAX_ANGLE):
        print(f"WARNING: Extreme angle! Roll={angle_roll:.1f}° Pitch={angle_pitch:.1f}°")

# ==================== MAIN FLIGHT LOOP ====================
def flight_loop():
    """Main flight control loop - runs at ~250Hz"""
    print("Starting flight loop...")
    
    loop_time = 0.004  # 4ms = 250Hz
    
    while True:
        start = utime.ticks_us()
        
        # 1. Read sensors
        update_sensors()
        
        # 2. Check safety
        check_safety()
        
        # 3. Update motors
        update_motors()
        
        # 4. Wait for next loop iteration
        elapsed = (utime.ticks_us() - start) / 1000000
        if elapsed < loop_time:
            utime.sleep(loop_time - elapsed)

# ==================== MAIN PROGRAM ====================
def main():
    """Main program entry point"""
    print("=" * 50)
    print("QUADCOPTER FLIGHT CONTROLLER")
    print("Raspberry Pi Pico W + MPU6050")
    print("=" * 50)
    
    # Initialize hardware
    init_hardware()
    
    # Start WiFi
    ip = start_wifi()
    print(f"\n>>> Connect to WiFi: {WIFI_SSID}")
    print(f">>> Password: {WIFI_PASSWORD}")
    print(f">>> Open browser: http://{ip}\n")
    
    # Start web server in separate thread
    _thread.start_new_thread(web_server, ())
    
    # Small delay to let server start
    utime.sleep(1)
    
    # Start main flight loop
    flight_loop()

# ==================== RUN ====================
if __name__ == "__main__":
    main()
```

---

## 7. WiFi Control Interface

### Control Methods:

**Option 1: Web Browser Control**
- Connect to WiFi: "QuadcopterControl"
- Open: http://192.168.4.1
- Use sliders for control
- Emergency stop button

**Option 2: Python Control Script** (for laptop/phone)

```python
import requests
import time

DRONE_IP = "192.168.4.1"

def send_command(throttle=0, pitch=0, roll=0, yaw=0, arm=0):
    url = f"http://{DRONE_IP}/cmd?t={throttle}&p={pitch}&r={roll}&y={yaw}&arm={arm}"
    try:
        requests.get(url, timeout=0.5)
    except:
        pass

# Example: Arm and increase throttle
send_command(arm=1)
time.sleep(0.5)

for t in range(0, 50, 5):
    send_command(throttle=t, arm=1)
    time.sleep(0.2)

# Disarm
send_command(arm=0)
```

**Option 3: Mobile App** (optional)
- Use MIT App Inventor
- Create sliders for controls
- HTTP requests to drone IP

---

## 8. Calibration & Testing

### Step-by-Step Testing Procedure:

#### Phase 1: Motor Test (No Propellers!)
1. Remove all propellers
2. Power on system
3. Connect to WiFi
4. Arm drone
5. Slowly increase throttle
6. Verify all motors spin
7. Check motor directions match configuration
8. Test emergency stop

#### Phase 2: IMU Calibration
1. Place drone on level surface
2. Run MPU6050 calibration
3. Record offsets
4. Verify angle readings (should be ~0° when level)

#### Phase 3: PID Tuning
**Start with low gains:**
```python
PID_ROLL = {'kp': 0.5, 'ki': 0.0, 'kd': 0.0}
PID_PITCH = {'kp': 0.5, 'ki': 0.0, 'kd': 0.0}
```

**Tuning procedure:**
1. Increase Kp until oscillation appears
2. Reduce Kp by 20-30%
3. Add Kd to dampen oscillations
4. Add small Ki for steady-state error
5. Test and repeat

#### Phase 4: Hover Test (With Propellers)
⚠️ **SAFETY FIRST** ⚠️
- Test in open area
- Use tether/safety line
- Wear eye protection
- Have emergency stop ready

1. Install propellers (check rotation!)
2. Secure drone with tether
3. Arm system
4. Gradually increase throttle
5. Observe stability
6. Tune PID as needed

#### Phase 5: Free Flight
Only after successful tethered tests!

---

## 9. Troubleshooting

### Common Issues:

**Motors don't spin:**
- Check power connections
- Verify motor driver wiring
- Test with minimal code
- Check PWM signals with multimeter

**Drone unstable/oscillates:**
- Reduce PID gains
- Check propeller directions
- Verify MPU6050 orientation
- Recalibrate IMU

**WiFi connection drops:**
- Reduce interference
- Use shorter distance
- Check power supply stability
- Add capacitors to power lines

**One motor slower:**
- Check motor condition
- Verify driver output
- Balance propellers
- Check for mechanical friction

---

## 10. Safety Guidelines

⚠️ **CRITICAL SAFETY RULES** ⚠️

1. **Always remove propellers during initial testing**
2. **Never test indoors initially**
3. **Use safety glasses**
4. **Keep fingers away from propellers**
5. **Have fire extinguisher nearby (LiPo safety)**
6. **Never leave battery unattended while charging**
7. **Use LiPo safety bag for charging/storage**
8. **Test emergency stop before first flight**
9. **Follow local drone regulations**
10. **Start with tethered flights**

---

## 11. Enhancements & Next Steps

### Possible Upgrades:
- **Altitude Hold**: Add barometer (BMP280)
- **GPS**: Add GPS module for position hold
- **FPV Camera**: Add camera for first-person view
- **Better Battery**: Voltage monitoring with ADC
- **Brushless Motors**: Use ESCs for better performance
- **Remote Control**: Add RC receiver support
- **Data Logging**: SD card for flight logs
- **LED Indicators**: Status LEDs for armed/disarmed
- **Buzzer**: Low battery warning

---

## 12. Bill of Materials (BOM)

| Component | Quantity | Est. Price (USD) |
|-----------|----------|------------------|
| Raspberry Pi Pico W | 1 | $6 |
| MPU6050 | 1 | $2 |
| DC Motors (8520) | 4 | $8 |
| L298N Motor Driver | 2 | $4 |
| 7.4V 2S LiPo Battery | 1 | $15 |
| 5V Regulator/BEC | 1 | $3 |
| Propellers | 4 | $3 |
| Quadcopter Frame | 1 | $10 |
| XT60 Connector | 1 | $1 |
| Wires & Connectors | - | $5 |
| **Total** | | **~$57** |

---

## Good Luck! 🚁

Remember: Start slow, test thoroughly, and prioritize safety! Building a quadcopter is a learning process. Don't rush it!

For questions or issues, check:
- MicroPython documentation
- RC Groups forum
- GitHub quadcopter projects
- r/Multicopter on Reddit