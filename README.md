# Two-wheeled self-balancing robot with wireless position control

This project implements a **two-wheel self-balancing robot** using an **ESP32**, an **MPU6050 IMU**, **quadrature encoders**, and wireless control through **MQTT**.  
The system stabilizes using a state-feedback controller, receiving references and control gains remotely via an **MQTT broker**.  

---

## 🚀 Main Features

- 📡 **Wireless control via MQTT** (publishing and subscribing to references and gains).
- ⚙️ **State-feedback controller** using position, velocity, angle, and angular velocity.
- 🎛️ **Remote tuning of control gains (k1, k2, k3, k4)** via MQTT topics.
- 📏 **Accurate sensing** using:
  - **MPU6050** (orientation and angular velocity).
  - **Encoders** for linear and angular position.
- ⚡ **TB6612FNG driver** for DC motors.
- 🔋 Battery-powered (tested with 3x3.3 V supply).

---

## 📡 MQTT Topics

The system uses the following topics:

### Input (subscription):
- `control/sr_roque/k1` → Gain **k1**
- `control/sr_roque/k2` → Gain **k2**
- `control/sr_roque/k3` → Gain **k3**
- `control/sr_roque/k4` → Gain **k4**
- `control/sr_roque/teta_ref` → Angle reference
- `control/sr_roque/x_ref` → Position reference

### Output (publishing):
- `control/sr_roque/k1_eco` → Echo of k1
- `control/sr_roque/k2_eco` → Echo of k2
- `control/sr_roque/k3_eco` → Echo of k3
- `control/sr_roque/k4_eco` → Echo of k4
- `control/sr_roque/x_ref_eco` → Smoothed position reference
- `control/sr_roque/uk` → Applied control signal

---

## 🔧 Required Hardware

- **ESP32**
- **MPU6050 IMU**
- **2 DC motors with quadrature encoders**
- **TB6612FNG motor driver**
- **LiPo battery** (~3x3.3 V)
- **WiFi connection** for MQTT broker

---

## ⚙️ Pin Configuration

### Encoders
- Encoder 1: `CHA → GPIO19`, `CHB → GPIO18`  
- Encoder 2: `CHA → GPIO4`, `CHB → GPIO5`

### Motor Driver (TB6612FNG)
- Motor A: `PWMA → GPIO25`, `A01 → GPIO26`, `A02 → GPIO27`  
- Motor B: `PWMB → GPIO32`, `A01_B → GPIO14`, `A02_B → GPIO12`

---

## 📊 Controller Gains

The following gains were tested in different sessions:  

- **k1** = -47, -47.5  
- **k2** = -38, -38.5  
- **k3** = -89, -88.5  
- **k4** = -44, -44.5, -45  

### Test conditions:
- Test 1 → `Vreal = 11.61V`, `Vm = 12.33V`  
- Test 2 → `Vreal = 12.03V`, `Vm = 12.776V`

---

## ⚡ Installation & Usage

1. Clone this repository into your development environment (e.g., PlatformIO).  
2. Install the required libraries:
   - `XSpaceIoT` from TheXspaceAcademy 
   - `I2Cdev`
   - `MPU6050_6Axis_MotionApps612`
3. Update WiFi credentials in the code:
   ```cpp
   const char* WIFI_SSID = "YOUR_SSID";
   const char* WIFI_PASSWORD = "YOUR_PASSWORD";

4. Upload the code to the ESP32.
5. Connect to the MQTT broker (www.xspace.pe:1883).
6. Send references and gains from an MQTT client (e.g., MQTT Explorer or Node-RED).

## 📈 Main ESP32 Tasks

ss_error → digital state-space servocontroller with state error feedback.

genera_x_ref → Smooths position reference to avoid abrupt steps.

prueba_sensores → Reads and prints sensor values.

prueba_motor → Tests motor control.

valida → Validates orientation and angular velocity.


