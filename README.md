# 🚀 Rocket Ascent Vector Control System

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()
[![Language](https://img.shields.io/badge/language-C-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)]()

> **Cascaded PID Control System for Thrust Vector Control (TVC)**  
> Developed for **Momentum Aerospace** Hackathon Challenge

A real-time rocket attitude control system implementing industry-standard cascaded control loops for precise thrust vector control during ascent.

---

## 🎯 Overview

This project implements a **two-loop cascaded PID controller** for rocket attitude stabilization:

- **Outer Loop:** Controls rocket orientation (angle/attitude)
- **Inner Loop:** Controls angular rates with gimbal actuators
- **Physics-Based:** Torque-to-gimbal angle conversion using real rocket dynamics
- **Real-Time:** Configurable time steps with sensor filtering
- **Production-Ready:** Anti-windup, saturation, and safety limits

**Use Case:** Small to medium rockets during ascent phase requiring active stabilization

---

## ✨ Key Features

### Control System
- ✅ **Cascaded PID Architecture** (industry standard for rockets/drones)
- ✅ **3-Axis Control** (pitch, yaw, roll)
- ✅ **Adaptive Control** with integral anti-windup
- ✅ **Low-Pass Filtering** for sensor noise rejection
- ✅ **Torque-Based Actuation** with physics calculations

### Safety & Robustness
- ✅ **Output Saturation** (prevents actuator damage)
- ✅ **Gimbal Angle Limits** (configurable max deflection)
- ✅ **Division-by-Zero Protection** in derivatives
- ✅ **Domain Checking** for trigonometric functions

### Data & Logging
- ✅ **CSV Output** for analysis and plotting
- ✅ **Real-Time Console** monitoring
- ✅ **Per-Axis Logging** (6 channels of data)
- ✅ **Test Validation** functions for gain tuning

---

## 📁 Project Structure

```
Aerospace-Hackathon/
├── src/
│   ├── main.c              # Main control loop & PID logic
│   ├── sensor.c            # File I/O for sensor data
│   ├── calculations.c      # Physics & dynamics calculations
│   ├── time_utils.c        # Timing utilities
│   └── test_script.c       # Validation functions
├── includes/
│   └── pid.h               # Type definitions & declarations
├── gyroscope.txt           # Input: Angular rates (3 values/line)
├── accelerometer.txt       # Input: Angular positions (3 values/line)
├── results.csv             # Output: Control data (auto-generated)
├── Makefile                # Build system
├── README.md               # This file
└── IMPLEMENTATION.md       # Detailed technical documentation
```

---

## 🚀 Quick Start

### Prerequisites

- GCC compiler (C99 or later)
- Make
- Unix-like environment (macOS, Linux, WSL)

### Installation & Build

```bash
# Clone the repository
git clone https://github.com/jailcode/Aerospace-Hackathon.git
cd Aerospace-Hackathon

# Build the project
make clean && make

# Run the control system
./app
```

### Expected Output

```
==============================================
  Rocket Ascent Vector Control System
  TVC PD Controller with Real-time Processing
==============================================

Current Iteration: 0
Axis 0: gyro measurement: 0.000000 rate command: 0.000000 gimbal angle: 0.000000 servo normalized: 0.000000
Axis 1: gyro measurement: 0.000000 rate command: 0.000000 gimbal angle: 0.000000 servo normalized: 0.000000
Axis 2: gyro measurement: 0.000000 rate command: 0.000000 gimbal angle: 0.000000 servo normalized: 0.000000

Current Iteration: 1
...
```

Data is logged to `results.csv` for post-flight analysis.

---

## 📊 Input/Output Format

### Input Files

#### `gyroscope.txt` (Angular Rates)
```
0.0,0.0,0.0
0.1,0.05,0.02
0.15,0.08,0.03
...
```
- **Format:** 3 comma-separated values per line
- **Units:** rad/s or deg/s (consistent with gains)
- **Axes:** pitch, yaw, roll

#### `accelerometer.txt` (Angular Positions)
```
0.001,-0.002,9.812
0.005,-0.003,9.810
...
```
- **Format:** 3 comma-separated values per line
- **Units:** degrees
- **Axes:** pitch, yaw, roll

### Output File

#### `results.csv`
```csv
axis,iteration,gyro_measurement,rate_command,gimbal_axis,server_motorized
0,0,0.000000,0.000000,0.000000,0.000000
1,0,0.000000,0.000000,0.000000,0.000000
...
```

**Columns:**
- `axis`: 0=Pitch, 1=Yaw, 2=Roll
- `iteration`: Loop counter
- `gyro_measurement`: Filtered angular rate
- `rate_command`: Inner loop setpoint
- `gimbal_axis`: Gimbal angle command (radians)
- `server_motorized`: Normalized servo signal [-1, 1]

---

## ⚙️ Configuration

### PID Gains (in `src/main.c`)

```c
double Kp = 1.0;          // Proportional gain
double Ki = 0.0;          // Integral gain (currently disabled)
double Kd = 0.1;          // Derivative gain
double angle_Kp = 1.0;    // Outer loop gain
```

### Physical Parameters

```c
double thrust = 7890;     // N - Motor thrust
double Ixx = 13.452;      // kg⋅m² - Pitch inertia
double Iyy = 13.452;      // kg⋅m² - Yaw inertia
double Izz = 0.847;       // kg⋅m² - Roll inertia
double h_COM = 0.561;     // m - Moment arm
double theta_max = 0.1;   // rad - Max gimbal angle (~5.7°)
```

**⚠️ Replace these with your actual rocket specifications!**

### Sensor Filtering

```c
double alpha = 0.7;       // Low-pass filter coefficient [0-1]
```
- Higher α = more filtering (smoother, slower)
- Lower α = less filtering (faster, noisier)

### Time Step

```c
delta_time = 0.1;         // seconds (10 Hz)
```

---

## 🎛️ Control Algorithm

### Two-Loop Architecture

```
┌─────────────────────────────────────┐
│  OUTER LOOP (Attitude Control)     │
│                                     │
│  setpoint[0,0,0] ──► angle_error   │
│                           │         │
│                    angle_Kp × error │
│                           │         │
│                           ▼         │
│                    rate_command     │
└───────────────┬─────────────────────┘
                │
                ▼
┌─────────────────────────────────────┐
│  INNER LOOP (Rate Control)         │
│                                     │
│  Sensor ──► Filter ──► PID ──►     │
│  Torque ──► Gimbal ──► Servo       │
└─────────────────────────────────────┘
```

### PID Controller

```
u(t) = Kp × e(t) + Ki × ∫e(t)dt + Kd × de(t)/dt
```

**Features:**
- Integral anti-windup (prevents saturation)
- Derivative smoothing (prevents noise amplification)
- Per-axis state tracking

### Torque-to-Gimbal Conversion

```
τ_required = α × I
θ_gimbal = arcsin(τ / (T × h))
```

Where:
- τ = Torque (N⋅m)
- α = Angular acceleration (rad/s²)
- I = Moment of inertia (kg⋅m²)
- T = Thrust (N)
- h = Moment arm (m)
- θ = Gimbal angle (rad)

---

## 🔧 Tuning Guide

### Step-by-Step Tuning Process

1. **Start Conservative**
   ```c
   Kp = 0.5, Ki = 0.0, Kd = 0.05
   ```

2. **Increase Kp** until system responds quickly but oscillates slightly

3. **Reduce Kp by 20-30%** for safety margin

4. **Add Kd** to dampen oscillations (typically 10-20% of Kp)

5. **Add Ki** only if steady-state error exists (start small: 0.01-0.1)

### Troubleshooting

| Problem | Solution |
|---------|----------|
| Oscillations | Decrease Kp, increase Kd, increase alpha |
| Slow response | Increase Kp, decrease alpha |
| Overshoot | Increase Kd, decrease Kp |
| Steady-state error | Increase Ki (carefully!) |
| Instability | Reduce all gains by 50%, add filtering |

---

## 📈 Data Analysis

### Plotting Results (Python)

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv('results.csv')

# Plot gimbal angles over time
for axis in [0, 1, 2]:
    data = df[df['axis'] == axis]
    plt.plot(data['iteration'], data['gimbal_axis'], 
             label=f'Axis {axis}')

plt.xlabel('Iteration')
plt.ylabel('Gimbal Angle (rad)')
plt.legend()
plt.grid(True)
plt.show()
```

---

## 🛠️ Hardware Integration

### Required Hardware

**Sensors:**
- IMU with 3-axis gyroscope (e.g., MPU6050, BMI088)
- IMU with 3-axis accelerometer
- I2C or SPI interface

**Actuators:**
- 2× Gimbal servos (pitch, yaw)
- 1× Fin servo or reaction wheel (roll)
- PWM driver (e.g., PCA9685)

### Sample Integration Code

```c
#include "i2c_driver.h"
#include "pwm_driver.h"

void read_hardware_imu(t_sensor_data *data)
{
    mpu6050_read_gyro(&data->gyro[0], &data->gyro[1], &data->gyro[2]);
    mpu6050_read_accel(&data->accel[0], &data->accel[1], &data->accel[2]);
}

void write_hardware_servos(double pitch, double yaw, double roll)
{
    pwm_set_servo(0, angle_to_pwm(pitch));
    pwm_set_servo(1, angle_to_pwm(yaw));
    pwm_set_servo(2, angle_to_pwm(roll));
}
```

---

## 📚 Documentation

- **[IMPLEMENTATION.md](IMPLEMENTATION.md)** - Detailed technical documentation
- **[task.md](task.md)** - Original challenge requirements
- **[tasks_for_me.md](tasks_for_me.md)** - Implementation task breakdown

---

## 🧪 Testing

### Validation Functions

The system includes a `differential()` test function:

```c
differential(rate_measurement, current_axis);
```

Checks if control gains are appropriate:
- "Gain is too low" → Increase Kp
- "Gain is too high" → Decrease Kp
- "Test Succeeded" → Gains are reasonable

---

## 🤝 Contributing

Contributions welcome! This is an open hackathon project.

### Development Guidelines

1. Follow existing code style (K&R)
2. Comment complex algorithms
3. Test with realistic sensor data
4. Update documentation for changes

---

## 📝 License

MIT License - Free to use for educational and commercial purposes.

---

## 👥 Contributors

This project was developed as a team effort during the Aerospace Hackathon:

| Name | GitHub Username | Role |
|------|----------------|------|
| **Tanmay Pandya** | [@tpandya42](https://github.com/tpandya42) | Control Systems & Implementation |
| **Anirudh Agarwal** | [@AnirudhNUS](https://github.com/AnirudhNUS) | Physics & Dynamics |
| **Pradhyun** | [@jailcode](https://github.com/jailcode) | Architecture & Integration |
| **Vaishnav** | [@Thebroken1](https://github.com/Thebroken1) | Algorithms & Testing |

---

## 🙏 Acknowledgments

**Repository:** [github.com/jailcode/Aerospace-Hackathon](https://github.com/jailcode/Aerospace-Hackathon)  
**Branch:** `tan`

**Special Thanks:**
- **Momentum Aerospace** - For providing this exciting challenge
- **Hackathon Organizers** - For the opportunity and support
- **Open-source Aerospace Community** - For inspiration and resources

---

## 📞 Support

For technical questions:
1. Check [IMPLEMENTATION.md](IMPLEMENTATION.md) for details
2. Review code comments in `src/main.c`
3. Open an issue on GitHub

---

## 🔗 Related Projects

- [OpenRocket](http://openrocket.info/) - Flight simulation
- [ArduPilot](https://ardupilot.org/) - Open-source autopilot
- [BetaFlight](https://betaflight.com/) - Flight controller firmware

---

**Built with precision engineering for aerospace applications** 🚀🛰️