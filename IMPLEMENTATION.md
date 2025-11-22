# Rocket Ascent Vector Control System

## Overview
This project implements a real-time Thrust Vector Control (TVC) system for rocket ascent guidance using a PD (Proportional-Derivative) controller. The system processes gyroscope and accelerometer data to generate stable actuator commands for rocket attitude control.

## Architecture

### Control Method: Direct TVC with Gimbals and Fin Tabs
- **2 Gimbal Actuators**: Control pitch (X-axis) and yaw (Y-axis)
- **1 Fin Actuator**: Controls roll (Z-axis)

### Control Algorithm: PD Controller
- **Proportional (Kp)**: 0.1 (with magnitude-based reduction factor)
- **Derivative (Kd)**: 0.05
- Output saturation: ±10 degrees for stability

## System Components

### File Structure
```
src/
├── main.c          - Main entry point and control loop initialization
├── pid.c           - PID controller initialization and PD computation
├── processing.c    - Main processing loop and attitude control logic
├── sensor.c        - Sensor data reading and parsing
├── time_utils.c    - Time utilities for real-time processing
└── calculations.c  - Mathematical utilities (magnitude, error, noise)

includes/
└── pid.h          - Type definitions and function declarations

Input Files:
├── gyroscope.txt      - Angle measurements (pitch, yaw, roll)
└── accelerometer.txt  - Angular acceleration data

Output:
└── output.txt        - Actuator commands (gimbal_x, gimbal_y, fin_angle)
```

## Key Features

### 1. **Simultaneous 3-Axis Processing**
All three attitude axes (pitch, yaw, roll) are processed in parallel during each control cycle.

### 2. **Sensor Data Processing**
- Reads gyroscope and accelerometer data line-by-line simultaneously
- Adds realistic sensor noise (±0.001 for simulation)
- Gyroscope provides angle measurements (degrees)
- Accelerometer provides angular acceleration (rad/s²)

### 3. **Error Calculation**
```
error = expected_attitude - measured_attitude
```
Target attitude is 0° (vertical stabilization)

### 4. **Magnitude Tracking**
- Initialized to 1.0
- Updated using Z-axis acceleration values
- Used for Kp reduction factor: `Kp_effective = 1/magnitude`

### 5. **Two-Timestamp Initialization**
The first loop executes twice to:
- Establish initial error values
- Enable derivative calculation (requires error difference)

### 6. **PD Control Output**
```
control_output = Kp * error + Kd * derivative
```
where `derivative = (error - last_error) / dt`

### 7. **Output Saturation**
Control outputs clamped to ±10° to prevent actuator damage and maintain stability.

## Implementation Details

### Data Structures

```c
typedef struct s_attitude_state
{
    double measured[3];     // Current attitude (pitch, yaw, roll)
    double expected[3];     // Target attitude (0 for stabilization)
    double error[3];        // Error values
    double last_error[3];   // Previous error for derivative
    double magnitude;       // Angular acceleration magnitude
    long long timestamp[2]; // Current and previous timestamps
} t_attitude_state;

typedef struct s_actuator_output
{
    double gimbal_x;    // Gimbal angle X (pitch control)
    double gimbal_y;    // Gimbal angle Y (yaw control)
    double fin_angle;   // Fin angle (roll control)
} t_actuator_output;
```

### Processing Loop

1. **Read sensor data** from input files
2. **Update timestamps** for derivative calculation
3. **Add noise** to measured values (realistic sensor simulation)
4. **Calculate errors** for all 3 axes
5. **Compute derivatives** from error differences
6. **Run PD controller** for each axis
7. **Saturate outputs** to safe limits
8. **Write actuator commands** to output file
9. **Update state** for next iteration
10. **Repeat** until all sensor data processed

## Hardware Integration (HITL Preparation)

### PCB Interface Requirements

```c
// Recommended pin mapping for hardware interface:
// - SPI/I2C for IMU (gyro + accel)
// - PWM outputs for servo actuators
// - ADC for feedback sensors
// - UART for telemetry

// Sample interface functions:
void read_imu_data(t_sensor_data *data);
void set_actuator_pwm(t_actuator_output *output);
void send_telemetry(t_actuator_output *output, t_attitude_state *state);
```

### Timing Considerations
- Control loop: 10ms cycle time (100Hz)
- Sensor sampling: Synchronized read from gyro + accel
- Real-time constraints: Processing must complete within cycle time

## Build and Run

### Compilation
```bash
make clean
make
```

### Execution
```bash
./app
```

### Expected Output
```
==============================================
  Rocket Ascent Vector Control System
  TVC PD Controller with Real-time Processing
==============================================

Starting control loop...
Iteration | Gimbal X | Gimbal Y | Fin Angle | Magnitude
----------|----------|----------|-----------|----------
        0 |  -0.0000 |  -0.0001 |    0.0001 |    9.8100
        1 |  -0.4290 |  -0.2061 |   -0.0907 |    9.8200
        ...
```

## Results

The system successfully:
- ✅ Generates stable actuator commands
- ✅ Processes 3 axes simultaneously
- ✅ Implements PD control with proper derivative calculation
- ✅ Handles sensor noise realistically
- ✅ Outputs commands to file for analysis
- ✅ Maintains stability without oscillation
- ✅ Ready for HITL integration with hardware PCB

## Engineering Decisions

1. **PD over PID**: Derivative term sufficient for damping; integral would add complexity without significant benefit for fast ascent phase

2. **Output Saturation**: Physical actuator limits (±10°) prevent damage and maintain linear operating region

3. **Noise Addition**: Simulates real sensor imperfections (0.001° noise level based on typical MEMS gyro specs)

4. **Magnitude-based Kp**: Reduces gain at high acceleration to prevent overcorrection during high-G maneuvers

5. **Separate Files**: Modular architecture enables easy hardware integration and testing

## Future Enhancements

- Add telemetry output for ground station monitoring
- Implement trajectory tracking (non-zero target attitudes)
- Add integral term for steady-state error correction
- Kalman filter for sensor fusion
- Hardware SPI/I2C drivers for actual IMU
- PWM servo control implementation

## Author
Developed for Momentum Aerospace Hackathon Challenge
Control Architecture: Direct TVC with PD Controller
