# Rocket Ascent Vector Control System - Technical Implementation

## Overview

This project implements a **Cascaded PID Control System** for rocket Thrust Vector Control (TVC) using a two-loop architecture commonly used in aerospace applications:
- **Outer Loop (Slow):** Attitude angle control
- **Inner Loop (Fast):** Angular rate control with gimbal actuator commands

**Developed for:** Momentum Aerospace Hackathon Challenge  
**Control Architecture:** Cascaded PID with torque-to-gimbal angle conversion  
**Programming Language:** C  
**Real-time Performance:** Configurable time step (default: 0.1s / 10 Hz)

---

## System Architecture

### Cascaded Control Loop Structure

```
┌──────────────────────────────────────────────────────────────┐
│                   OUTER LOOP (Angle Control)                 │
│                                                              │
│  outer_setpoint[3] = {0, 0, 0}  ────► angle_error            │
│                                        ▼                     │
│                              angle_Kp × angle_error          │
│                                        ▼                     │
│                              inner_setpoint (rate command)   │
└─────────────────────────────┬────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────┐
│                   INNER LOOP (Rate Control)                  │
│                                                              │
│  Sensor Reading (Gyro + Accel)                               │
│            │                                                 │
│            ▼                                                 │
│  Low-pass Filter (α=0.7)                                     │
│            │                                                 │
│            ▼                                                 │
│  PID Controller (rate_measurement → u)                       │
│            │                                                 │
│            ▼                                                 │
│  Torque-to-Gimbal Angle Conversion                           │
│            │                                                 │
│            ▼                                                 │
│  Normalization & Saturation                                  │
│            │                                                 │
│            ▼                                                 │
│  Servo Commands [-1, 1]                                      │
└──────────────────────────────────────────────────────────────┘
```

---

## Key Components

### 1. Physical Parameters

```c
// Rocket Physical Properties
double thrust = 7890;          // N - Motor thrust
double Ixx = 13.452;           // kg⋅m² - Pitch moment of inertia
double Iyy = 13.452;           // kg⋅m² - Yaw moment of inertia  
double Izz = 0.847;            // kg⋅m² - Roll moment of inertia
double h_COM = 0.561;          // m - Distance from gimbal to center of mass
double theta_max = 0.1;        // rad - Maximum gimbal deflection (≈5.7°)
double MAX_u_value = 10.0;     // Maximum control signal value
```

**Note:** These are example values. Replace with your actual rocket specifications.

### 2. PID Controller Parameters

```c
// PID Gains
double Kp = 1.0;               // Proportional gain
double Ki = 0.0;               // Integral gain (currently disabled)
double Kd = 0.1;               // Derivative gain

// Outer loop gain
double angle_Kp = 1.0;         // Angle-to-rate conversion gain

// Integral anti-windup
double integral_max = 5.0;     // Maximum integral accumulation
```

### 3. Sensor Filtering

```c
// Low-pass filter for gyroscope data
double alpha = 0.7;            // Filter coefficient (0-1)
```
- Higher α = more filtering (smoother but slower response)
- Lower α = less filtering (faster but noisier response)

---

## Control Algorithm

### Outer Loop: Attitude Control

The outer loop converts angle errors to rate commands:

```c
// For each axis (pitch, yaw, roll)
angle = data.accel[current_axis];                     // Current angle from sensor
angle_error = outer_setpoint[current_axis] - angle;  // Error calculation
inner_setpoint[current_axis] = angle_Kp * angle_error; // Rate command
```

**Purpose:** Maintains desired rocket orientation (typically vertical: 0°, 0°, 0°)

### Inner Loop: Rate Control

The inner loop implements a full PID controller:

```c
double PID(int current_axis, double setpoint, double measurement, double delta_time)
{
    // Calculate error
    double current_error = setpoint - measurement;
    
    // Proportional term
    double P = Kp * current_error;
    
    // Integral term (with anti-windup)
    integral[current_axis] += current_error * delta_time;
    integral_windup(current_axis, integral_max);
    double I = Ki * integral[current_axis];
    
    // Derivative term
    double derivative = (current_error - previous_error[current_axis]) 
                       / fmax(delta_time, 1e-6);
    double D = Kd * derivative;
    
    // Store error for next iteration
    previous_error[current_axis] = current_error;
    
    return (P + I + D);
}
```

**Key Features:**
- **Anti-windup:** Prevents integral term from growing unbounded
- **Derivative protection:** Uses `fmax(delta_time, 1e-6)` to prevent division by zero
- **Per-axis state:** Separate integral and previous error for each axis

### Sensor Filtering

Applies exponential moving average to gyroscope readings:

```c
double apply_filter_gyro(int current_axis, double measurement)
{
    filtered_gyro[current_axis] = alpha * filtered_gyro[current_axis] 
                                  + (1 - alpha) * measurement;
    return filtered_gyro[current_axis];
}
```

**Formula:** `y[n] = α × y[n-1] + (1-α) × x[n]`

---

## Torque-to-Gimbal Angle Conversion

### Physics

The gimbal produces torque through thrust vectoring:

```
τ_required = α × I        (Torque = Angular Acceleration × Moment of Inertia)
τ_gimbal = T × h × sin(θ)  (Torque = Thrust × Moment Arm × sin(Gimbal Angle))
```

Solving for gimbal angle:

```
θ = arcsin(τ_required / (T × h))
```

### Implementation

```c
double torque_to_gimbal_angle(double expected_alpha, double current_axis_MOI)
{
    // Calculate required torque
    double tau_required = expected_alpha * current_axis_MOI;
    
    // Calculate gimbal angle: θ = arcsin(τ / (thrust × h_COM))
    double theta = asin(fmax(fmin(tau_required / (thrust * h_COM), 1.0), -1.0));
    
    // Apply saturation
    theta = saturation_check(theta, theta_max);
    
    return theta;
}
```

**Safety Features:**
- `fmax(fmin(...))`: Clamps input to arcsin domain [-1, 1]
- `saturation_check()`: Limits output to ±theta_max

---

## Normalization & Saturation

### Normalization

Converts control signal to servo range:

```c
double normalise(double u, double max_value, double lowerlimit, double upperlimit)
{
    // Saturate input
    if (u > max_value) u = max_value;
    if (u < -max_value) u = -max_value;
    
    // Normalize to [-1, 1]
    double normalized = u / max_value;
    
    // Map to custom range if needed
    if (lowerlimit != -1 || upperlimit != 1)
        normalized = lowerlimit + (normalized + 1.0) * (upperlimit - lowerlimit) / 2.0;
    
    return normalized;
}
```

**Default Output:** [-1, 1] for servo PWM control

---

## Data Structures

### Sensor Data

```c
typedef struct s_sensor_data
{
    double gyro[6];    // Angular rates: [pitch, yaw, roll, reserved...]
    double accel[6];   // Angular positions: [pitch, yaw, roll, reserved...]
} t_sensor_data;
```

**File Format:**
- `gyroscope.txt`: 3 values per line (angular rates in rad/s or deg/s)
- `accelerometer.txt`: 3 values per line (angular positions in degrees)

---

## Main Control Loop

```c
void run_inner_loop(float delta_time)
{
    // Open input files
    FILE *gyro_file = open_sensor_file("gyroscope.txt", "r");
    FILE *accel_file = open_sensor_file("accelerometer.txt", "r");
    FILE *plotter = fopen("results.csv", "w");
    
    // CSV header
    fprintf(plotter, "axis,iteration,gyro_measurement,rate_command,gimbal_axis,server_motorized\n");
    
    // Main loop
    while (read_sensor_line(gyro_file, accel_file, &data))
    {
        for (int current_axis = 0; current_axis < 3; current_axis++)
        {
            // 1. Outer loop: Angle error → Rate command
            angle = data.accel[current_axis];
            angle_error = outer_setpoint[current_axis] - angle;
            inner_setpoint[current_axis] = angle_Kp * angle_error;
            
            // 2. Filter sensor data
            rate_measurement = apply_filter_gyro(current_axis, data.gyro[current_axis]);
            
            // 3. Inner loop: PID control
            u = PID(current_axis, inner_setpoint[current_axis], 
                    rate_measurement, delta_time);
            
            // 4. Convert to gimbal angle
            if (current_axis == 0)
                gimbal_angle = torque_to_gimbal_angle(u, Ixx);
            else if (current_axis == 1)
                gimbal_angle = torque_to_gimbal_angle(u, Iyy);
            else 
                gimbal_angle = torque_to_gimbal_angle(u, Izz);
            
            // 5. Normalize for servo
            u = normalise(gimbal_angle, MAX_u_value, -1, 1);
            
            // 6. Log data
            fprintf(plotter, "%d,%d,%f,%f,%f,%f\n", 
                   current_axis, count, rate_measurement, 
                   inner_setpoint[current_axis], gimbal_angle, u);
        }
        
        // Wait for next time step
        usleep((unsigned int)(delta_time * 1000000));
        count++;
    }
}
```

### Loop Execution Flow

1. **Read Sensors:** Get gyro (rates) and accelerometer (angles)
2. **Outer Loop:** Calculate angle error → rate command
3. **Filter:** Apply low-pass filter to gyro data
4. **Inner Loop:** PID control on rate error
5. **Convert to Gimbal:** Torque-to-angle conversion
6. **Normalize:** Scale to servo range [-1, 1]
7. **Log & Output:** Write to CSV for analysis
8. **Sleep:** Wait for next time step

---

## Output Files

### results.csv

```csv
axis,iteration,gyro_measurement,rate_command,gimbal_axis,server_motorized
0,0,0.005000,-0.000100,0.000001,-0.000000
0,1,0.010000,-0.000150,0.000002,-0.000000
...
```

**Columns:**
- `axis`: 0=Pitch, 1=Yaw, 2=Roll
- `iteration`: Loop count
- `gyro_measurement`: Filtered angular rate (rad/s)
- `rate_command`: Inner loop setpoint (rad/s)
- `gimbal_axis`: Commanded gimbal angle (radians)
- `server_motorized`: Normalized servo command [-1, 1]

---

## Tuning Guidelines

### If System Oscillates:
1. **Decrease Kd** (reduce derivative gain)
2. **Decrease Kp** (reduce proportional gain)
3. **Increase alpha** (more filtering, slower response)
4. **Decrease angle_Kp** (slower outer loop)

### If Response is Too Slow:
1. **Increase Kp** (faster response to error)
2. **Decrease alpha** (less filtering)
3. **Increase angle_Kp** (faster outer loop)

### If Steady-State Error Exists:
1. **Increase Ki** (enable integral term)
2. **Ensure integral_max is appropriate**

### General Tuning Process:
1. Start with Ki = 0 (PD controller)
2. Increase Kp until slight oscillation
3. Reduce Kp by 20-30%
4. Add Kd to dampen oscillations
5. Add Ki if steady-state error remains

---

## File Structure

```
Aerospace-Hackathon/
├── src/
│   ├── main.c              # Main control loop & PID implementation
│   ├── sensor.c            # File I/O for gyro & accelerometer
│   ├── calculations.c      # Physics calculations (torque, thrust, etc.)
│   ├── time_utils.c        # Time utilities
│   ├── pid.c               # PID initialization (not used in current impl)
│   ├── processing.c        # Alternative processing logic (commented out)
│   └── test_script.c       # Differential test function
├── includes/
│   └── pid.h               # Type definitions & function declarations
├── gyroscope.txt           # Input: Angular rates
├── accelerometer.txt       # Input: Angular positions
├── results.csv             # Output: Control system data
├── Makefile                # Build system
├── README.md               # Project documentation
└── IMPLEMENTATION.md       # This file
```

---

## Compilation & Execution

```bash
# Clean previous build
make clean

# Compile
make

# Run with default settings
./app

# Output will be written to results.csv
```

---

## Validation & Testing

### differential() Function

Used for gain tuning validation:

```c
void differential(double current, double expected)
{
    double differential = current + expected;
    if (differential > 1)
        printf("Gain is too low\n");
    else if (differential < -1)
        printf("Gain is too high\n");
    else
        printf("Test Succeeded\n");
}
```

Called after each axis calculation to verify control signal magnitude.

---

## Hardware Integration

### Required Interfaces

**Sensors:**
- IMU with gyroscope (3-axis angular rate)
- IMU with accelerometer (3-axis angle measurement)
- I2C or SPI interface

**Actuators:**
- 2× Gimbal servos (pitch, yaw)
- 1× Fin servo or reaction wheel (roll)
- PWM interface (50-400 Hz)

### Sample Hardware Mapping

```c
// Read from hardware IMU
void read_hardware_sensors(t_sensor_data *data)
{
    i2c_read_mpu6050(&data->gyro, &data->accel);
}

// Write to hardware servos
void write_hardware_actuators(double gimbal_x, double gimbal_y, double fin)
{
    pwm_write(SERVO_CHANNEL_1, servo_angle_to_pwm(gimbal_x));
    pwm_write(SERVO_CHANNEL_2, servo_angle_to_pwm(gimbal_y));
    pwm_write(SERVO_CHANNEL_3, servo_angle_to_pwm(fin));
}
```

---

## Physical Rocket Parameters

### How to Determine Your Values

**Moment of Inertia (I):**
1. Use CAD software (SolidWorks, Fusion360) with material properties
2. Or calculate for cylinder: `I = (1/12) × m × L² + (1/4) × m × r²`
3. Or measure experimentally with torsion pendulum

**Moment Arm (h_COM):**
1. Find center of mass by balancing rocket
2. Measure distance from gimbal pivot to COM
3. This is your h_COM value

**Maximum Gimbal Angle (theta_max):**
1. Measure physical gimbal limits
2. Start conservative (5-10°)
3. Increase based on testing

**Thrust:**
1. Check motor datasheet
2. Or measure with load cell
3. Account for variations during flight

---

## Advanced Topics

### Why Cascaded Control?

**Benefits:**
1. **Separation of Concerns:** Angle control vs. rate control
2. **Faster Inner Loop:** Rate control responds quickly to disturbances
3. **Stability:** Easier to tune than single-loop system
4. **Real-world Standard:** Used in quadcopters, aircraft, rockets

### Control Frequencies

- **Outer Loop (Angle):** 10-50 Hz
- **Inner Loop (Rate):** 100-1000 Hz

Current implementation runs both at same rate (configurable delta_time).
For optimal performance, inner loop should run 10× faster than outer loop.

---

## Limitations & Future Work

**Current Limitations:**
- Integral term disabled (Ki = 0)
- Same loop frequency for inner/outer
- File-based I/O (not real-time hardware)

**Future Improvements:**
1. Enable integral control with proper tuning
2. Implement separate loop frequencies
3. Add Kalman filter for sensor fusion
4. Implement feed-forward control
5. Add trajectory tracking (non-zero setpoints)
6. Real-time hardware integration

---

## Contributors

This project was developed collaboratively during the Aerospace Hackathon:

- **Tanmay Pandya** ([@tpandya42](https://github.com/tpandya42)) - Control systems design and implementation
- **Anirudh Agarwal** ([@AnirudhNUS](https://github.com/AnirudhNUS)) - Physics modeling and dynamics calculations
- **Pradhyun** ([@jailcode](https://github.com/jailcode)) - System architecture and integration
- **Vaishnav** ([@Thebroken1](https://github.com/Thebroken1)) - Algorithm development and testing

**Repository:** [github.com/jailcode/Aerospace-Hackathon](https://github.com/jailcode/Aerospace-Hackathon)

---

## References

- Momentum Aerospace Hackathon Challenge
- Modern Control Theory (Ogata)
- Rocket Propulsion Elements (Sutton & Biblarz)
- PID Control Implementation and Tuning (Åström & Hägglund)

---

**Built for aerospace engineering excellence by a dedicated team** 🚀
