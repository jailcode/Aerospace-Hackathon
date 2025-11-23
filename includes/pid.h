#ifndef PID_H
#define PID_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>

typedef struct s_pid_controller
{
    double Kp;          // Proportional gain
    double Ki;          // Integral gain
    double Kd;          // Derivative gain
    double setpoint;    // Desired target value
    double integral;    // Integral term
    double last_error;  // Last error value
    double output;      // Control output
    double time_step;   // Time step for calculations
    double simulation; // Simulation start time
} t_pid_controller;

typedef struct s_sensor_data
{
    double gyro[6];     // Gyroscope x, y, z
    double accel[6];    // Accelerometer x, y, z
} t_sensor_data;

typedef struct s_actuator_output
{
    double gimbal_x;    // Gimbal angle X
    double gimbal_y;    // Gimbal angle Y
    double fin_angle;   // Fin angle
} t_actuator_output;

typedef struct s_attitude_state
{
    double measured[3];     // Current measured attitude (pitch, yaw, roll)
    double expected[3];     // Expected attitude
    double error[3];        // Error values
    double last_error[3];   // Previous error for derivative
    double magnitude;       // Angular acceleration magnitude (z-axis)
    long long timestamp[2]; // Current and previous timestamps
} t_attitude_state;

// PID initialization functions (pid.c)
t_pid_controller    *init_pid(t_pid_controller *pid, int size);
int                 *init_error(int size);
t_attitude_state    *init_attitude_state();
double              pd_controller(double error, double derivative, double Kp, double Kd);

// Time utility functions (time_utils.c)
long long           get_current_time();
long long           get_time_difference(long long old_time, long long new_time);

// Calculation functions (calculations.c)

double delta_time(double initial_time, double final_time);
double delta_acceleration(double initial_velocity, double final_velocity);
double delta_angular_position(double initial_angular_position, double final_angular_position);
double angular_velocity(double delta_angular_position, double delta_time);
double delta_angular_velocity(double initial_angular_velocity, double final_angular_velocity);
double angular_acceleration(double delta_angular_velocity, double delta_time);
// Sensor functions (sensor.c)
void                parse_sensor_data(double *gyro, double *accel, const char *data);
void                wait_for_sensor_signal();
int                 read_sensor_line(FILE *gyro_file, FILE *accel_file, t_sensor_data *data);
FILE                *open_sensor_file(const char *filename, const char *mode);

// Processing functions (processing.c)
void                process_attitude(t_attitude_state *state, t_sensor_data *sensor, 
                                    t_pid_controller *pid, t_actuator_output *output, int first_loop);
void                write_output(FILE *output_file, t_actuator_output *output);
void                run_control_loop();

#endif
