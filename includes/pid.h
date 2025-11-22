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

// PID initialization functions (pid.c)
t_pid_controller    *init_pid(t_pid_controller *pid, int size);
int                 *init_error(int size);

// Time utility functions (time_utils.c)
long long           get_current_time();
long long           get_time_difference(long long old_time, long long new_time);

// Calculation functions (calculations.c)
double              calculate_magnitude(double x, double y, double z);
double              calculate_kp(double magnitude);
double              calculate_error(double setpoint, double current_value, double *integral);

// Sensor functions (sensor.c)
void                parse_sensor_data(double *gyro, double *accel, const char *data);
void                wait_for_sensor_signal();

#endif
