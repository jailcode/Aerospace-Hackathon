

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>

#ifndef PID_H
#define PID_H

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


typedef struct s_error
{
    double error_pitch;
    double error_roll;
    double error_yaw;
} t_error;


//function definitions:
void    init_pid(t_pid_controller *pid);
void    init_error(t_error *error);

#endif PID_H