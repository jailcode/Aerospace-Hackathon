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





//function definitions:
t_pid_controller    *init_pid(t_pid_controller *pid, int size);

#endif PID_H
