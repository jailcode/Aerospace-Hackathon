

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>


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
} t_pid_controller;






#endif PID_H