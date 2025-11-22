#include "../includes/pid.h"

int *init_error(int size)
{
    int *error;
    error = (int *)malloc(size * sizeof(int));
    for (int i = 0; i < size; i++)
        error[i] = 0;
    return (error);
}

t_pid_controller *init_pid(t_pid_controller *pid, int size)
{
    int i = -1;

    pid = (t_pid_controller *)malloc(sizeof(t_pid_controller) * size);
    if (!pid)
        return (NULL);
    while(++i < size)
    {
        pid[i].Kp = 1.0;
        pid[i].Ki = 0.0;
        pid[i].Kd = 0.0;
        pid[i].setpoint = 0.0;
        pid[i].integral = 0.0;
        pid[i].last_error = 0.0;
        pid[i].output = 0.0;
        pid[i].time_step = 0.01;  
    }
    return (pid);
}
