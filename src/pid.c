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
        pid[i].Kp = 0.1;   // Reduced proportional gain for stability
        pid[i].Ki = 0.0;
        pid[i].Kd = 0.05;  // Reduced derivative gain
        pid[i].setpoint = 0.0;
        pid[i].integral = 0.0;
        pid[i].last_error = 0.0;
        pid[i].output = 0.0;
        pid[i].time_step = 0.01;  
    }
    return (pid);
}

t_attitude_state *init_attitude_state()
{
    t_attitude_state *state = (t_attitude_state *)malloc(sizeof(t_attitude_state));
    if (!state)
        return NULL;
    
    for (int i = 0; i < 3; i++)
    {
        state->measured[i] = 0.0;
        state->expected[i] = 0.0;
        state->error[i] = 0.0;
        state->last_error[i] = 0.0;
    }
    state->magnitude = 1.0;  // Initialize to 1
    state->timestamp[0] = 0;
    state->timestamp[1] = 0;
    
    return state;
}

double pd_controller(double error, double derivative, double Kp, double Kd)
{
    return Kp * error + Kd * derivative;
}
