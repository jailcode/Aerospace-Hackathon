#include "../includes/pid.h"


double get_current_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}


int    *init_error(int size)
{
    int *error;
    error = (int *)malloc(size * sizeof(int));
    for (int i = 0; i < size; i++)
        error[i] = 0;
    return (error);
}

t_pid_controller *init_pid(t_pid_controller *pid)
{
    pid = (t_pid_controller *)malloc(sizeof(t_pid_controller));
    pid->Kp = 1.0;
    pid->Ki = 0.0;
    pid->Kd = 0.0;
    pid->setpoint = 0.0;
    pid->integral = 0.0;
    pid->last_error = 0.0;
    pid->output = 0.0;
    pid->time_step = 0.01; // Example time step
    //pid->simulation = get_current_time();
    return (pid);
}

int main(void)
{
    t_pid_controller *pid;
    int *error;

    error = init_error(3);
    pid = init_pid(pid);
    init_pid(&pid);
    double time = get_current_time();
    
    printf("time: %f\n", time);
    start_loop()

    return (0);
}


void    init_pid(t_pid_controller *pid)
{
    pid->Kp = 1.0;
    pid->Ki = 0.0;
    pid->Kd = 0.0;
    pid->setpoint = 0.0;
    pid->integral = 0.0;
    pid->last_error = 0.0;
    pid->output = 0.0;
}

void    init_error(t_error *error)
{
    error->error_pitch = 0.0;
    error->error_roll = 0.0;
    error->error_yaw = 0.0;
}