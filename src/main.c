#include "../includes/pid.h"


double get_current_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
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

int main(void)
{
    double time = get_current_time();
    printf("Current time: %f seconds\n", time);
    return (0);
}