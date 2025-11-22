#include "../includes/pid.h"


long long get_current_time()
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
        pid[i].time_step = 0.01; // Example time step
    }
    return (pid);
}

int main(void)
{
    t_pid_controller *pid = NULL;
    int *error;

    error = init_error(3);
    pid = init_pid(pid, 3);
    long long time = get_current_time();
    printf("error: %d %d %d\n", error[0], error[1], error[2]);
    printf("time: %lld\n", time);

    return (0);
}
