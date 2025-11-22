#include "../includes/pid.h"

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
