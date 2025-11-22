#include "../includes/pid.h"

long long get_current_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000); // Return milliseconds
}

long long get_time_difference(long long old_time, long long new_time)
{
    return new_time - old_time;
}
