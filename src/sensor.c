#include "../includes/pid.h"

void parse_sensor_data(double *gyro, double *accel, const char *data)
{
    // Expected format: "gx,gy,gz,ax,ay,az"
    if (sscanf(data, "%lf,%lf,%lf,%lf,%lf,%lf",
               &gyro[0], &gyro[1], &gyro[2],
               &accel[0], &accel[1], &accel[2]) != 6)
    {
        fprintf(stderr, "Error parsing sensor data\n");
    }
}

void wait_for_sensor_signal()
{
    usleep(10000);
}
