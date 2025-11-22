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

FILE *open_sensor_file(const char *filename, const char *mode)
{
    FILE *file = fopen(filename, mode);
    if (!file)
    {
        fprintf(stderr, "Error opening file: %s\n", filename);
        return NULL;
    }
    return file;
}

int read_sensor_line(FILE *gyro_file, FILE *accel_file, t_sensor_data *data)
{
    char gyro_line[256];
    char accel_line[256];
    
    if (!fgets(gyro_line, sizeof(gyro_line), gyro_file))
        return 0;
    if (!fgets(accel_line, sizeof(accel_line), accel_file))
        return 0;
    
    // Parse gyroscope data (angles)
    if (sscanf(gyro_line, "%lf,%lf,%lf", 
               &data->gyro[0], &data->gyro[1], &data->gyro[2]) != 3)
    {
        fprintf(stderr, "Error parsing gyroscope line\n");
        return 0;
    }
    
    // Parse accelerometer data (angular acceleration)
    if (sscanf(accel_line, "%lf,%lf,%lf",
               &data->accel[0], &data->accel[1], &data->accel[2]) != 3)
    {
        fprintf(stderr, "Error parsing accelerometer line\n");
        return 0;
    }
    
    return 1;
}
