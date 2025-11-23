#include "../includes/pid.h"

void parse_sensor_data(double *gyroscope, double *accelerometer, const char *data){
    //Ax,Ay,Az (Angular) & ax,ay,az(Linear) (linear for ax and ay is not needed)

    if(sscanf(data,"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf%lf",
        &gyroscope[0],&gyroscope[1],&gyroscope[2],
        //&gyroscope[3],&gyroscope[4],&gyroscope[5],
        &accelerometer[0],&accelerometer[1],&accelerometer[2],
        &accelerometer[3],&accelerometer[4],&accelerometer[5]) != 6
    )
    {
        fprintf(stderr,"Data Parsing Error");
    };

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
    
    // gyroscope data (angles)
    if (sscanf(gyro_line, "%lf,%lf,%lf", 
               &data->gyro[0], &data->gyro[1], &data->gyro[2]) != 3)
    {
        fprintf(stderr, "Error parsing gyroscope line\n");
        return 0;
    }
    
    // accelerometer data (angular acceleration)
    if (sscanf(accel_line, "%lf,%lf,%lf",
               &data->accel[0], &data->accel[1], &data->accel[2]) != 3)
    {
        fprintf(stderr, "Error parsing accelerometer line\n");
        return 0;
    }
    
    return 1;
}
