#include "../includes/pid.h"

double calculate_magnitude(double x, double y, double z)
{
    return sqrt(x * x + y * y + z * z);
}

double calculate_kp(double magnitude)
{
    if (magnitude == 0.0)
        return 0.0;
    return 1.0 / magnitude;
}

double calculate_error(double setpoint, double current_value, double *integral)
{
    double error = setpoint - current_value;
    if (integral != NULL)
        *integral += error;
    return error;
}
