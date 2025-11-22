#include "../includes/pid.h"

double calculate_magnitude(double x, double y)
{
    return sqrt(x * x + y * y);
}

double reduction_factor_kp(double magnitude)
{
    if (magnitude == 0.0)
        return 0.0;
    return 1.0 / magnitude;
}

double calculate_error(double setpoint, double current_value)
{
    double error = current_value - setpoint;
    return error;
}
