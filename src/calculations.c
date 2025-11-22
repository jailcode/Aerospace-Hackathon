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

double add_noise(double value, double noise_level)
{
    // Generate random noise between -noise_level and +noise_level
    double noise = ((double)rand() / RAND_MAX) * 2.0 * noise_level - noise_level;
    return value + noise;
}

void update_magnitude(t_attitude_state *state, double az)
{
    // Update magnitude based on z-axis acceleration
    // Initialize to 1.0 if first time, otherwise update
    if (state->magnitude == 0.0)
        state->magnitude = 1.0;
    else
        state->magnitude = fabs(az);
}
