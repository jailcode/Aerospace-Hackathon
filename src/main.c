#include "../includes/pid.h"
#include <math.h>

void	integral_windup(int axis, int integral_max);

double	integral[3] = {0};
double	integral_max = 5;
double	prev_error[3] = {0};

double	thrust = 7890; // N

// PID gains
double	Kp = 1.0f;
double	Ki = 0.0f;
double	Kd = 0.1f;

double	angle_Kp = 1.0;

double	outer_setpoint[3] = {0.0, 0.0, 0.0};
// double	inner_setpoint[3];
// Physical paramters
// double	thrust = 30.0;
double	MAX_u_value = 10.0;
double	Ixx = 13.452;
double	Iyy = 13.452;
double	Izz = 0.847;
double	theta_max = 0.1; // maximum deflection for gimbal in rad
						// we start with 10 but make it smaller/bigger as testing goes on;
double	h_COM = 0.561;
// we need to define a function for this that calculates values as it changes
// need a function for normalising a given value to -1 and 1

// need to add a filder for sensor noise
double	PID(int axis, double setpoint, double measurement, double dt)
{
	double	error;
	double	P;
	double	I;
	double	derivative;
	double	D;

	error = setpoint - measurement;
	P = Kp * error;
	integral[axis] += error * dt;
	integral_windup(axis, integral_max);
	I = Ki * integral[axis];
	derivative = (error - prev_error[axis]) / fmax(dt, 1e-6);
	D = Kd * derivative;
	prev_error[axis] = error;
	return (P + I + D);
}

void	integral_windup(int axis, int integral_max)
{
	if (integral[axis] > integral_max)
		integral[axis] = integral_max;
	if (integral[axis] < -integral_max)
		integral[axis] = -integral_max;
}

double	normalise(double u, double max_value, double lowerlimit,
		double upperlimit)
{
	double	normalized;

	if (u > max_value)
		u = max_value;
	if (u < -max_value)
		u = -max_value;
	normalized = u / max_value;
	if (lowerlimit != -1 || upperlimit != 1)
		normalized = lowerlimit + (normalized + 1.0) * (upperlimit - lowerlimit)
			/ 2.0;
	return (normalized);
}

double	alpha = 0.7;
double	filtered_gyro[3] = {0};

double	apply_filter_gyro(int axis, double measurement)
{
	filtered_gyro[axis] = alpha * filtered_gyro[axis] + (1 - alpha)
		* measurement;
	return (filtered_gyro[axis]);
}

double	saturation_check(double theta, double theta_max)
{
	if (theta > theta_max)
		theta = theta_max;
	if (theta < -theta_max)
		theta = -theta_max;
	return (theta);
}

double	torque_to_gimbal_angle(double expected_alpha, double axis_MOI)
{
	double	tau_required;
	double	theta;

	// since torque is = alpha * MOI;
	tau_required = expected_alpha * axis_MOI;
	// gimbal produces torque so: tau = thrust * h_COM * sin(theta)
	theta = asin(fmax(fmin(tau_required / (thrust * h_COM), 1.0), -1.0));
	theta = saturation_check(theta, theta_max);
	return (theta);
}

void	run_inner_loop(float dt)
{
	t_sensor_data	data;
	FILE			*gyro_file;
	FILE			*accel_file;
	double			inner_setpoint[3] = {0.0, 0.0, 0.0};
	double			rate_measurement;
	double			u;
	double			angle;
	double			angle_error;
    double          gimbal_angle;

	if (dt <= 0)
		dt = 0.01;
	gyro_file = open_sensor_file("gyroscope.txt", "r");
	accel_file = open_sensor_file("accelerometer.txt", "r");
	if (!gyro_file || !accel_file)
		return ;
	while (read_sensor_line(gyro_file, accel_file, &data))
	{
		for (int axis = 0; axis < 3; axis++)
		{
			angle = data.accel[axis];
			angle_error = outer_setpoint[axis] - angle;
			inner_setpoint[axis] = angle_Kp * angle_error;
			
            rate_measurement = apply_filter_gyro(axis, data.gyro[axis]);
			u = PID(axis, inner_setpoint[axis], rate_measurement, dt);
			
            if (axis == 0)
                gimbal_angle = torque_to_gimbal_angle(u, Ixx);
            else if (axis == 1)
                gimbal_angle = torque_to_gimbal_angle(u, Iyy);
            else gimbal_angle = torque_to_gimbal_angle(u, Izz);

            u = normalise(gimbal_angle, MAX_u_value, -1, 1);
			//u = saturation_check(u, MAX_u_value);

			printf("Axis %d: gyro=%f rate_cmd=%f gimbal_angle%f servo_norm=%f\n", axis, rate_measurement,
				inner_setpoint[axis], gimbal_angle, u);
		}
		usleep((unsigned int)(dt * 1000000));
	}
	fclose(gyro_file);
	fclose(accel_file);
}

int	main(void)
{
	float	dt;

	printf("==============================================\n");
	printf("  Rocket Ascent Vector Control System\n");
	printf("  TVC PD Controller with Real-time Processing\n");
	printf("==============================================\n\n");
	dt = 0.1;
	// Run the main control loop
	// run_control_loop(dt);
	run_inner_loop(dt);
	// apparently rocks use two PID loops called cascaded control
	return (0);
}
