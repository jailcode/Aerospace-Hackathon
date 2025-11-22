#include "../includes/pid.h"

double	integral[3] = {0};
double	prev_error[3] = {0};
// PID gains
double	Kp = 1.0f;
double	Ki = 0.0f;
double	Kd = 0.1f;

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
	I = Ki * integral[axis];
	derivative = (error - prev_error[axis]) / dt;
	D = Kd * derivative;
	prev_error[axis] = error;
	return (P + I + D);
}

void	run_loop(float dt)
{
	t_sensor_data	data;
	FILE			*gyro_file;
	FILE			*accel_file;
	double			setpoint[3] = {0.0, 0.0, 0.0};
	double			measurement;
	double			output;

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
			measurement = data.gyro[axis];
			output = PID(axis, setpoint[axis], measurement, dt);
			printf("Axis %d: gyro=%f PID output=%f\n", axis, measurement,
				output);
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
	run_loop(dt);
	return (0);
}
