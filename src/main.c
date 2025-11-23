#include "../includes/pid.h"

double	integral[3] = {0};
double	prev_error[3] = {0};
// PID gains
double	Kp = 1.0f;
double	Ki = 0.0f;
double	Kd = 0.1f;

#define ENGINE_THRUST 30;

double	h_COM = 0.561;
// we need to define a function for this that calculates values as it changes
// need a function for normalising a given value to -1 and 1
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


double  normalise(double u, double max_value, double lowerlimit, double upperlimit)
{
    
}

void	run_inner_loop(float dt)
{
	t_sensor_data	data;
	FILE			*gyro_file;
	FILE			*accel_file;
	double			setpoint[3] = {0.0, 0.0, 0.0};
	double			measurement;
	double			u;
    double          MAX_u_value; //we need this for normalising it
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
			u = PID(axis, setpoint[axis], measurement, dt);
            u = normalise(u, MAX_u_value, -1, 1);
			printf("Axis %d: gyro=%f PID output=%f\n", axis, measurement, u);
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
	run_innter_loop(dt);
		// apparently rocks use two PID loops called cascaded control
	return (0);
}
