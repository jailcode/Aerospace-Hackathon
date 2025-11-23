#include "../includes/pid.h"
#include <math.h>

void	integral_windup(int current_axis, int integral_max);

double	integral[3] = {0};
double	integral_max = 5;
double	previous_error[3] = {0};

double	thrust = 7890; // N

int count = 0;

// PID gains
double	Kp[3]= {1.0,1.0,1.0}; //proportional
double	Ki[3]= {0.0,0.0,0.0}; //integral
double	Kd[3]= {0.2,0.2,0.2}; //derivative

double	angle_Kp = 1.0;

double	outer_setpoint[3] = {0.0, 0.0, 0.0};
// double	inner_setpoint[3];
// Physical paramters
// double	thrust = 30.0;
double	MAX_u_value = 10.0;

# define NUMBER_OF_AXES 3

double	Ixx = 13.452;
double	Iyy = 13.452;
double	Izz = 0.847;
double	theta_max = 0.1; // maximum deflection for gimbal in rad
						// we start with 10 but make it smaller/bigger as testing goes on;
double	h_COM = 0.0863;
// we need to define a function for this that calculates values as it changes

double	PID(int current_axis, double setpoint, double measurement, double delta_time)
{
	double	current_error;
	double	P;
	double	I;
	double	derivative;
	double	D;

	current_error = setpoint - measurement;
	P = Kp[current_axis] * current_error;
	integral[current_axis] += current_error * delta_time;
	I = Ki[current_axis] * integral[current_axis];
    integral_windup(current_axis, integral_max);
	derivative = (current_error - previous_error[current_axis]) / fmax(delta_time, 1e-6);
    saturation_check(derivative, 1000.0); // to avoid extreme derivative spikes
	D = Kd[current_axis] * derivative;
	previous_error[current_axis] = current_error;
	return (P + I + D);
}

void	integral_windup(int current_axis, int integral_max)
{
	if (integral[current_axis] > integral_max)
		integral[current_axis] = integral_max;
	if (integral[current_axis] < -integral_max)
		integral[current_axis] = -integral_max;
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

double	apply_filter_gyro(int current_axis, double measurement)
{
	filtered_gyro[current_axis] = alpha * filtered_gyro[current_axis] + (1 - alpha)
		* measurement;
	return (filtered_gyro[current_axis]);
}



double	saturation_check(double theta, double theta_max)
{
	if (theta > theta_max)
		theta = theta_max;
	if (theta < -theta_max)
		theta = -theta_max;
	return (theta);
}

double	torque_to_gimbal_angle(double expected_alpha, double current_axis_MOI)
{
	double	tau_required;
	double	theta;

	// since torque is = alpha * MOI;
	tau_required = expected_alpha * current_axis_MOI;
	// gimbal produces torque so: tau = thrust * h_COM * sin(theta)
	theta = asin(fmax(fmin(tau_required / (thrust * h_COM), 1.0), -1.0));
	theta = saturation_check(theta, theta_max); // this is there to limit the actual gimbel angle
	return (theta);
}

void open_files(FILE **gyro_file, FILE **accel_file)
{
    *gyro_file = open_sensor_file("gyroscope.txt", "r");
    *accel_file = open_sensor_file("accelerometer.txt", "r");
    if (!*gyro_file || !*accel_file)
    {
        fprintf(stderr, "Failed to open sensor files\n");
        return ;
    }
}

void calculate_inner_setpoint(int current_axis, double *inner_setpoint, t_sensor_data data)
{
    double			angle;
	double			angle_error; // rename these variables so it makes sense
    angle = data.accel[current_axis];
	angle_error = outer_setpoint[current_axis] - angle;
	inner_setpoint[current_axis] = angle_Kp * angle_error;
    inner_setpoint[current_axis] = angle_Kp * (outer_setpoint[current_axis]);
}

double calculate_gimbal_angle(double u, double current_axis)
{
    double gimbal_angle;
    if (current_axis == 0)
        gimbal_angle = torque_to_gimbal_angle(u, Ixx);
    else if (current_axis == 1)
        gimbal_angle = torque_to_gimbal_angle(u, Iyy);
    else gimbal_angle = torque_to_gimbal_angle(u, Izz);
    return gimbal_angle;
}

void	run_loop(float delta_time)
{
	t_sensor_data	data;
	FILE			*gyro_file;
	FILE			*accel_file;
	double			inner_setpoint[3] = {0.0, 0.0, 0.0};
	double			rate_measurement;
	double			desired_angular_acceleration;
	double          actuator_command;
    double          gimbal_angle;

	if (delta_time <= 0)
		delta_time = 0.01;
    
    open_files(&gyro_file, &accel_file); // opens the sensor files

	FILE *plotter = fopen("results.csv","w");
	fprintf(plotter,"axis,iteration,gyro_measurement,inner_setpoint,gimbal_axis,desired_angular_acceleration,actuator_command\n");

	while (read_sensor_line(gyro_file, accel_file, &data))
	{	
		printf("Current Iteration: %d\n",count);

		for (int current_axis = 0; current_axis < NUMBER_OF_AXES; current_axis++)
		{

			//outer_setpoint is how fast we want to rotate
            //inter_setpoint is the angular acceleration we need to get there
			calculate_inner_setpoint(current_axis, inner_setpoint, data);


            rate_measurement = apply_filter_gyro(current_axis, data.gyro[current_axis]);
			desired_angular_acceleration = PID(current_axis, inner_setpoint[current_axis], rate_measurement, delta_time);
			if (current_axis == 1){
				rate_measurement = apply_filter_gyro(current_axis,rate_measurement) * 10.0;
			}
            gimbal_angle = calculate_gimbal_angle(desired_angular_acceleration, current_axis);
            actuator_command = normalise(gimbal_angle, MAX_u_value, -1, 1);
			//u = saturation_check(u, MAX_u_value);

			printf("%d,%d,%f,%f,%f,%f,%f\n",
                current_axis,                   // axis
                count,                          // iteration
                rate_measurement,               // gyro_measurement
                inner_setpoint[current_axis],   // rate_command
                gimbal_angle,                   // gimbal_axis
                desired_angular_acceleration,   // calculated angular acceleration
                actuator_command);              // normalized servo / motor command
        
            fprintf(plotter,"%d,%d,%f,%f,%f,%f,%f\n",
                current_axis,
                count,
                rate_measurement,
                inner_setpoint[current_axis],
                gimbal_angle,
                desired_angular_acceleration,
                actuator_command * 10.0);
			differential(rate_measurement,current_axis);

		}
		usleep((unsigned int)(delta_time * 1000000));
		count++;

	}
	fclose(gyro_file);
	fclose(accel_file);
	fclose(plotter);
}

int	main(void)
{
	float	delta_time;

	printf("==============================================\n");
	printf("  Rocket Ascent Vector Control System\n");
	printf("  TVC PD Controller with Real-time Processing\n");
	printf("==============================================\n\n");
	delta_time = 0.1;
	// Run the main control loop
	// run_control_loop(delta_time);

	run_loop(delta_time);
	// apparently rocks use two PID loops called cascaded control
	return (0);
}
