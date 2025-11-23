#include "../includes/pid.h"
#include "math.h"
// ax ay az (linear accel) Ax Ay Az(angular accel)  assuming all 6 are ready to use
double g = 9.81; //m^2
double mass = 64.4; //kg
double dry_mass = 31.8; //kg
double thrust = 7890; //N
double burn_time = 5.15; //s
double rocket_lenght = 169; //cm
double rocket_diameter = 20; //cm
double max_velocity = 628; //m/s
double max_acceleration = 136; //m/s^2

double delta_time(double final_time, double initial_time){
    double delta_time = final_time - initial_time;
    return delta_time;
}

double delta_acceleration(double final_velocity, double initial_velocity){
    double delta_acceleration = final_velocity - initial_velocity;
    return delta_acceleration;
}
double delta_angular_position(double final_angular_position, double initial_angular_position){
    double delta_angular_position = final_angular_position - initial_angular_position;
    return delta_angular_position;
}

double delta_surface_altitude(double final_surface_altitude, double initital_surface_altitude){
    double delta_surface_altitude = final_surface_altitude - initital_surface_altitude;
    return delta_surface_altitude;
}

double angular_velocity(delta_angular_position, delta_time){
    double angular_velocity = delta_angular_position / delta_time;
    return angular_velocity;
}



double delta_angular_velocity(double final_angular_velocity, double initial_angular_velocity){
    double delta_angular_velocity = final_angular_velocity - initial_angular_velocity;
    return delta_angular_velocity; 
}
double angular_acceleration(delta_angular_velocity, delta_time){
    double angular_acceleration = delta_angular_velocity / delta_time;
}

double magnitude_of_change(double linear_accelration){};

double thrust_applied_from_motor(){};

double center_of_mass_calculation(){

}

double moment_of_inertia(){};
double torque(angular_acceleration, moment_of_inertia){

    double torque = angular_acceleration * moment_of_inertia;
    return torque;

};
double acceleration(moment_of_inertia,torque){};
double net_force(double mass , double acceleration){
    return mass*acceleration;
}

double drag(double fluid_density, double velocity){};//incomplete

double thrust_to_weight_ratio ( mass,gravity, thrust){
    double twr = thrust/(mass*gravity);
    return twr;
}

double horizontal_drift (double velocity_axis_x, double velocity_axis_y){
    double h_drift = sqrt(floor(velocity_axis_x *2) + floor(velocity_axis_y*2));
    return h_drift;
}