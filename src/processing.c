#include "../includes/pid.h"

void process_attitude(t_attitude_state *state, t_sensor_data *sensor, 
                      t_pid_controller *pid, t_actuator_output *output, int first_loop)
{
    // Update timestamps
    state->timestamp[1] = state->timestamp[0];
    state->timestamp[0] = get_current_time();
    
    // Get time delta for derivative calculation
    long long dt = get_time_difference(state->timestamp[1], state->timestamp[0]);
    if (dt == 0) dt = 1; // Prevent division by zero
    
    // Update magnitude using z-axis acceleration
    update_magnitude(state, sensor->accel[2]);
    
    // Calculate Kp based on magnitude
    double kp = reduction_factor_kp(state->magnitude);
    if (kp == 0.0) kp = 1.0; // Fallback
    
    // For first loop, run twice to initialize differential values
    int iterations = first_loop ? 2 : 1;
    
    for (int iter = 0; iter < iterations; iter++)
    {
        // Read measured values from sensor (angles from gyroscope)
        for (int i = 0; i < 3; i++)
        {
            // Add noise to measured values (simulating sensor noise)
            state->measured[i] = add_noise(sensor->gyro[i], 0.001);
            
            // Calculate error (expected - measured)
            // Expected is target attitude (0 for stabilization), measured is current attitude
            state->error[i] = state->expected[i] - state->measured[i];
        }
        
        if (first_loop && iter == 0)
        {
            // First iteration in first loop - just store errors
            for (int i = 0; i < 3; i++)
                state->last_error[i] = state->error[i];
            continue;
        }
    }
    
    // Calculate derivative of error
    double derivative[3];
    double dt_sec = (dt > 0) ? (dt / 1000.0) : 0.01;  // Convert to seconds with fallback
    
    for (int i = 0; i < 3; i++)
    {
        derivative[i] = (state->error[i] - state->last_error[i]) / dt_sec;
        state->last_error[i] = state->error[i];
    }
    
    // Run PD controller for each axis
    double control_output[3];
    for (int i = 0; i < 3; i++)
    {
        // Use Kp directly without magnitude scaling to avoid instability
        control_output[i] = pd_controller(state->error[i], derivative[i], 
                                         pid[i].Kp, pid[i].Kd);
        
        // Saturate output to prevent extreme values
        if (control_output[i] > 10.0) control_output[i] = 10.0;
        if (control_output[i] < -10.0) control_output[i] = -10.0;
    }
    
    // Map control outputs to actuator commands
    // Gimbal X controls pitch (axis 0), Gimbal Y controls yaw (axis 1), Fin controls roll (axis 2)
    output->gimbal_x = control_output[0];
    output->gimbal_y = control_output[1];
    output->fin_angle = control_output[2];
    
    // Update expected values for next iteration 
    // Expected stays at target (zero for stabilization) - don't use output as setpoint
    // The measured value will change based on system response to control output
}

void write_output(FILE *output_file, t_actuator_output *output)
{
    fprintf(output_file, "%.6f,%.6f,%.6f\n", 
            output->gimbal_x, output->gimbal_y, output->fin_angle);
}

void run_control_loop(int dt)
{
    // Open sensor input files
    FILE *gyro_file = open_sensor_file("gyroscope.txt", "r");
    FILE *accel_file = open_sensor_file("accelerometer.txt", "r");
    FILE *output_file = open_sensor_file("output.txt", "w");
    
    if (!gyro_file || !accel_file || !output_file)
    {
        fprintf(stderr, "Failed to open required files\n");
        return;
    }
    
    // Initialize PID controllers for 3 axes
    t_pid_controller *pid = NULL;
    pid = init_pid(pid, 3);
    
    // Initialize attitude state
    t_attitude_state *state = init_attitude_state();
    
    // Sensor data and output structures
    t_sensor_data sensor;
    t_actuator_output output = {0.0, 0.0, 0.0};
    
    // Seed random number generator for noise
    srand(time(NULL));
    
    int first_loop = 1;
    int iteration = 0;
    
    printf("Starting control loop...\n");
    printf("Iteration | Gimbal X | Gimbal Y | Fin Angle | Magnitude\n");
    printf("----------|----------|----------|-----------|----------\n");
    
    // Main processing loop - read sensor data line by line
    while (1)
    {
        // Process all 3 attitude axes simultaneously
        process_attitude(state, &sensor, pid, &output, first_loop);
        
        // Write output to file
        write_output(output_file, &output);
        
        // Print status
        printf("%9d | %8.4f | %8.4f | %9.4f | %9.4f\n", 
               iteration, output.gimbal_x, output.gimbal_y, 
               output.fin_angle, state->magnitude);
        
        first_loop = 0;
        iteration++;
        
        // Small delay to simulate real-time processing
        sleep(dt);
        //wait_for_sensor_signal();
    }
    
    printf("\nControl loop completed. %d iterations processed.\n", iteration);
    printf("Output written to output.txt\n");
    
    // Cleanup
    fclose(gyro_file);
    fclose(accel_file);
    fclose(output_file);
    free(pid);
    free(state);
}
