#include "../includes/pid.h"

int main(void)
{
    printf("==============================================\n");
    printf("  Rocket Ascent Vector Control System\n");
    printf("  TVC PD Controller with Real-time Processing\n");
    printf("==============================================\n\n");
    
    // Run the main control loop
    run_control_loop();
    
    return (0);
}
