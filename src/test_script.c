#include <stdio.h>
#include <../includes/pid.h>

void differential(double current, double expected){
    double differential = fabs(current) - fabs(expected);
    if (differential > 1.0){
        printf("Gain is too low\n");
    }
    else if(differential < -1.0){
        printf("Gain is too high\n");
    }

    else {
        printf("Test Suceeded\n");
    }
}