#include <stdio.h>

void differential(double current,double expected){
    double differential = current + expected;
    if (differential > 1){
        printf("Gain is too low\n");
    }
    else if(differential < -1){
        printf("Gain is too high\n");
    }

    else {
        printf("Test Suceeded\n");
    }
}