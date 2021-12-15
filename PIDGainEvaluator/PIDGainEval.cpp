#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "PCA9685.h"
#include "BNO055.h"
#include "PID.h"

vector eulerAngles;

// Control Loop Properties
const double LOOP_FREQ = 100;                          // Control Loop Frequency
const double LOOP_TIME_S = (1 / LOOP_FREQ);          // Control Loop Execution Time in Sec
const double LOOP_TIME_US = LOOP_TIME_S * 1'000'000; // Control Loop Exec Time in Micro Sec
unsigned long loop_timer;                            // Loop Timer

int main()
{
    loop_timer = micros();

    while (true)
    {

        while (micros() - loop_timer < LOOP_TIME_US)
            ;
        loop_timer = micros();
        std::cout << "loop Timer is : " << loop_timer << std::endl;
    }

    return 0;
}