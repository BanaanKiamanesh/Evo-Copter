#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "PCA9685.h"

#define MIN_PWM 2048
#define MAX_PWM 4095

int main()
{
    PCA9685 pca;
    pca.setPWMFreq(500);

    for (int i = 1; i <= 16; i++)
    {
        pca.setPWM(i, MIN_PWM);
        delay(25);
    }

    delay(250);
    std::cout << "\nAll Chennels Set to " << MIN_PWM << " Which is the Min DutyCycle\n\n";

    return 0;
}