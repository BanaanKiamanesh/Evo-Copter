#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "PCA9685.h"

#define MIN_PWM 2048
#define MAX_PWM 4095

int main(int argc, char **argv)
{
    if (argc == 2)
    {
        PCA9685 pca;
        pca.setPWMFreq(500);

        int speed = std::stoi(argv[1]);

        for (int i = 1; i <= 16; i++)
        {
            pca.setPWM(i, speed);
            delay(25);
        }

        delay(250);
        std::cout << "\nAll Chennels Set to " << speed << " Which is the Min DutyCycle\n\n";
    }
    else
    {
        std::cerr << "Usage : " << argv[0] << " <speed>\n";
        return 1;
    }
    return 0;
}