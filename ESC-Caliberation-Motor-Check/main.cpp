#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "PCA9685.h"

// declare MAX and MIN Available PWM Length for the ESCs
#define MIN_PWM 2048
#define MAX_PWM 4095

// Motor Pins properties
const int pins[] = {1, 2, 15, 16};

int main()
{
    PCA9685 pca;

    // Check if Motor Driver is Connected
    int dev_status = pca.getFD();
    if (dev_status == -1)
    {
        std::cout << "\n\nDevice Not Found!\n\n";
        return 1;
    }

    pca.setPWMFreq(500); // Set Motor Driver PWM Frequency to 500 Hz

    std::cout << "\n========================================================";
    std::cout << "\nMotor Caliberation Sequence Starting!... \n";

    for (int i = 0; i < 4; i++) // Set Throttle to the Highest
    {
        pca.setPWM(pins[i], MAX_PWM);
        delay(25);
    }
    std::cout << "Throttle Set to the Highest!\n";

    // Ask the User to Connect the Motors
    char motor_check = 0;
    while (true)
    {
        std::cout << "\nConnect the Motors and type Y or y!\n";
        std::cin >> motor_check;
        if (motor_check == 'y' || motor_check == 'Y')
            break;
    }
    delay(3000); // Wait a bit

    // Set to the Min Throttle
    for (int i = 0; i < 4; i++)
    {
        pca.setPWM(pins[i], MIN_PWM);
        delay(25);
    }

    std::cout << "\n\nESCs Caliberated!\n";
    std::cout << "========================================================\n\n";
    delay(1500); // Calibration Sequence Ended

    // Fun Wibes!
    for (int i = 0; i < 16; i++)
    {
        pca.setPWM(pins[i % 4], 4000);
        delay(250);
        pca.setPWM(pins[i % 4], MIN_PWM);
        delay(100);
    }

    return 0;
}