#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "PCA9685.h"
#include "BNO055.h"
#include "PID.h"

int main(int argc, char **argv) // Main Func Input Seq: Throttle, K_p, K_i, K_d, Tau, Max Allowed Motor Speed
{
    if (argc != 6)
    { // Error to Say That There is not Enough Input Arguments to the cmd!
        std::cerr << "Usage: " << argv[0] << " <name> \n";
        return 1;
    }

    /* PCA9685 Motor Driver Initialization */
    float throttle = std::stof(argv[1]);
    PCA9685 pca = PCA9685();
    pca.setPWMFreq(500);

    const int motor_pins[] = {13, 14, 15, 16}; // Motor Pins
    int motor_speeds[4];
    const int min_motor_speed = 2240;               // Min Available Motor Speed
    const int max_motor_speed = std::stoi(argv[6]); // Max Available Motor Speed

    /* BNO055 IMU Initialization */
    BNO055 imu = BNO055();
    float angle;

    /* PID Control Properties Added */
    double Kp = std::stod(argv[2]);  // p Gain
    double Ki = std::stod(argv[3]);  // i Gain
    double Kd = std::stod(argv[4]);  // d Gain
    double tau = std::stod(argv[5]); // Tau

    PID pid = PID(Kp, Ki, Kd, tau);
    float setpoint = 0;
    float pidVal = 0;

    /* Control Loop Properties */
    const double loop_freq = 100;                        // Control Loop Frequency
    const double loop_time_s = (1 / loop_freq);          // Control Loop Execution Time in Sec
    const double loop_time_us = loop_time_s * 1'000'000; // Control Loop Exec Time in Micro Sec
    unsigned long loop_timer;                            // Loop Timer
    double current_time = 0;                             // Current Time in the Loop According to the Start Time
    loop_timer = micros();                               // Loop Timer Initialization
    const float simulation_time = 7.5;                   // Simulation Time in Seconds

    while (true)
    {
        if (current_time < 0)
        {
            std::cerr << "\nTerminate Due to an Error, Loop Time can not be negative!\n";
            break;
        }
        else if (current_time < 1.5 || current_time > 0)
            setpoint = 0.0f;
        else if (current_time < 3 || current_time > 1.5)
            setpoint = 45.0f;
        else if (current_time < 4.5 || current_time > 3)
            setpoint = 0.0f;
        else if (current_time < 6 || current_time > 4.5)
            setpoint = -45.0f;
        else if (current_time < 7.5 || current_time > 6)
            setpoint = 0.0f;

        angle = imu.read_angle(2);            // Update IMU Angle
        pidVal = pid.update(setpoint, angle); // args ==> (setpoint, measurement)

        /* Motor Speed Mixing Algorithm */
        motor_speeds[0] = throttle + pidVal;
        motor_speeds[1] = throttle + pidVal;
        motor_speeds[2] = throttle - pidVal;
        motor_speeds[3] = throttle - pidVal;

        // Constrain Motor Speeds and Set Speeds to the Motors
        for (int i = 0; i < 4; i++)
        {
            if (motor_speeds[i] < min_motor_speed)
                motor_speeds[i] = min_motor_speed;

            else if (motor_speeds[i] > max_motor_speed)
                motor_speeds[i] = max_motor_speed;

            pca.setPWM(motor_pins[i], motor_speeds[i]);
        }

        std::cout << "loop Timer is : " << loop_timer << " Angle = " << angle << std::endl;
        while (micros() - loop_timer < loop_time_us)
            ;
        loop_timer = micros();
        current_time += (double)loop_time_us / 1000000;
    }

    return 0;
}