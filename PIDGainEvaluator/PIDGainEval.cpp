#include <iostream>
#include <fstream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cmath>
#include "PCA9685.h"
#include "BNO055.h"
#include "PID.h"

int main(int argc, char **argv) // Main Func Input Seq: Throttle, K_p, K_i, K_d, Tau, Max Allowed Motor Speed
{
    /*
     Initial Harware Setups
    */
    if (argc != 7)
    { // Error to Say That There is not Enough Input Arguments to the cmd!
        std::cerr << "Usage: " << argv[0] << " <Throttle> <K_p> <K_i> <K_d> <Tau> <Max_Speed> \n";
        return 1;
    }

    /* PCA9685 Motor Driver Initialization */
    int throttle = std::stoi(argv[1]);
    PCA9685 pca;
    pca.setPWMFreq(500);

    const int motor_pins[] = {14, 16}; // Motor Pins
    int motor_speeds[2];
    const int min_motor_speed = 2240;               // Min Available Motor Speed
    const int max_motor_speed = std::stoi(argv[6]); // Max Available Motor Speed

    /* BNO055 IMU Initialization */
    BNO055 imu;
    float angle = 0;

    /* Control Loop Properties */
    const double loop_freq = 100;                        // Control Loop Frequency
    const double loop_time_s = (1 / loop_freq);          // Control Loop Execution Time in Sec
    const double loop_time_us = loop_time_s * 1'000'000; // Control Loop Exec Time in Micro Sec
    unsigned long loop_timer;                            // Loop Timer
    double current_time = 0;                             // Current Time in the Loop According to the Start Time
    loop_timer = micros();                               // Loop Timer Initialization
    const float simulation_time = 5;                     // Simulation Time in Seconds

    /* PID Control Properties Added */
    float Kp = std::stof(argv[2]);  // p Gain
    float Ki = std::stof(argv[3]);  // i Gain
    float Kd = std::stof(argv[4]);  // d Gain
    float tau = std::stof(argv[5]); // Tau

    PID pid;
    pid.init(Kp, Ki, Kd, tau, loop_time_s);
    float setpoint = 0;                       // PID Setpoint
    float pidVal = 0;                         // PID Correction Val
    const int pid_val_min = -800;             // Min PID Correction Val
    const int pid_val_max = 800;              // Max PID Correction Val
    pid.set_bounds(pid_val_min, pid_val_max); // PID Correction Value Bounds

    /* Error Data Logs Save */
    const int max_overshoot = 40;                     // Max Allowed Overshoot in Degrees
    int loop_counter = 0;                             // Indicates How Many Time the Loop Iterated!
    const int arr_size = simulation_time * loop_freq; // Array Size to Hold the Error Data
    float err_data[arr_size];                         // Array to Hold Error Data
    std::ofstream data("pid_gain_err.txt");           // Create File Obj

    for (int k = 0; k < arr_size; k++) // Fill The Array with a Large Number
        err_data[k] = 100000;

    /*
     Main Control Loop
    */
    while (true)
    {
        if (current_time < 5)
        {
            setpoint = 0.0f;

            if (current_time > 1.5) // Dont Let the Error Grow too Much!
                if (abs(setpoint - abs(angle)) >= max_overshoot)
                    break;
        }
        else
            break;

        angle = imu.read_angle(2);            // Update IMU Angle
        pidVal = pid.update(setpoint, angle); // args ==> (setpoint, measurement)
        err_data[loop_counter] = angle;       // Save Error

        /* ONE Axis Motor Speed Mixing Algorithm */
        motor_speeds[0] = (int)throttle - pidVal;
        motor_speeds[1] = (int)throttle + pidVal;

        // Constrain Motor Speeds and Set Speeds to the Motors
        for (int i = 0; i < 2; i++)
        {
            if (motor_speeds[i] < min_motor_speed)
                motor_speeds[i] = min_motor_speed;

            else if (motor_speeds[i] > max_motor_speed)
                motor_speeds[i] = max_motor_speed;

            pca.setPWM(motor_pins[i], motor_speeds[i]);
        }

        // Log Status to the Console
        std::cout << "Angle = " << angle << "\tPID = " << pidVal << "\terror = " << pid.get_err() << "\tMotor Pair 1 = " << motor_speeds[0] << "\tMotor Pair 2 = " << motor_speeds[1] << std::endl;

        while (micros() - loop_timer < loop_time_us) // Add Enough Delay to make the Contol Loop Exec in the Specified Time
            ;

        loop_timer = micros();                          // Record Starting Time of the Next Iteration
        current_time += (double)loop_time_us / 1000000; // What is the Current Time from the Beginning of the Execution
        loop_counter++;                                 // Count the Iterations
    }

    delay(100); // Just a Small Delay

    for (int i = 0; i < 2; i++) // Kill the Motors
        pca.setPWM(motor_pins[i], 2048);

    // Save Error Data to the File
    if (data.is_open())
    {
        for (int ii = 0; ii < arr_size; ii++)
            data << err_data[ii] << "\n";

        data.close();
    }
    else // throw Error if File Was not Available
        std::cerr << "\n\nError : Unable to open file!\n\n";

    return 0;
} // Hopefully a Happy Ending