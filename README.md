# Project: Evo

### This is a Raspberry pi Based QuadCopter that is stabilized using bio-geography-based Optimization Algorithm.

Hardware Used for the Quadcopters Flight Controller:
1. BNO055 BOSCH IMU
2. PCA9685 PWM driver Module
3. Raspberry pi 4

To Stabilize the Quadcopter PID Controller Gains are tuned using the Bio-Geography based optimization algorithm which is a metaheuristic algorithm inspired by nature.

Quadcopter is made in + configuration. 

For the controller, an anti-windup PID controller is implemented with a low-pass filter cascaded into the derivative gain. Plus the derivation is done on the sensor measurements directly. So the Controller has an extra parameter to be tuned named tau!

The code is written in C++ and Python.

Workflow manual:

0. The first thing to know is quadcopters are inherently unstable and they can go really mad if the controller gains are not tuned so well and we have any RHPZ. So be aware of possible damage and harming probability. So the process of controller gain tuning is done separately on each axis.

1. A PID Gain Evaluator is created where it takes throttle, k_p, k_i, k_d, tau, and max allowed motor speed as input arguments. It tests the performance of the drone with the given gains for 5 seconds and then the flight data of the simulation is stored in a text file.

2. A Python script is created where BBO is implemented. Which estimates the gains and runs the previously made PID gain, evaluator. After the evaluation is done, reads the flights data into a NumPy array and calculates the cost. and the Cost is a combination of peak time, rise time, and the RMS of the data.

3. The results are shown at the end of the "Optimization.ipynb" file.


Testing Consideration Points:

0. First thing to do if you want to try this is to bound the drone to the stand well, where only one axis is free. Make sure the free axis is also constrained and cant travel 360.

1. In case you didn't notice, if the flight data contains large values(100000), it means the overshoot was more than 40 degrees therefore the simulation was not complete and stopped in the middle.

2. Wear safety goggles if you are anywhere near the drone when it's up and running.

3. After the Simulation is done for 3 axes, 12 controller gains are ready to put into the flight controller!

#### Enjoy flying your baby :)