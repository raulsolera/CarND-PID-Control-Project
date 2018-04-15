# CARND Unscented Kalman Filter Project

[//]: # (Image References)

[image1]: ./report_images/Kp-params.png "Kp-params"
[image2]: ./report_images/Kd-params.png "Kd-params"
[image3]: ./report_images/Ki-params.png "Ki-params"
[image4]: ./report_images/optimal-params.png "Optimal-params"
[image5]: ./report_images/PID-magnitude-comparison.png "PID-magnitude-comparison"

Self-Driving Car Engineer Nanodegree Program

This project consist in the implementation of a PID controller in C++ and testing it with the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).


## Compiling and executing the project

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. Clone the repo and navigate to it on a Terminal.
2. (Within the project repo) mkdir build
3. cd build
4. cmake ..
5. make
6. ./PID [params]

For parameters use:
- Kp
- Ki
- Kd
- use twiddle algorithm: 0 / 1(or other values) --> yes / no
- time from to measure error for twiddle algo
- time range to measure error for twiddle algo

Example with twiddle:
- ./PID 0.125 0.001 1 1 500 1500
Example without twiddle:
- ./PID 0.125 0.001 1 0


## Tunning K params

We started by testing different Kp params (steering proportionl to CTE) and obtained the following results:
![alt text][image1]
None of the configurations, however, got to complete a full lap and the car seemed to behave too responsive.

Then we add the derivative term to the controller trying with different values:
![alt text][image2]
The results inmediately result in a much smoother control (he derivative term smoothens the reactiveness of the proportional term ) and the car got to complete a full lap without problems.

Finally we test different values for the integral term:
![alt text][image3]
The integral term add some extra responsiveness when the cummulative CTE grows this made the driving wilder and it is not clear that it contributes to reduce the cummulative total error as we will see later.

## Reflection on magnitude of the parameters
It can be seen that the K's parameters do not move in the same magnitude, if we explore the terms that these parameters are multiplying: CTE, derivative of CTE and cummulative CTE it stands the reason for this difference in magnitude that is in the order of Kd ~ 10Kp and Kd ~ 1000 Ki:
![alt text][image4]

## Tunning using twiddle algorithm
Finally the twiddle algorithm was implemented to fine tune the Ks parameters. Each epoch of the twiddle algorithm is performed over more than a full lap (cotrolled by time) and each time a parameter is tune the simulator is restarted to measure error over the same path. After a long tuning up process a final optimal terms of xxx were found.
Here this results are compared with the previous manually tune parameters:
![alt text][image5]