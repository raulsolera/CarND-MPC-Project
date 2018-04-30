# CarND-Controls-MPC
Udacity Self-Driving Car Engineer Nanodegree Program - Term 2, Project 5

---

## Introduction
This project consist in the implementation of an MPC controller in C++ such that a car can drive around a virtual track using the Term 2 Simulator. The simulated car's actuators have a 100ms latency (delay) that must be taken into account.

A kinematic model will be used being the state represented by 4 variables: x and y position (with respect to car initial coordinates) angle psi and velocity v /velocity.
The model will predict values for 2 actuators delta (change in angle) and a (acceleration).
The error will be measure by the CTE (cross track error or deviation from the optimal trajectory) and epsi (deviation from the direction of the optimal trajectory).
The transition equations of the model are given by:
- x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
- y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
- psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
- v[t+1] = v[t] + a[t] * dt
- cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
- epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

### Project pipeline
The MPC model pipeline consists in the following steps:
- Obtaining the optimal trajectory (in this project it will be given by the simulator: vector ptsx and ptsy in main.cpp) and approximate it by a 3rd degree polynomial.
- Obtaining the actual state of the car and actuators.
- Consider latency to predict state and errors (cte and epsi) in the moment that the actuators predicted will be effective.
- Pass predicted state and coefficients of the optimal trajectory to the optimizer (ipopt will be used)
- Apply optimal actuators returned by the optimizer to the car and start again the pipeline loop.

### Parameter settings and tuning up
There are 2 groups of parameters to tune the model:
- In one hand the number of steps N and the time of each step dt must be specify.
- On the other hand the cost function has to be tune up to obtain optimal results. This implies considering terms other than CTE and epsi. In example: difference with a reference velocity, magnitude of the actuators or magnitude of the change in the actuators. For this project all the previous terms will be considered applying a certain weight to each one and the tuning up will be done changing the weights.

## Results and reflection
After some tuning up the model is able to drive the car in a smooth way reaching velocities over 90 mph, by far much better results than that of the PID controller.
- Values of N = 12 and dt = 0.1 where used. Lower values of N implies lower accuracy and the car sometimes gets out of the road. Higher values suppose a too high computational cost not at reach for any computer (not in fact for my 10 years old laptop). Timesteps below the latency time suppose a waste of computational resources and over it will produce inaccurate results.
- In the case of the cost, higher weights of 3000 and 1000 are assigned to CTE and epsi and lower ones (in the order of 2 or 3 orders of magnitude) to the rest with the exception of the change in delta actuator that greatly improves the smoothness of the driving.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.