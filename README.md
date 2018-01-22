# Udacity Self-Driving Car Engineer Nanodegree Program
# *Model Predictive Controller Project*

## Intro

This repository provides a solution to the Model Predictive Control (MPC) project of UDACITY Self Driving Car Nanodegree program. The goal of the project is to control drive of a car around a track in the [simulator](https://github.com/udacity/self-driving-car-sim/releases) using MPC strategy. The drive has to be robust with no wavy or abnormal movements. The drive has to be acceptable in real world. Hence keeping the car in the center of the track is critical.

As instructed by the UDACITY, IPOPT and ADCpp libraries need to be used to achieve the project's goal. These libraries provide optimized control actuation values in order to minimize a defined error. The error is result of difference between the car projected position and a fitted polynomial to the desired driving position (waypoints). The optimized values are calculated for a limited number of future time steps to avoid extra calculations. The optimizer communicate only the first set of actuations with the simulator and predicts vehicle states in the future time steps again. This allows the optimizer to actively compute the best actuation values by predicting near future states of the car.


![equations](https://github.com/ArmanKh9/P5-MPC/blob/master/pics/predict.png)


## Rubric Points

- **The Model**: *Student describes their model in detail. This includes the state, actuators and update equations.*

In this project a very simplified model of a vehicle motion is used. The model only takes kinematics of the drive into consideration. However, in reality dynamics of the vehicle plays a critically important role in the vehicle motion that cannot be ignored. In the kinematic model shown below, state of the vehicle including its x and position, heading direction (psi), velocity (v), cte, heading direction error (epsi) are calculated for each time step based on information from the previous timestep.

![equations](https://github.com/ArmanKh9/P5-MPC/blob/master/pics/eqns.png)

- **Timestep Length and Elapsed Duration (N & dt)**: *Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.*

In this solution, N=10 timsteps with duration of dt=0.1 for each step were chosen. A small change in dt can sometime cause erratic movements in the track. Small values of dt such as 0.05 were chosen and it resulted in toomany actuation and erratic movement. Larget dt up to 0.4 were also examined which caused distorted polynomial fits and eventually a wavy drive with vehicle getting off the track.
N=10 is computationally efficient while predicting a long enough future time. Timestep numbers up to 20 were chosen and no significant effect was seen on the drive. However, larger timestep number caused hardware overload and larger latency in communications between the simulator and optimizer.


- **Polynomial Fitting and MPC Preprocessing**: *A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.*

The waypoints are provided by the simulator in map coordinates. These waypoints were transformed and translated to the vehicle's coordinate system. The vehicle coordinate system assumes the vehicle to be at (0,0) with psi increasing counter-clockwise. This coordinate system change facilitates the polynomial fitting and evaluation process. Also vehicle velocity was converted from m/hr to m/s for a better understanding and consistency in units. However, the velocity was fed to the solver in m/hr unit.

```cpp
//transforming waypoints into vehicle coordinate system negative sign of psi is due to the vehicle model in the simulator
for (int k=0; k<n; k++){
  double x_prime = ptsx[k] - px;
  double y_prime = ptsy[k] - py;
  way_x.push_back(x_prime*cos(-psi) - y_prime*sin(-psi));
  way_y.push_back(x_prime*sin(-psi) + y_prime*cos(-psi));
}
```


- **Model Predictive Control with Latency**: *The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.*

The latency causes the vehicle to move for another 100 millisecond before the new actuation values affect the vehicle in the simulator. The latency is taken into account by predicting the vehicle state in 100 millisecond from the current state. This is done by using the kinematic model. Then the predicted state is fed into the solver instead of the current state. This approach required the velocity to be converted to m/s.

```cpp
double lat = 0.100; // 100 milliseconds

double px_lat = v_ms * lat * cos(psi);
double py_lat = 0.0;
double psi_lat = (v_ms / 2.67) * (-1 * delta) * lat;
double v_lat = v_ms + accel * lat;
double cte_lat = cte + v_ms * sin(epsi) * lat;
double epsi_lat = epsi + (v_ms / 2.67) * (-1 * delta) * lat;
double v_lat_mph = v_lat/0.44704;

// creating state vector by feeding predicted state in 100 milliseconds
Eigen::VectorXd state(6);
state<<px_lat, py_lat, psi_lat, v_lat_mph, cte_lat, epsi_lat;
}
```
## Tuning

As described in the lessons, the cost function in the MPC model needs to tuned to optimize behavior of the solver and its actuation values. It was noted that multiplying the steering value change between timesteps by correction factor of 1000 improve the drive and make steering much smoother. Also, limiting the acceleration cost between timesteps seems to make the drive smoother. A correction factor of 100 was chosen for acceleration between timesteps.

## Reference Velocity

The tuning process was done for the velocity of 30 mile per hour. The correction factor values may need to change for other velocity values. Also other correction factors needs to be used for higher velocities. However, as the car smoothly drives around the track with the selected velocity and correction factors, no further experimentation was done.

---
# *Udacity's original README content*

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do y
