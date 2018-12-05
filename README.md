# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

[//]: # (Image References)

[image1]: ./images/Jerky_MPC.png "Jerky MPC"
[image2]: ./images/Smoother_MPC.png "Smoother MPC"

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
* For matplotlib in C++ the following needs to be done
  * In `matplotlibcpp.h` the python include is present as
  ```
  #include <python2.7/Python.h> 
  ```
  To make sure the include works correctly update the target_link_libraries to include python2.7 in `CMakeLists.txt` as well
  * sudo apt-get install pypy-dev
  * sudo apt-get install python-dev
  * sudo apt-get install libfreetype6-dev pkg-config libpng12-dev
  * pip install matplotlib==2.2.3 (For python 2.7)
  * After this the namespace plt should refere to the matplotlibcpp when plt functions needs to be used
  ```
  namespace plt = matplotlibcpp;
  ```


## Basic Build Instructions

* Clone this repo.
* Make a build directory: `mkdir build && cd build`
* Compile: `cmake .. && make`
* Run it: `./mpc`
* If matplotlibcpp needs to be build as well use: 
  * `set(sources src/matplotlibcpp.h)`
  * `target_link_libraries(python2.7)` 

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.


## Project Instructions and Rubric

If enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a) for instructions and the project rubric.

### Introduction

The purpose of this project was to implement a model predictive controller (MPC) to drive a vehicle in a reference trajectory in a simulated environment.

MPC uses an optimizer to find the control inputs that minimizes the cross-track error (cte).

* Initialization
  * Define the trajectory length, N, and time duration of each steps, dt
  * Initialize the vehicle kinematic model and the actuator models
  * Define a cost function to minimize the cte

* Optimization solver
  * Pass the current state into the kinematic model of the MPC
  * Estimate the cost function minimizer using Inter point optimizer (Ipopt). In the process we get the control signals as output
  * Apply the control signals for the next step

### Vehicle model

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
       = cte[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

```

* `x, y` : Car's position.
* `psi` : Car's heading direction.
* `v` : Car's velocity.
* `cte` : Cross-track error.
* `epsi` : Orientation error.
* `a` : Car's acceleration (throttle).
* `delta` : Steering angle.

### Cost function and penalty weights

Given the constraints of our model here are the objectives which helped me to determine the penalty weights:

- Minimize cte
- Minimize our heading error `epsi`
- Go as fast as possible. However, I set a limit of 50
- Consecutive steering angles to naot vary which results in oscillations in the controller
- Consecutive accelerations needed to be smoothed out for optimal fuel consumption in real vehicles

So mathematically it should be like:

```
cost = A * cte^2 + B * epsi^2 + C * (v - vmax)^2 +
       D * delta^2 + E * a^2 + F * (a` - a)^2 +  G * (delta` - delta)^2

... integrated over all time steps
```

I experimentally came to the conclusion of the following values for the penalty weights:
```
const int cte_weight = 1000; // A
const int epsi_weight = 1000; // B
const int v_weight = 1; // C
const int delta_weight = 50; // D
const int a_weight = 50; // E
const int delta_dt_weight = 25000; // F
const int a_dt_weight = 5000; // G

/* Cost based on the reference state - cte, orientation, velocity */
for (size_t i = 0; i < N; i++) {
   fg[0] += cte_weight * CppAD::pow(vars[cte_start + i], 2);
   fg[0] += epsi_weight * CppAD::pow(vars[epsi_start + i], 2);
   fg[0] += v_weight * CppAD::pow(vars[v_start + i] - ref_v, 2);
}

/* Costs for steering (delta) and acceleration (a) */
for (size_t i = 0; i < N-1; i++) {
   fg[0] += delta_weight * CppAD::pow(vars[delta_start + i], 2);
   fg[0] += a_weight * CppAD::pow(vars[a_start + i], 2);
}

/* Costs related to the change in steering and acceleration (makes the ride smoother) */
for (size_t i = 0; i < N-2; i++) {
   fg[0] += delta_dt_weight * pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
   fg[0] += a_dt_weight * pow(vars[a_start + i + 1] - vars[a_start + i], 2);
}
```

### Timestep Length and Elapsed Duration

* N: This represents how many states we look into future. Experimentall, I chose N = 10.
* dt: This represents in how much time we expect environmental changes. I chose dt = 0.1 as this the latenacy between the controller output and the actuation in the simulated system.

If N is too small, we cannot predict the future well. If N is too large then we may plan for a long future which not be what we are expecting. 


### Polynomial Fitting and MPC Preprocessing

* The waypoints to estimate the road curves were given in global coordinate system, which were transformed to vehicle's local coordinate system.
* I used a 3rd order polynomial because it fits most of the road. Using a smaller order can result in underfitting while using a higher order can result in overfitting of the curves.

```
size_t n_waypoints = ptsx.size();
auto ptsx_transformed = Eigen::VectorXd(n_waypoints);
auto ptsy_transformed = Eigen::VectorXd(n_waypoints);
for (unsigned int i = 0; i < n_waypoints; i++ ) {
  double dX = ptsx[i] - px;
  double dY = ptsy[i] - py;
  double minus_psi = 0.0 - psi;
  ptsx_transformed( i ) = dX * cos( minus_psi ) - dY * sin( minus_psi );
  ptsy_transformed( i ) = dX * sin( minus_psi ) + dY * cos( minus_psi );
}

// Fit polynomial to the points - 3rd order.
auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);
```

### Model Predictive Control with Latency   

Note we have to take the 100ms latency into account, to compute the kinematic model and do the actuation before feeding it into the predictor. Also, I added a smoothing strategy to make the controller more oscillations agnostic. I calculated the average of the predicted steering angles and accelerations based on the cte over a third of the prediction steps. This helps in minimizing the abrupt decision changes in the controller. Rather the controller takes decision based on a little more estimated horizon.

```
  size_t M = std::ceil(N/3);
  double delta_sum = 0.0;
  double a_sum = 0.0;
  for (size_t i = 0; i < M; ++i) {
    delta_sum += solution.x[delta_start + i];
    a_sum += solution.x[a_start + i];
  }
  optimizedSolver.push_back(delta_sum/M);
  optimizedSolver.push_back(a_sum/M);
  optimizedSolver.push_back(solution.x[cte_start]);
  optimizedSolver.push_back(solution.x[psi_start]);
  for (size_t i = 0; i < N; ++i) {
    optimizedSolver.push_back(solution.x[x_start + i]);
    optimizedSolver.push_back(solution.x[y_start + i]);
  }
```

Before applying this averaging, I got a jerky controller:

![Jerky MPC controller with oscillations][image1]

However, after applying the averaging, I achieved a fairly smoother controller:

![Smoother MPC controller with relatively oscillations][image2]      

In the later case, the ride will be much more pleasant than the former.

## Call for IDE Profiles Pull Requests

* Since, I have used codelite for ease of working I am sharing those profiles here:
  * Udacity_MPC-P5.project
  * Udacity_MPC-P5.workspace
