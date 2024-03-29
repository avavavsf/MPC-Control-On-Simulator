## MPC Control on Simulated Vehicles

The final video of the simulated vehicles controled by my MPC controler is here below.
* Resulution: 640X480; Graphics Quality: Fasttest )

[![IMAGE ALT TEXT](http://img.youtube.com/vi/MRL76I7RATI/0.jpg)](https://youtu.be/MRL76I7RATI "MPC control ")

### The Model
The MPC allows the car to follow the trajectory along a line by using predicted(calculated) actuators like steering angle and acceleration.

In our model,the states vector are:
```
states = [x,y,psi,v]
```
where `(x,y)` is position of the vehicle; `psi` is orientation of the vehicle; `v` is velocity.
```
Actuators: [delta,a]
```
where `delta` is steering angle and `a` is acceleration (throttle/brake combined).

The vehicle model is implemented is a kinematic bicycle model that ignore tire forces, gravity, and mass.  Kinematic model is implemented using the following equations:
```
      x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      v_[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```   
where  `Lf` measures the distance between the front of the vehicle and its center of gravity (the larger the vehicle, the slower the turn rate); `cte` is  cross-track error (the difference between the line and the current vehicle position y in the coordinate space of the vehicle); `epsi` is the orientation error. 

### Choice Timestep Length and Elapsed Duration (N & dt)
The hyperparameter `N` is the number of timestamps in future, and `dt` is time elapse in seconds between two actuations. 
First I started with the value of N = 25, and dt = 0.05, which is the values in the mpc quiz. However, it did not workout as expected. I increased N to 60 to get a longer smoother line. At slow speed it gave good results, so I set the speed limit at 40. However as I started increasing speed, N=60 at high speeds started having problem fitting a smooth degree-3 polynomial especially around curves. After fail and trial many times. I stettled at a reference speed of 65, `N=7`, and `dt=0.07`. 

### Model Predictive Control with Latency
There's a 100 millisecond latency between actuations commands. So in order to account for it, I take the current state from telemetary data and project the current state in car-coordinates to a future timestamp using the kinematic model as expalined above in Model section above.
The equations for new state with latency are:
```
double lat_x = 0.0+v*0.1;
double lat_y = 0.0;
double lat_psi = 0.0;
double lat_cte = polyeval(coeffs, 0) - 0;
double lat_epsi = lat_psi -atan(coeffs[1] + 2 * coeffs[2] * lat_x + 3 * coeffs[3] * lat_x * lat_x);
```


### Dependencies

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


### Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

