# CarND-Controls-PID
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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## PID Controller:

A PID (Proportional, Integral, Derivative) controller is a control loop feedback controller which is widely used in different control systems. A PID controller is like adding mass, damper and spring to the physical system (steering system) except it's done in digital way.

Error is an input variable for the controller:

cte = desired_state - measured_state

With the proportional band (P) only, the PID controller output is proportional to the cte. It takes into account only the present value of cte. This is similar to adding a spring to the system.

With derivative (D) part, the controller output is proportional to the rate of change of cte (its derivative). The parameter is used to reduce overshooting and dump oscillations caused by the P. Thus this is similar to adding damper to the physical system.

Integral term (I) takes into account the integral of cte over the past time. It is used to reduce systematic bias. 

PID Tuning is the process of tuning the proportional, derivative, integral co-efficients of the controller. A good PID should reach the setpoint smoothly without a systematic bias.

I have manually tuned the PID for this project. The car can still be tuned further as you will notice some oscillations in the beginning of the video. 

Video Link: [a link] https://youtu.be/rPRmKiuKeK0

However, the car finishes the track without getting off the road. A good PID system should be robust to external disturbance. I would have introduce random noise into the system and test controller's robustness if I had more time.