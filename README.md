# **Localization**

[//]: # (Image References)

[particle_filter_implementation]: ./image/particle_filter_implementation.png "Particle Implementation"
[particle_count_study]: ./image/particle_count_study.png "Particle Count Study"
[screenshot]: ./image/screenshot.png "Screenshot"

## Particle Filter Project

The robot has been kidnapped and transported to a new location!
Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project, a 2-D particle filter in C++ is implemented to localize the robot.
The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide).
At each time step, the filter will also get observation and control data.

![screenshot]

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

## File Description

|Files|Description|
|:----|:----------|
|`src/main.cpp`|1. Communicates with the Term 2 Simulator receiving data measurements. <br /> 2. Call functions to run the particle filter.|
|`src/FusionKF.h` <br /> `src/FusionKF.cpp`|Generic class of FusionEKF and FusionUKF|
|`src/particle_filter.h` <br /> `src/particle_filter.cpp`|Implementation of the particle filter.|
|`src/helper_functions.h`|Some helper functions.|
|`src/map.h`|Class for map data.|
|`src/visualization.ipynb`|Some experimental results on the number of the particles.|

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Or just execute the script `./build.sh`.

## Basic Run Instructions

|Command|Description|
|:------|:----------|
|`./particle_filter`|Run the program directly.|
|`./run.sh`|Run the program using the script.|

## Implementation of the Particle Filter

The particle filter is implemented in `particle_filter.cpp`.
The detail is illustrated in the following figure.
![particle_filter_implementation]


## Discussion of the number of the particles

A experiement was performed to study the effect of the error by the number of the particles.
The following chart shows the x, y, yaw error against the number of the particles.
The particle filter starts to converge when there is only 4 particles.
The error only improves slightly after 100 particles.
Therefore, 100 particles were used in the project.

![particle_count_study]
