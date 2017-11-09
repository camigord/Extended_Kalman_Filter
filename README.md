# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program


<img src="./assets/video.gif?raw=true" width="400">

Implementing a Kalman filter to estimate the state of a moving object with noisy LiDAR and radar measurements. The project employs the simulation environment provided by UDACITY which can be downloaded from [here](https://github.com/udacity/self-driving-car-sim/releases).

The video above shows what the simulator looks like when the provided script is used to track the object. The simulator provides the script the measured data (either LiDAR or radar), and the script feeds back the measured estimation marker, and RMSE values from its Kalman filter. LiDAR measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) (required to communicate with the simulator) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. cd build
2. cmake ..
3. make
4. ./ExtendedKF


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
