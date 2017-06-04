##############################
Extended Kalman Filter Project
##############################


In this project we utilize a kalman filter to estimate the state of a moving object
of interest with noisy lidar and radar measurements.

Dependencies
------------
Ubuntu 16.04 (recommended to make uWebSOcketIO work smoothly)

This project involves the a Simulator which can be downloaded
[here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall
[uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either
Linux or Mac systems. For windows you can use either Docker, VMware, or even
 [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/)
 to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and
ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF


Kalman Filter
--------------

Is an algorithm that uses a series of measurements observed over time, containing
statistical noise and other inaccuracies, and produces estimates of unknown variables
that tend to be more accurate than those based on a single measurement alone, by using Bayesian inference and
estimating a joint probability distribution over the variables for each timeframe.

Extended Kalman Filter
----------------------
This is an extension of Kalman Filter to work with nonlinear systems.
