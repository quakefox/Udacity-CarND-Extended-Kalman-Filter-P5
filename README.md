# Extended Kalman Filter Project Starter Code

This project utilizes a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

This project involves the Term 2 Simulator offered by Udacity which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.



[//]: # (Image References)

[image1]: ./output/lidar_only.png "EKF with Lidar Only"
[image2]: ./output/radar_only.png "EKF with Radar Only"
[image3]: ./output/lidar_radar_combined.png "EKF with Lidar and Radar"





Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"] <= root mean square error of predicted postion x from groundtruth x

["rmse_y"] <= root mean square error of predicted postion y from groundtruth y

["rmse_vx"] <= root mean square error of predicted velocity x from groundtruth velocity x

["rmse_vy"] <= root mean square error of predicted velocity y from groundtruth velocity y

---

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
4. Run it: `./ExtendedKF `


## Implementation and Performance 

The kalman filter performs prediction and uses inputs from both Radar and Lidar sensors to perform updates. In the update step, an extended kalman filter is used to process Radar data input. In the error formular y = z - z_pred, the angle of y[1] has been normalized between value interval [-Pi, +Pi] to avoid abrupt angle change. 

With only Lidar input, 

![alt text][image1]

On dataset 1, the RMSE results were:
X: 0.1838
Y: 0.1542
VX: 0.5748
VY: 0.4893

On dataset 2, the RMSE results were:
X: 0.1644
Y: 0.1557
VX: 0.6356
VY: 0.5746



With only Radar input, 

![alt text][image2]

On dataset 1, the RMSE results were:
X: 0.2353
Y: 0.3354
VX: 0.5244
VY: 0.7104

On dataset 2, the RMSE results were:
X: 0.2391
Y: 0.3373
VX: 0.6422
VY: 0.7739

Clearly, the Radar data is noiser than Lidar, yielding in higher RMSEs.



With both Lidar and Radar inputs, 

![alt text][image3]

On dataset 1, the RMSE results were:
X: 0.0964
Y: 0.0853
VX: 0.4154
VY: 0.4316

On dataset 2, the RMSE results were:
X: 0.0727
Y: 0.0968
VX: 0.4893
VY: 0.5078

Clearly, when using both sensor inputs, we are achieving better RMSE than that from Lidar or Radar sensor alone. 


## Generating Additional Data

To generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.