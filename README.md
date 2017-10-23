# Unscented Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project reburic. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

## Basic Build Instructions
I was using Windows 10 and VisualStudio17

-to build this project using **Bash for window** :

    navigate to projet
    write cmd : mkdir build
    then navigate to build
    write cmd : cmake .. -G "Unix Makefiles" && make


## UKF Steps

1. init Matrcies & vectors & wieghts >> in UKF constructor
2. recive the initializtion readings >> in function ProcessMeasurement line:83 
3. Prediction function >> line:129
   * first augmnet the sigma points >> in function AugmentSegmaPoints line:165
   * then predict sigma points >> in function PredictSigmaPoints line:198
   * finaly Predicted Mean and Covariance >> line:138
4. now we swich betwwen laser and radar to update position >>line:115
   * lidar fucntion is linear , so we can use code from EKF >>function UpdateLidar line:249
   * but radar fucntion is not linear, we use UKF >> fucnion UpdateRadar line:288
5. finally we calculate NIS for Lidar and Radar >>line:281 & line:372
6. tune noise :
   * I started tuning "a" noise from 30 trying less values, I was convinced that we can't get better results than .75
   * then I tuned "yawd" noise, started from .75 and going up, I found 1.5 is the best
   * I made sure those values are then best by plotting NIS values, and print out how much values are above 7.8(red line) and less than .352 (green line) for Radar ,
and above 5.9 (red line) & less than 0.103 (green line) for Lidar


plot for a=.75 & yawdd=1.5 :

*Lidar*

`NIS_laser : _5p9:0.0240481, _p103:0.0440882`

![Lidar](https://github.com/anasmatic/CarND-Term2-Project2_Unscented-Kalman-Filter/blob/master/PlotNIS/laser_a0.75yawd1.5.png)

------

*Radar*

`NIS_rader : _7p8:0.012024, _p352:0.156313`

![Radar](https://github.com/anasmatic/CarND-Term2-Project2_Unscented-Kalman-Filter/blob/master/PlotNIS/radar_a0.75yawd1.5.png)



