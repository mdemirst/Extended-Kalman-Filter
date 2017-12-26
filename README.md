# Extended Kalman Filter
This repository presents an extended kalman filter (EKF) library and one of its applications as a sample use case. 

# Sample use case: Lidar and radar sensor fusion

First, check out readme from [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

Sensor fusion algorithm based on EKF is used to track a vehicle. It predicts 2D coordinate and velocity of the vehicle from lidar and radar measurements. Results are shown on the animation below. You can check out full video from [here](https://youtu.be/EC-NPt4UdZ0)

![Sensor fusion](https://media.giphy.com/media/xULW8KXaxdFFj28QAU/giphy.gif "Sensor Fusion")

Red and blue circles correspond to lidar and radar measurements, respectively. Green triangles are the predicted coordinates of the vehicle. Root mean square error is below 0.1m for position estimation and 0.5m/s for velocity estimation.

# Quickstart 
## Extended Kalman Filter
You can use extended kalman filter library presented here in your own projects. You'll only need source files under EKF folder:

```bash
EKF/kalman_filter.h
EKF/kalman_filter.cpp
EKF/extended_kalman_filter.h
EKF/extended_kalman_filter.cpp
```

Additionally, you'll need to provide implementations of certain functions which are defined as pure virtual in `extended_kalman_filter.h`. Please feel free to inspect `ekf_lidar.h` or `ekf_radar.h` files to see how to inherit from `ExtendedKalmanFilter` class and implement those necessary functions.

## Sensor fusion
If you want to use EKF for sensor fusion, the most simple and the cleanest way is to initialize seperate filters for each sensor type and keep them synced to each other when any of the filters gets an update. Please feel free to inspect `ekf_fusion.h` to understand fusion pipeline.
