# LiDAR, Lane Detection
Lane Detection | LiDAR Intensity

## Introduction
Due to the lack of data, it is very difficult for the camera to recognize lanes where lanes are not drawn well. So I built an algorithm to detect lanes using the strength of LiDAR. Also, it regress lanes through cubic_spline in Ubuntu, and the results can be seen through rviz.


CMakeLists.txt is not uploaded separately because users can fill it out as needed.

## Prerequisites
It runs on Ubuntu 20.04, and the version below doesn't matter if you install the latest version.

- numpy
- open3d
- pcl

## Topics
There are a lot of topics in LIDAR, but we get the information of the lane. These are the values where the point cloud is transacred, which only takes the information of the road. You also need to type 'lane' after lidar_cpp when you run the code.


## Inference
Before you start the code, please check the path carefully when importing cubic_spline_planner.py, and make sure you have pcl and serial_node.cpp!

If it doesn't run well, please look at the profile and contact us via "landsky1234@naver.com" email!
```Shell
roscore
rosrun [catkin_ws/'your_directory'] lidar_cpp lane
rosrun [catkin_ws/'your_directory'] lane_detection.py
```


## Result
- [https://youtu.be/dyvAe9ug7Qk](https://www.youtube.com/watch?v=j-CIY5nhay4)
