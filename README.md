# Rack detection and sensor fusion project

## Introduction

Preliminary project to get my hands dirty with sensor fusion and computer vision. The project empoys a mobile robot that detects racks via segmentation and uses odometry and imu data to localize with a particle filter.



## Usage
To run rack_detection

1. Clone "tests" repository
2. Add .bag file in the "rack_detection" package
3. Run command: roslaunch rack_detection launch.xml
 
To run sensor fusion

1. Clone "tests" repository
2. Add test_data folder in the "sensor_fusion" package
3. Run command: roslaunch sensor_fusion launch.xml
