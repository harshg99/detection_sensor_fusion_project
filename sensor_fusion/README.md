# tests

1. Clone "tests" repository
2. Add test_data folder in the "sensor_fusion" package
3. Run command: roslaunch sensor_fusion launch.xml

# Description

FOr this problem, a custom particle filter was written to estimate the robots pose.

The pipeline is described as follows:

1) Initialise particles corresponding to the robot's pose randomnly and assign equal weights
2) Predict new pose of particles based on odometry twist values
3) Update the weights of each particle based on their proximity to latest sensor measurement updates.
   The likelihood is computes using a gaussian posterior imposed over the sensor measurements. The 
   mean of the gaussian posterior is the excact sensor pose measurement. The variance is the 
   covariance in the pose. 
5) Evaluate the weighted sum of the particles to obtain a good estimate of the exact pose
6) If number of samppled effective particles that have comparable weights reduces below a threshold
   the particles are resampled from the existing set of effectinve particles.
7) The process is repeated for the entire duration of the simulation at a frequency of 10 Hz


Issues:

1) One of the primary issues with the particle filter algorithm is the drift. The most likely issue is due to the lack of transformation between the sensor frame and robot base frame.
