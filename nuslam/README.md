# NUSLAM
Author: Katie Hughes

### Launch slam with known data association:
`ros2 launch nuslam nuslam.launch.xml`

This will show the red, blue, and green turtles in an environment with cylindrical obstacles. The red robot corresponds to the `nusim` simulated robot, and the red cylinders are the true obstacle locations. The blue robot corresponds to the `turtle_control` odometry robot. Finally, the green robot corresponds to `slam`'s EKF slam state estimate, and the green cylinders are the obstacle locations according to the EKF results. The yellow cylinders correspond to the `fake_sensor` topic which simulates the relative location of the obstacles with noise added. This topic provides the known data association for the EKF algorithm.

### Launch SLAM with unknown data association (using lidar data):
`ros2 launch nuslam landmark_detect.launch.xml`

This will read from the lidar data instead of the `fake_sensor` topic and perform circle detection and data association to recognize obstacles. This will show everything in the previous launchfile, with the addition of some teal obstacles that correspond to the result of the circle detection algorithm.

### Notes

Publishing to `cmd_vel` topic will move the red robot through the environment, which will update the other two robots accordingly. The green SLAM robot should stay close to the red robot, which will become even more apparent if you drive into the obstacles in the environment. 

If you want to change the number of obstacles that the slam algorithm can detect, you need to modify the `max_obstacles` parameter in the `slam.yaml` config file. The current configuration can detect up to 3 obstacles.

Additionally, if you want to adjust the noise levels (or turn noise off completely), this can be done in the `noise.yaml` file in the `nusim` package.

### Results

Below is an image of the result of my SLAM algorithm with known data association.

![Slam Path](images/LabeledSlamPath.png?raw=true "Slam Path")

Finally, below is a video of my SLAM algorithm working with simulated lidar data (with noise) and unknown data association. The initial estimate of the SLAM position is not great, but after taking in more measurements, you can see the green robot "snap" back to the location of the red. By the end of the run, the green and red robots are essentially on top of each other, while the odometry estimate is far away from the true location.

[BESTSLAM.webm](https://user-images.githubusercontent.com/53623710/224413751-8f278ed7-de68-4f1f-9841-a1173f067ec1.webm)


At the end of this run, the final position of the red `nusim` (real) robot was (-0.022714, 0.028604) m.  The final position of the green SLAM robot was (-0.018955, 0.030844) m. The final position of the blue odometry robot was (0.0091331, -0.34456) m. The offsets of the SLAM and odometry estimates relative to the true location are summarized below.


| robot | x difference | y difference | 
| ------| ------------- | --------------- |
| green (SLAM) | 0.4 cm | 0.2 cm |
| blue (odometry) | 3.2 cm | 37.3 cm |
