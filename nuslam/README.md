# NUSLAM
Author: Katie Hughes

Launch simulated slam:
`ros2 launch nuslam nuslam.launch.xml`
This will show the red, blue, and green turtles in an environment with cylindrical obstacles. The red robot corresponds to the `nusim` simulated robot, and the red obstacles are the true obstacle locations. The blue robot corresponds to the `turtle_control` odometry robot. Finally, the green robot corresponds to `slam`'s EKF slam state estimate, and the green obstacles are the obstacle locations according to the EKF results.

Publishing to `cmd_vel` topic will move the red robot through the environment, which will update the other two robots accordingly. The green slam robot should stay close to the red robot, which will become even more apparent if you drive into the obstacles in the environment. The image below show the path of each robot as they travel through the environment and collide with the cylinders.

If you want to change the number of obstacles that the slam algorithm can detect, you need to modify the `max_obstacles` parameter in the `slam.yaml` config file. The current configuration can detect up to 3 obstacles.

Below is an image of the result of my SLAM algorithm with known data association.

![Slam Path](images/LabeledSlamPath.png?raw=true "Slam Path")

Finally, below is a video of my SLAM algorithm working with simulated lidar data (with noise) and unknown data association. The teal obstacles correspond to my circle detection results. 

[BESTSLAM.webm](https://user-images.githubusercontent.com/53623710/224413751-8f278ed7-de68-4f1f-9841-a1173f067ec1.webm)


At the end, the final position of the red `nusim` (real) robot was (-0.022714, 0.028604) m.  The final position of the green SLAM robot was (-0.018955, 0.030844) m. The final position of the blue odometry robot was (0.0091331, -0.34456) m. The offsets of the green and blue robots relative to the red are summarized below.


| robot | x difference | y difference | 
| ------| ------------- | --------------- |
| green (SLAM) | 0.4 cm | 0.2 cm |
| blue (odometry) | 3.2 cm | 37.3 cm |
