# NUSLAM
Author: Katie Hughes

Launch simulated slam:
`ros2 launch nuslam nuslam.launch.xml`
This will show the red, blue, and green turtles in an environment with cylindrical obstacles. The red robot corresponds to the `nusim` simulated robot. The blue robot corresponds to the `turtle_control` odometry robot. Finally, the green robot corresponds to `slam`'s EKF slam state estimate.

Publishing to `cmd_vel` topic will move the red robot through the environment, which will update the other two robots accordingly. The green slam robot should stay close to the red robot, which will become even more apparent if you drive into the obstacles in the environment (see below).


![Slam Path](images/SlamPath.png?raw=true "Slam Path")



[ANOTHERSLAMSUCCESS.webm](https://user-images.githubusercontent.com/53623710/221661687-1b6214c4-21d7-4cd7-845f-e3173a0a2be5.webm)
