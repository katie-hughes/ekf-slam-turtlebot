# nuturtle_control
Author: Katie Hughes

## Description

This package provides an odometry estimate for the turtlebot that can interface with either a simulated or real robot.

Launch the odometry nodes: 
`ros2 launch nuturtle_control start_robot.launch.xml`
This will always launch the `turtle_control` and `odometry` nodes.

If run with option `robot:=localhost`, this will run the `numsr_turtlebot` node in the `numsr_turtlebot` package, which enables the real robot to publish `sensor_data` and `wheel_commands` data. 

If run with option `robot:=nusim` (the default), the nusim node will also be run. This will simulate the position of the "real" robot as the red robot, which will publish `sensor_data` and respond to `wheel_commands`. The odometry robot is always pictured as the blue robot.

This launchfile allows you to control the robot (real or simulated) using the `cmd_vel` topic to publish a desired body twist. If run with `cmd_src:=circle`, this will run the `circle` node, which will command the robot to drive in a circle at a velocity and radius specified by the user. If run with `cmd_src:=teleop`, this will launch `turtlebot3_teleop`;s `teleop_keyboard` node, which allows you to provide desired twists from keyboard inputs.

Finally, you can also specify if you want a visualization in rviz with the `use_rviz` argument set to either true or false. Using rviz is disabled if running with option `robot:=localhost`.


## Testing on the Physical Turtlebot

After running the appropriate launchfile to enable the circle node, I called the /control service to begin driving in a circle of velocity=0.1m/s, radius=0.2m.

```
ros2 service call /control nuturtle_control/srv/Control "velocity: 0.1
radius: 0.2" 

```

Attached are videos of the real robot and the simulation following the circle, as well as some control with the teleop_twist_keyboard.

[Screencast from 02-07-2023 02:10:45 PM.webm](https://user-images.githubusercontent.com/53623710/217355056-224986ab-f284-460a-ac51-71d3b78edb90.webm)

https://user-images.githubusercontent.com/53623710/217356011-f5fc95c1-a762-4d5b-8f22-dd024d270db4.MOV


After this, the following is the offset between odom and blue/base_footprint (excluding covariance fields for readability). The odometry calculations are pretty close to zero, with some small offsets on the order of a few cm. 
```
header:
  stamp:
    sec: 1675800753
    nanosec: 339875593
  frame_id: odom
child_frame_id: blue/base_footprint
pose:
  pose:
    position:
      x: 0.07423022223650083
      y: -0.0036443594243778196
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.19670268415657285
      w: 0.9804631834217945
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
---

```

Worked with: Nick Morales, Liz Metzger, Hang Yin