# nuturtle_control
Author: Katie Hughes

## Description

TODO


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
