# NUSIM
Author: Katie Hughes

Launch the simulation: 
`ros2 launch nusim nusim.launch.xml`
This will show the red turtle in rviz in an environment with red cylinders, whose coordinates and radius are specified in config/basic_world.yaml.

Reset the simulation: 
`ros2 service call /reset std_srvs/srv/Empty `

Teleport the turtle to a specified location: 
`ros2 service call nusim/teleport nusim/srv/Teleport "x: 2
y: 3.0
theta: 1.0
"
`

![Nusim rviz simulation](images/nusim1.png?raw=true "Nusim rviz simulation")


Worked with: Nick Morales, Ava Zahedi, Allan Garcia-Casal, Liz Metzger, Shantao Cao