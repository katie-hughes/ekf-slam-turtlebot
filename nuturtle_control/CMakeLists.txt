cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(turtlelib REQUIRED)


find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/InitialPose.srv" "srv/Control.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")


add_executable(turtle_control src/turtle_control.cpp)
ament_target_dependencies(turtle_control
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  nuturtlebot_msgs
  nav_msgs)

target_link_libraries(turtle_control turtlelib::turtlelib "${cpp_typesupport_target}")

add_executable(odometry src/odometry.cpp)
ament_target_dependencies(odometry
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  nuturtlebot_msgs
  nav_msgs)

target_link_libraries(odometry turtlelib::turtlelib "${cpp_typesupport_target}")


add_executable(circle src/circle.cpp)
ament_target_dependencies(circle
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  nuturtlebot_msgs
  nav_msgs)

target_link_libraries(circle turtlelib::turtlelib "${cpp_typesupport_target}")


add_executable(mytest src/test.cpp)
ament_target_dependencies(mytest
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  nuturtlebot_msgs
  nav_msgs
  tf2_msgs
  rmw)

target_link_libraries(mytest turtlelib::turtlelib "${cpp_typesupport_target}")

install(TARGETS
  turtle_control odometry circle mytest
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
