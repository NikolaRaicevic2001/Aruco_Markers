cmake_minimum_required(VERSION 3.8)
project(aruco_velocity_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install main Python scripts
install(
  PROGRAMS 
    scripts/ArucoPointCloudMatch.py
    scripts/ArucoVelocityEstimator.py
    scripts/ArucoVisualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoMarkersVelocity.msg"
  DEPENDENCIES geometry_msgs
  DEPENDENCIES std_msgs
 )

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

ament_package()