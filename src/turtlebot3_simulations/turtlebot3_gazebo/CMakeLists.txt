################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "std_msgs"
   "std_srvs"
)

set(EXEC_NAME "turtlebot3_drive")

#add_executable(${EXEC_NAME} src/turtlebot3_drive.cpp)
#ament_target_dependencies(${EXEC_NAME} ${dependencies})

add_executable(turtlebot3_drive src/turtlebot3_drive.cpp)
ament_target_dependencies(turtlebot3_drive rclcpp geometry_msgs sensor_msgs nav_msgs tf2)

#add_executable(robot_pilot src/Pilot.cpp) 
#ament_target_dependencies(robot_pilot rclcpp geometry_msgs sensor_msgs nav_msgs tf2 std_msgs)

add_executable(q_learning src/qlearning.cpp) 
ament_target_dependencies(q_learning rclcpp geometry_msgs sensor_msgs nav_msgs tf2 std_msgs std_srvs)

add_executable(run_demo src/run_demo.cpp) 
ament_target_dependencies(run_demo rclcpp geometry_msgs sensor_msgs nav_msgs tf2 std_msgs std_srvs)

add_library(obstacle1 SHARED models/turtlebot3_dqn_world/obstacle_plugin/obstacle1.cc)
target_link_libraries(obstacle1 ${GAZEBO_LIBRARIES})

add_library(obstacle2 SHARED models/turtlebot3_dqn_world/obstacle_plugin/obstacle2.cc)
target_link_libraries(obstacle2 ${GAZEBO_LIBRARIES})

add_library(obstacles SHARED models/turtlebot3_dqn_world/obstacle_plugin/obstacles.cc)
target_link_libraries(obstacles ${GAZEBO_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS 
  turtlebot3_drive
  #robot_pilot
  q_learning
  run_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models rviz worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_package()
