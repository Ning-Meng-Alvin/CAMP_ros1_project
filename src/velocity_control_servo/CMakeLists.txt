cmake_minimum_required(VERSION 3.0.2)
project(velocity_control_servo_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(velocity_control_servo_node src/velocity_control_servo_node.cpp)
add_dependencies(velocity_control_servo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_control_servo_node ${catkin_LIBRARIES})
