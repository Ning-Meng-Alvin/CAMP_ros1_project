# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "/home/camp/ros1_project/catkin_franka/src/franka_ros/franka_gazebo/include;/home/camp/ros1_project/catkin_franka/libfranka/include;/usr/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3".split(';') if "/home/camp/ros1_project/catkin_franka/src/franka_ros/franka_gazebo/include;/home/camp/ros1_project/catkin_franka/libfranka/include;/usr/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;gazebo_ros_control;kdl_parser;controller_manager;controller_interface;control_toolbox;pluginlib;hardware_interface;transmission_interface;joint_limits_interface;eigen_conversions;urdf;angles;franka_hw;franka_msgs;franka_gripper;franka_example_controllers".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lfranka_hw_sim;-lfranka_gripper_sim;/home/camp/ros1_project/catkin_franka/libfranka/build/libfranka.so.0.9.0;-lorocos-kdl".split(';') if "-lfranka_hw_sim;-lfranka_gripper_sim;/home/camp/ros1_project/catkin_franka/libfranka/build/libfranka.so.0.9.0;-lorocos-kdl" != "" else []
PROJECT_NAME = "franka_gazebo"
PROJECT_SPACE_DIR = "/home/camp/ros1_project/catkin_franka/devel"
PROJECT_VERSION = "0.8.0"
