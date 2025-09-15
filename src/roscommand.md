conda deactivate
source /opt/ros/noetic/setup.bash
source ~/ros1_project/devel/setup.bash



roslaunch franka_panda_moveit_config moveit_planning_execution.launch

cd ~/ros1_project
catkin build franka_panda_moveit_config
source devel/setup.bash


roslaunch franka_panda_moveit_config my_servo.launch

rostopic echo /servo_server/status

rostopic echo /servo_server/delta_twist_cmds

rostopic echo -n 5 /Panda_arm_controller/command