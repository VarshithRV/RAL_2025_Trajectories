ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur16e
ros2 launch ur_robot_driver ur_control.launch.py launch_rviz:=false ur_type:=ur16e robot_ip:=192.168.1.7
