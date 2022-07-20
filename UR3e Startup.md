### Links:
- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
- https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/tag/v1.0.5
- http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial

### Commands
```bash
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.77.22 kinematics_config:=/home/user/workspace/ur3e2_calib.yaml

roslaunch ur3e_moveit_config ur3e_moveit_planning_execution.launch

roslaunch ur3e_moveit_config moveit_rviz.launch config:=true
```
