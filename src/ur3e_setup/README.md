### Robot Preparation
- Make sure External Control URCaps is running
- UR Driver should say: `Robot connected to reverse interface. Ready to receive control commands.`
---
### ROS Preparation
```bash
cd ~/workspace
catkin build
source devel/setup.bash # Once per terminal instance
```
---
### Launch Sequence
```bash
# Initiate UR driver communication with real robot
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.77.22 kinematics_config:=/home/user/workspace/src/ur3e2_calib.yaml z_height:=0.8

# Start MoveIt for UR3e
roslaunch ur3e_moveit_config ur3e_moveit_planning_execution.launch

# Start RViz
roslaunch ur3e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur3e_moveit_config)/launch/moveit.rviz
```

rectified
{"logitech_webcam":{"rot":[-0.35633811354637146,0.5798797011375427,0.6025930643081665,-0.38935157656669617],"trans":[0.9262733459472656,-0.331470251083374,0.7659745216369629]}}

good
{"logitech_webcam":{"rot":[-0.3696555197238922,0.6283388137817383,0.6154246926307678,-0.36251094937324524],"trans":[0.9403526186943054,-0.35773995518684387,0.8288238048553467]}}
