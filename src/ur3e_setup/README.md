# Steps to follow for real UR3e Arm

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
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.77.21 kinematics_config:=/home/user/workspace/src/ur3e2_calib.yaml z_height:=0.8

# Start MoveIt for UR3e
roslaunch ur3e_moveit_config ur3e_moveit_planning_execution.launch

# Start RViz
roslaunch ur3e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur3e_moveit_config)/launch/moveit.rviz
```

# Docker run for real robot
docker run -it --rm --name UR3Container --net=host --pid=host --privileged --env="DISPLAY=$DISPLAY" --volume="$PWD:/home/${USER}/workspace/src" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/dev:/dev:rw" --ulimit rtprio=99 --ulimit rttime=-1 ur3e_image:latest

# install missing packages
rosdep install --from-paths src --ignore-src -r

# Initiate UR driver communication with real robot with gripper
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.77.22 kinematics_config:=$(rospack find ur_calibration)/calib/ur3e_calib.yaml z_height:=0.766 use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR

# Run the 2-Finger Gripper Driver Node
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /tmp/ttyUR

# Run the 2-Finger Gripper Simple Controller Node
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

# Run the 2-Finger Gripper Status Listener Node
rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py






