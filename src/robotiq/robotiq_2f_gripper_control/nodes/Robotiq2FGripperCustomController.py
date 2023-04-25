#!/usr/bin/env python

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep
from std_msgs.msg import String


def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150

    if char == 'r':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 0

    if char == 'c':
        command.rPR = 255

    if char == 'o':
        command.rPR = 0   

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.rPR = int(char)
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.rSP += 25
        if command.rSP > 255:
            command.rSP = 255
            
    if char == 'l':
        command.rSP -= 25
        if command.rSP < 0:
            command.rSP = 0

            
    if char == 'i':
        command.rFR += 25
        if command.rFR > 255:
            command.rFR = 255
            
    if char == 'd':
        command.rFR -= 25
        if command.rFR < 0:
            command.rFR = 0

    return command
        
def callback(data):
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    
    command  = outputMsg.Robotiq2FGripper_robot_output();

    if data.data == 'c' :
        command = genCommand('a', command)
        command = genCommand('c', command)
    
    elif data.data == 'o' :
        command = genCommand('a', command)
        command = genCommand('o', command)
    
    elif data.data == 'r' :
        command = genCommand('r', command)
    
    elif data.data == 'a' :
        command = genCommand('a', command)
    
    pub.publish(command)

def publisher():
    """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
    rospy.init_node('Robotiq2FGripperCustomController')
    
    sub = rospy.Subscriber('gripper_command', String, callback)

    rospy.spin() 
                        

if __name__ == '__main__':
    publisher()
