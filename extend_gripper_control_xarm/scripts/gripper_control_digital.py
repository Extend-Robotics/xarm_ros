#!/usr/bin/env python3

import rosnode
import rospy
import sys
import copy
import rospkg
import extend_msgs
from extend_msgs.msg import GripperControl
from std_msgs.msg import String
import xarm_msgs.srv

#Creating the ros node and service client
rospy.init_node("xarm_gripper")
rospy.wait_for_service("/xarm/gripper_config")
rospy.wait_for_service("/xarm/gripper_move")

def dataCallback(msg):
    # Remaping Range [0,1] to [0,850]
    gripper_value = 850 + (-850 * msg.gripperAnalog.data)
    gripper_control = rospy.ServiceProxy("/xarm/gripper_move", xarm_msgs.srv.GripperMove)
    gripper_action = gripper_control(gripper_value)
    
if __name__ == '__main__': 
    #Subscribe to Digital Gripper Data Stream from Unity  
    gripper_speed_service = rospy.ServiceProxy("/xarm/gripper_config", xarm_msgs.srv.GripperConfig)
    gripper_speed_value = gripper_speed_service(5000)
    rospy.Subscriber("extend_gripper_command", GripperControl, dataCallback)
    rospy.spin() 
