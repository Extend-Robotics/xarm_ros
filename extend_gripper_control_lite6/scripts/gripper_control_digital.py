#!/usr/bin/env python

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
rospy.init_node("lite6_gripper")
rospy.wait_for_service("/ufactory/close_lite6_gripper")
rospy.wait_for_service("/ufactory/open_lite6_gripper")

def dataCallback(msg):
    # Remaping Range [0,1] to [0,850]
    if msg.gripperDigital.data:
        gripperControl = rospy.ServiceProxy("/ufactory/close_lite6_gripper", xarm_msgs.srv.Call)
        gripperAction = gripperControl()
    else:
        gripperControl = rospy.ServiceProxy("/ufactory/open_lite6_gripper", xarm_msgs.srv.Call)
        gripperAction = gripperControl()

if __name__ == '__main__':
    #Subscribe to Digital Gripper Data Stream from Unity
    #gripper_speed_service = rospy.ServiceProxy("/xarm/gripper_config", xarm_msgs.srv.GripperConfig)
    #gripper_speed_value = gripper_speed_service(5000)
    rospy.Subscriber("extend_gripper_command", GripperControl, dataCallback)
    rospy.spin()