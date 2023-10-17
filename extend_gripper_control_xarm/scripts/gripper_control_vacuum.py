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
rospy.wait_for_service("/xarm/vacuum_gripper_set")

isInitialValue = True 
gripperValueReceived = False

def dataCallback(msg):
    # Remaping Range [0,1] to [0,850]
    global isInitialValue, gripperValueReceived
    if(isInitialValue):
        isInitialValue = False
        gripperValueReceived = msg.gripperDigital.data
        serviceCall(gripperValueReceived)
    else:
        if(gripperValueReceived != msg.gripperDigital.data):
            gripperValueReceived = msg.gripperDigital.data
            serviceCall(gripperValueReceived)

def serviceCall(gripperValue): 
    print("Service Call was made")   
    gripperControl = rospy.ServiceProxy("/xarm/vacuum_gripper_set", xarm_msgs.srv.SetInt16)
    gripperAction = gripperControl(gripperValue)
    
if __name__ == '__main__': 
    
    #Subscribe to Digital Gripper Data Stream from Unity  
    rospy.Subscriber("extend_gripper_command", GripperControl, dataCallback)
    rospy.spin() 
