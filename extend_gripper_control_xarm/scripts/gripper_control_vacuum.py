#!/usr/bin/env python3

import rosnode
import rospy
import sys
import copy
import rospkg
import extend_msgs
from extend_msgs.msg import GripperControl, GripperResponse
from std_msgs.msg import Header
import xarm_msgs.srv
import os

isInitialValue = True 
gripperValueReceived = False

def initialize():
    #Initialize the Modbus service and the response publisher
    pubGripperCommandRepublisher = rospy.Publisher('extend_gripper_republished_command',GripperControl,queue_size=1)
    pubGripperResponse = rospy.Publisher('extend_gripper_response',GripperResponse,queue_size=1)
    return pubGripperCommandRepublisher,pubGripperResponse


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

    header = Header()
    header.seq = 0
    header.frame_id = ""
    header.stamp = rospy.Time.now()

    pubGripperCommandRepublisherData = GripperControl()
    pubGripperCommandRepublisherData = msg
    pubGripperCommandRepublisherData.header = header
    pubGripperCommandRepublisher.publish(pubGripperCommandRepublisherData)

    #Fetching the Gripper Response Joint States and Force
    pubGripperResponseData = GripperResponse()
    pubGripperResponseData.header = header
    pubGripperResponse.publish(pubGripperResponseData)

def serviceCall(gripperValue): 
    print("Service Call was made")   
    gripperControl = rospy.ServiceProxy(vacuumGripperSetServiceName, xarm_msgs.srv.SetInt16)
    gripperAction = gripperControl(gripperValue)
    
if __name__ == '__main__': 
    #Creating the ros node and service client
    rospy.init_node("xarm_gripper")
    vacuumGripperSetServiceName = os.environ['ROS_NAMESPACE']  + "/xarm/vacuum_gripper_set"
    rospy.wait_for_service(vacuumGripperSetServiceName)

    print("Starting the script to control Vacuum Gripper")
    (pubGripperCommandRepublisher,pubGripperResponse) = initialize()
    #Subscribe to Digital Gripper Data Stream from Unity  
    rospy.Subscriber("extend_gripper_command", GripperControl, dataCallback, queue_size=1)
    rospy.spin() 
