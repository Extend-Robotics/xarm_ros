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



def initialize():
    #Initialize the Modbus service and the response publisher
    pubGripperCommandRepublisher = rospy.Publisher('extend_gripper_republished_command',GripperControl,queue_size=1)
    pubGripperResponse = rospy.Publisher('extend_gripper_response',GripperResponse,queue_size=1)
    return pubGripperCommandRepublisher,pubGripperResponse


def dataCallback(msg):
    # Remaping Range [0,1] to [0,850]
    gripper_value = 850 + (-850 * msg.gripper_analog.data)
    gripper_control = rospy.ServiceProxy(gripperMoveServiceName, xarm_msgs.srv.GripperMove)
    gripper_action = gripper_control(gripper_value)

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

    
if __name__ == '__main__': 
    #Creating the ros node and service client
    rospy.init_node("xarm_gripper")

    gripperConfigServiceName = os.environ['ROS_NAMESPACE']  + "/xarm/gripper_config"
    gripperMoveServiceName = os.environ['ROS_NAMESPACE']  + "/xarm/gripper_move"

    rospy.wait_for_service(gripperConfigServiceName)
    rospy.wait_for_service(gripperMoveServiceName)


  
    (pubGripperCommandRepublisher,pubGripperResponse) = initialize()  
    gripper_speed_service = rospy.ServiceProxy(gripperConfigServiceName, xarm_msgs.srv.GripperConfig)
    gripper_speed_value = gripper_speed_service(5000)
    #Subscribe to Gripper Data Stream from Unity
    rospy.Subscriber("extend_gripper_command", GripperControl, dataCallback, queue_size=1)
    rospy.spin() 
