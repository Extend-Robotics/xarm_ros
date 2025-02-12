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

#Creating the ros node and service client
rospy.init_node("lite6_gripper")
rospy.wait_for_service("ufactory/close_lite6_gripper")
rospy.wait_for_service("ufactory/open_lite6_gripper")

def initialize():
    #Initialize the Modbus service and the response publisher
    pubGripperCommandRepublisher = rospy.Publisher('extend_gripper_republished_command',GripperControl,queue_size=1)
    pubGripperResponse = rospy.Publisher('extend_gripper_response',GripperResponse,queue_size=1)
    return pubGripperCommandRepublisher,pubGripperResponse


def dataCallback(msg):
    # Remaping Range [0,1] to [0,850]
    if msg.gripperDigital.data:
        gripperControl = rospy.ServiceProxy("ufactory/close_lite6_gripper", xarm_msgs.srv.Call)
        gripperAction = gripperControl()
    else:
        gripperControl = rospy.ServiceProxy("ufactory/open_lite6_gripper", xarm_msgs.srv.Call)
        gripperAction = gripperControl()

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
    #Subscribe to Digital Gripper Data Stream from Unity
    #gripper_speed_service = rospy.ServiceProxy("/xarm/gripper_config", xarm_msgs.srv.GripperConfig)
    #gripper_speed_value = gripper_speed_service(5000)
    (pubGripperCommandRepublisher,pubGripperResponse) = initialize()
    rospy.Subscriber("extend_gripper_command", GripperControl, dataCallback)
    rospy.spin()

