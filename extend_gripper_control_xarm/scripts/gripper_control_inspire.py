#!/usr/bin/env python3

import rosnode
import rospy
import sys
import copy
import rospkg
import extend_msgs
from extend_msgs.msg import GripperControl, GripperResponse
from std_msgs.msg import String
import xarm_msgs.srv 
from gripper_response_inspire import GetForceValue,GetJointValues

from xarm_msgs.srv import ConfigToolModbusRequest, GetSetModbusDataRequest
from std_msgs.msg import Float64, Float64MultiArray, Header

#Creating the ros node and service client
rospy.init_node("robotiq_gripper")
rospy.wait_for_service("xarm/config_tool_modbus")
rospy.wait_for_service("xarm/getset_tgpio_modbus_data")

def initialize():
    #Initialize the Modbus service and the response publisher
    pubGripperResponse = rospy.Publisher('extend_gripper_response',GripperResponse,queue_size=0)
    pubGripperCommandRepublisher = rospy.Publisher('extend_gripper_republished_command',GripperControl,queue_size=0)
    gripperModbusService = rospy.ServiceProxy("xarm/getset_tgpio_modbus_data", xarm_msgs.srv.GetSetModbusData)

    return pubGripperResponse,pubGripperCommandRepublisher,gripperModbusService




def dataCallback(msg):
    if len(msg.handJointValues) > 0:
        # Remaping Range [0,1] to [0,2000] and converting to high and low order bytes
        lowOrderLittleFinger,highOrderLittleFinger = SplitDecimal(int(round(20.62*msg.handJointValues[0])))
        lowOrderRingFinger,highOrderRingFinger = SplitDecimal(int(round(20.62*msg.handJointValues[1])))
        lowOrderMiddleFinger,highOrderMiddleFinger = SplitDecimal(int(round(20.62*msg.handJointValues[2])))
        lowOrderIndexFinger,highOrderIndexFinger = SplitDecimal(int(round(20.62*msg.handJointValues[3])))
        lowOrderThumbFinger,highOrderThumbFinger = SplitDecimal(int(round(51.282*(msg.handJointValues[4]+6))))
        lowOrderThumbBaseFinger,highOrderThumbBaseFinger = SplitDecimal(int(round(25*(msg.handJointValues[5]+6))))

        #gripper_modbus_service = rospy.ServiceProxy("xarm/getset_tgpio_modbus_data", xarm_msgs.srv.GetSetModbusData)
        #Commanding the Hand movement
        gripperModbusData = GetSetModbusDataRequest()
        gripperModbusData.send_data = [0x01, 0x06, 0x05, 0xC2,
                                     highOrderLittleFinger,lowOrderLittleFinger,
                                     highOrderRingFinger,lowOrderRingFinger,
                                     highOrderMiddleFinger,lowOrderMiddleFinger,
                                     highOrderIndexFinger,lowOrderIndexFinger,
                                     highOrderThumbFinger,lowOrderThumbFinger, 
                                     highOrderThumbBaseFinger,lowOrderThumbBaseFinger]
        gripperModbusData.respond_len = 6
        gripperModbusData.host_id = 9
        gripperModbusData.is_transparent_transmission = False
        gripperModbusData.use_503_port = False
        gripperModbusService(gripperModbusData) 

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
        pubGripperResponseData.gripperJointValues = GetJointValues(gripperModbusService)
        pubGripperResponseData.gripperForceValues = GetForceValue(gripperModbusService)
        pubGripperResponse.publish(pubGripperResponseData)

        


def SplitDecimal(decimal):
    # Mask to isolate low order byte (8 least significant bits)
    lowOrderMask = 0xFF
    # Mask to isolate high order byte (8 most significant bits)
    highOrderMask = 0xFF00
    
    # Extract low order byte
    lowOrderByte = decimal & lowOrderMask
    # Extract high order byte and shift to the right by 8 bits
    highOrderByte = (decimal & highOrderMask) >> 8
    
    return lowOrderByte, highOrderByte



if __name__ == '__main__': 
    
    (pubGripperResponse,pubGripperCommandRepublisher,gripperModbusService) = initialize()  

    #Reset the Gripper
    gripperModbusData = GetSetModbusDataRequest()
    gripperModbusData.send_data = [0x01, 0x06, 0x05, 0xC2,0x00,0x00,0x03,0xE8,0x03,0xE8,0x00,0x00,0x00,0x00,0x00,0x00]
    gripperModbusData.respond_len = 6
    gripperModbusData.host_id = 9
    gripperModbusData.is_transparent_transmission = False
    gripperModbusData.use_503_port = False
    gripperModbusService(gripperModbusData) 
   
    # #Subscribe to Digital Gripper Data Stream from Unity  
    rospy.Subscriber("extend_gripper_command", GripperControl, dataCallback, queue_size=1)
    rospy.spin() 
