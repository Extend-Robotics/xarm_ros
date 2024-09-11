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

from xarm_msgs.srv import ConfigToolModbusRequest, GetSetModbusDataRequest
from std_msgs.msg import Float64, Float64MultiArray, Header

def GetForceValue(gripperModbusService):
    fingerForceValue = [0] * 6
    gripperModbusDataRequest = GetSetModbusDataRequest()
    gripperModbusDataRequest.send_data = [0x01, 0x03, 0x06, 0x2E,0x00,0x06]
    gripperModbusDataRequest.respond_len = 15
    gripperModbusDataRequest.host_id = 9
    gripperModbusDataRequest.is_transparent_transmission = False
    gripperModbusDataRequest.use_503_port = False
    response = gripperModbusService(gripperModbusDataRequest) 

    for i in range(6):
        fingerForceValue[i] = CombinedSignedValue(response.respond_data[i+i+3],response.respond_data[i+i+4])
    
    return fingerForceValue


def GetJointValues(gripperModbusService):
    handJointValue = [0] * 6
    gripperModbusDataRequest = GetSetModbusDataRequest()
    gripperModbusDataRequest.send_data = [0x01, 0x03, 0x05, 0xFE,0x00,0x06]
    gripperModbusDataRequest.respond_len = 15
    gripperModbusDataRequest.host_id = 9
    gripperModbusDataRequest.is_transparent_transmission = False
    gripperModbusDataRequest.use_503_port = False
    response = gripperModbusService(gripperModbusDataRequest) 

    for i in range(6):
        handJointValue[i] = CombinedSignedValue(response.respond_data[i+i+3],response.respond_data[i+i+4])  ## Modbus return the Value between 0 - 2000 need to refactor it to the Joint Values

    ##Coverting 0-2000 to joint values for each finger 

    handJointValue[0] = handJointValue[0] / 20.22
    handJointValue[1] = handJointValue[1] / 20.22
    handJointValue[2] = handJointValue[2] / 20.22
    handJointValue[3] = handJointValue[3] / 20.22
    handJointValue[4] = handJointValue[4] / 90.909
    handJointValue[5] = handJointValue[5] / 31.746
    return handJointValue

    
def CombinedSignedValue(upperValue, lowerValue):
    combinedHexValue = hex(upperValue)[2:] + hex(lowerValue)[2:]
    combinedSignedIntValue = int(combinedHexValue, 16)
    if combinedSignedIntValue & (1 << (16 - 1)):
        combinedSignedIntValue -= 1 << 16
    return combinedSignedIntValue
    
    
