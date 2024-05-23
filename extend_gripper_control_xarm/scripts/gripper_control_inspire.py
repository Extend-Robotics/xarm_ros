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

from xarm_msgs.srv import ConfigToolModbusRequest, GetSetModbusDataRequest

#Creating the ros node and service client
rospy.init_node("robotiq_gripper")
rospy.wait_for_service("/xarm/config_tool_modbus")
rospy.wait_for_service("/xarm/getset_tgpio_modbus_data")


def dataCallback(msg):

    # Remaping Range [0,1] to [0,2000]
    lowOrderLittleFinger,highOrderLittleFinger = split_decimal(int(round(20.62*msg.handJointValues[0])))
    lowOrderRingFinger,highOrderRingFinger = split_decimal(int(round(20.62*msg.handJointValues[1])))
    lowOrderMiddleFinger,highOrderMiddleFinger = split_decimal(int(round(20.62*msg.handJointValues[2])))
    lowOrderIndexFinger,highOrderIndexFinger = split_decimal(int(round(20.62*msg.handJointValues[3])))
    lowOrderThumbFinger,highOrderThumbFinger = split_decimal(int(round(51.282*(msg.handJointValues[4]+6))))
    lowOrderThumbBaseFinger,highOrderThumbBaseFinger = split_decimal(int(round(25*(msg.handJointValues[5]+6))))

    gripper_modbus_service = rospy.ServiceProxy("/xarm/getset_tgpio_modbus_data", xarm_msgs.srv.GetSetModbusData)
    gripper_modbus_data = GetSetModbusDataRequest()
    little_finger = 500
    low_order, high_order = split_decimal(little_finger)
    gripper_modbus_data.send_data = [0x01, 0x06, 0x05, 0xC2,
                                     highOrderLittleFinger,lowOrderLittleFinger,
                                     highOrderRingFinger,lowOrderRingFinger,
                                     highOrderMiddleFinger,lowOrderMiddleFinger,
                                     highOrderIndexFinger,lowOrderIndexFinger,
                                     highOrderThumbFinger,lowOrderThumbFinger, 
                                     highOrderThumbBaseFinger,lowOrderThumbBaseFinger]
    gripper_modbus_data.respond_len = 6
    gripper_modbus_data.host_id = 9
    gripper_modbus_data.is_transparent_transmission = False
    gripper_modbus_data.use_503_port = False
    gripper_modbus_service(gripper_modbus_data) 



def split_decimal(decimal):
    # Mask to isolate low order byte (8 least significant bits)
    low_order_mask = 0xFF
    # Mask to isolate high order byte (8 most significant bits)
    high_order_mask = 0xFF00
    
    # Extract low order byte
    low_order_byte = decimal & low_order_mask
    # Extract high order byte and shift to the right by 8 bits
    high_order_byte = (decimal & high_order_mask) >> 8
    
    return low_order_byte, high_order_byte



if __name__ == '__main__': 
    #Configure the Baudrate for the tool modbus
    gripper_baudrate_service = rospy.ServiceProxy("/xarm/config_tool_modbus", xarm_msgs.srv.ConfigToolModbus)
    gripper_baudrate_config = ConfigToolModbusRequest()
    gripper_baudrate_config.baud_rate = 115200  #Baudrate for the Robotiq grippers
    gripper_baudrate_config.timeout_ms = 3
    gripper_baudrate_service(gripper_baudrate_config)    

    # #Reset Gripper 
    gripper_modbus_service = rospy.ServiceProxy("/xarm/getset_tgpio_modbus_data", xarm_msgs.srv.GetSetModbusData)
    gripper_modbus_data = GetSetModbusDataRequest()
    gripper_modbus_data.send_data = [0x01, 0x06, 0x05, 0xC2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    gripper_modbus_data.respond_len = 6
    gripper_modbus_data.host_id = 9
    gripper_modbus_data.is_transparent_transmission = False
    gripper_modbus_data.use_503_port = False
    gripper_modbus_service(gripper_modbus_data) 

    #Activate Gripper 
    # gripper_modbus_data = GetSetModbusDataRequest()
    # gripper_modbus_data.send_data = [0x01, 0x06, 0x05, 0xCE, 0x00, 0x00]
    # gripper_modbus_data.respond_len = 6
    # gripper_modbus_data.host_id = 9
    # gripper_modbus_data.is_transparent_transmission = False
    # gripper_modbus_data.use_503_port = False
    # response = gripper_modbus_service(gripper_modbus_data)  
    # print("request : 01 06 05 CE 00 00 E8 F9 ")
    # print(response)
   
    #Subscribe to Digital Gripper Data Stream from Unity  
    rospy.Subscriber("/extend_gripper_command", GripperControl, dataCallback, queue_size=1)
    rospy.spin() 

