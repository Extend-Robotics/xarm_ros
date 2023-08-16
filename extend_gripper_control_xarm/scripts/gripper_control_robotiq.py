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
    # Remaping Range [0,1] to [0,255]
    gripper_value = 255 * msg.gripperAnalog.data
    gripper_modbus_service = rospy.ServiceProxy("/xarm/getset_tgpio_modbus_data", xarm_msgs.srv.GetSetModbusData)
    gripper_modbus_data = GetSetModbusDataRequest()
    gripper_modbus_data.send_data = [0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x09, 0x00, 0x00, gripper_value, 0xFF, 0xFF]
    gripper_modbus_data.respond_len = 6
    gripper_modbus_data.host_id = 9
    gripper_modbus_data.is_transparent_transmission = False
    gripper_modbus_data.use_503_port = False
    gripper_modbus_service(gripper_modbus_data)

if __name__ == '__main__': 
    #Configure the Baudrate for the tool modbus
    gripper_baudrate_service = rospy.ServiceProxy("/xarm/config_tool_modbus", xarm_msgs.srv.ConfigToolModbus)
    gripper_baudrate_config = ConfigToolModbusRequest()
    gripper_baudrate_config.baud_rate = 115200  #Baudrate for the Robotiq grippers
    gripper_baudrate_config.timeout_ms = 3
    gripper_baudrate_service(gripper_baudrate_config)    

    #Reset Gripper 
    gripper_modbus_service = rospy.ServiceProxy("/xarm/getset_tgpio_modbus_data", xarm_msgs.srv.GetSetModbusData)
    gripper_modbus_data = GetSetModbusDataRequest()
    gripper_modbus_data.send_data = [0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    gripper_modbus_data.respond_len = 6
    gripper_modbus_data.host_id = 9
    gripper_modbus_data.is_transparent_transmission = False
    gripper_modbus_data.use_503_port = False
    gripper_modbus_service(gripper_modbus_data) 


    #Activate Gripper 
    gripper_modbus_data = GetSetModbusDataRequest()
    gripper_modbus_data.send_data = [0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
    gripper_modbus_data.respond_len = 6
    gripper_modbus_data.host_id = 9
    gripper_modbus_data.is_transparent_transmission = False
    gripper_modbus_data.use_503_port = False
    gripper_modbus_service(gripper_modbus_data)      

    #Subscribe to Digital Gripper Data Stream from Unity  
    rospy.Subscriber("/extend_gripper_command", GripperControl, dataCallback, queue_size=1)
    rospy.spin() 
