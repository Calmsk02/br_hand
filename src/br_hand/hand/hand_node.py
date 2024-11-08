#!/usr/bin/python
#
# (Copyright) Bluerobin inc.
# Author: Minsu Kim < minsu309@bluerobin.co.kr, minsu309@snu.ac.kr >
# Description: Main control code for Bluerobin Hand

import json, os, time
import rospy, rosparam
from br_hand.msg import MotorInfo
from br_hand.srv import MotorCMD, MotorCMDResponse
from br_hand.hand.br_device import BRhand
from br_hand.hand.br_dxl_protocol import *
from br_hand.utils.utilities import get_rosparam_name, get_rosparam_value, squeeze_list

def main():
    
    # Read set values from json file
    file = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'setting.json')
    with open(file, 'r') as f:
        data = json.load(f)
      
        
    # Bluerobin Hand setup
    hand = BRhand(data["port"],
               data["motor_id"],
               data["motor_type"],
               data["motor_mode"],
               data["description"])
    
    # Ros param setup
    rosparam.set_param('port', str(data["port"]))
    rosparam.set_param('gain', str(data["gain"]))
    rosparam.set_param('control', str(data["control"]))
    
    control_param_name = get_rosparam_name("control")
    for i in range(8):
        param_name = control_param_name[2*i]
        val = data["motor_mode"][0][i]
        rosparam.set_param(param_name, str(val))
    
    # ROS Service callback function
    def sevice_callback(req):
        dxl_id = data["motor_id"][0][req.index]
        if req.cmd_type == (BR_CMD_TYPE.CONNECT.value):
            ret = True
        elif req.cmd_type == (BR_CMD_TYPE.ENABLE.value):
            ret = hand.enable(dxl_id, req.val)
        elif req.cmd_type == (BR_CMD_TYPE.CHANGEMODE.value):
            ret = hand.changeMode(dxl_id, req.val)
        elif req.cmd_type == (BR_CMD_TYPE.REBOOT.value):
            ret = hand.reboot(dxl_id)
        return MotorCMDResponse(ret)
    
    try:
        # Ros initialization
        rospy.init_node("BRHand", anonymous=True)
        rate = rospy.Rate(30)
        server = rospy.Service('motor_command', MotorCMD, sevice_callback)
        server # not used (avoid warning)
        publisher = rospy.Publisher('/motor', MotorInfo, queue_size=10)
        
        while not rospy.is_shutdown():
            # read data
            motor_data = MotorInfo()
            hand.read()
            
            # ros: motor data publisher
            motor_data.position = squeeze_list(hand.position)
            motor_data.velocity = squeeze_list(hand.velocity)
            motor_data.current  = squeeze_list(hand.current)
            publisher.publish(motor_data)
            
            # ros: read parameters
            # gain_param = get_rosparam_value("gain")
            control_param = get_rosparam_value("control")
           
            # write data
            for i in range(8):
                if control_param[2*i] == 5:                     # check if ctrl mode is position
                    hand.position[0][i] = control_param[2*i+1]  # update position command
                elif control_param[2*i] == 0:                   # check if ctrl mode is current
                    hand.current[0][i] = control_param[2*i+1]   # update current command
                elif control_param[2*i] == "PD":
                    continue
                    
            hand.write() # bulk write (ref: dynamixel sdk)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass   

if __name__ == '__main__':
    main()