#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# (Copyright) Bluerobin inc.
# Author: Minsu Kim < minsu309@bluerobin.co.kr, minsu309@snu.ac.kr >
# Description: DXL motor Class for Bluerobin Hand

from br_hand.hand.br_dxl_protocol import *
from br_hand.utils.utilities import BRLogger
from dynamixel_sdk import *
import numpy as np
import time
import threading

lock = threading.Lock()

# Basic motor class
class BRdxlDevice():
    def __init__(self, port, m_ids, m_model, mode, description="default"):
        if len(m_ids) != len(m_model) or len(m_ids) != len(mode):
            print("Please check the device informations (class input).")
            quit()

        # Motor Information
        self.port               = port
        self.n_device           = len(m_ids)
        self.deviceID           = m_ids
        self.deviceModel        = m_model
        self.deviceMode         = mode
        self.description        = description
        
        # Port and Data handler
        self.portHandler        = PortHandler(self.port)
        self.packetHandler      = PacketHandler(PROTOCOL_VERSION)
        self.groupBulkRead      = GroupBulkRead(self.portHandler, self.packetHandler)
        self.groupBulkWrite4    = GroupBulkWrite(self.portHandler, self.packetHandler)
        self.groupBulkWrite2    = GroupBulkWrite(self.portHandler, self.packetHandler)
        
        # data logger
        self.logger             = BRLogger("hand")
        
        # Set protocols
        self.setProtocols()

        # Motor data
        self.current            = np.zeros(self.n_device, dtype=np.float32)
        self.position           = np.zeros(self.n_device, dtype=np.float32)
        self.velocity           = np.zeros(self.n_device, dtype=np.float32)
    
    def __del__(self):
        if self.portHandler:
            self.portHandler.clearPort()
            
    def openPort(self):
        # Open port
        try: self.portHandler.closePort()
        except: pass
        if self.portHandler.openPort():
            self.logger.log(f"\nSucceeded to open the port {self.port}")
        else: 
            self.logger.log(f"\nFailed to open the port {self.port}")
            quit()
            
    def closePort(self):
        self.portHandler.closePort()
        
    def setBaudRate(self, baudrate):
        # Set port baudrate
        if self.portHandler.setBaudRate(baudrate): 
            self.logger.log("Succeeded to change the port baudrate")
        else: 
            self.logger.log("Failed to change the port baudrate")
            quit()
            
    def setProtocols(self):
        self.openPort()
        self.setBaudRate(BAUDRATE)

        # Reboot the device
        for dxl_id in self.deviceID:
            self.reboot(dxl_id)
        
        # Add bulkread params
        for index, (dxl_id, model, mode) in enumerate(zip(self.deviceID, self.deviceModel, self.deviceMode)):
            data_len = DXL_SERIES[model].len_present_position
            data_len += DXL_SERIES[model].len_present_velocity
            data_len += DXL_SERIES[model].len_present_current
            result = self.groupBulkRead.addParam(dxl_id, DXL_SERIES[model].addr_present_current, data_len)
            if result == 0:
                self.logger.log(f"[ID:{dxl_id}] groupBulkRead addparam failed")
                quit()
            
            # set control mode
            self.setMode(dxl_id, mode)
            
        print("--------\nDXL Motor Infos")
        print(f"IDS:{self.deviceID}")
        print(f"Mode: {self.deviceMode}")
        print(f"PORT: {self.port}")
        print(f"Type:{self.deviceModel}")
        print(f"Description: {self.description}")


    def reboot(self, dxl_id):
        with lock:
            time.sleep(0.1)
            dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, dxl_id)
            if dxl_comm_result != COMM_SUCCESS:
                self.logger.log(f"[ID:{dxl_id}] failed to reboot {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return 0
            elif dxl_error != 0:
                self.logger.log(f"[ID:{dxl_id}] failed to reboot {self.packetHandler.getRxPacketError(dxl_error)}")
                return 0
            else:
                self.logger.log(f"[ID:{dxl_id}] reboot Success")
                return 1


    def setMode(self, dxl_id, mode):
        with lock:
            time.sleep(0.1)
            index = self.deviceID.index(dxl_id)
            model = self.deviceModel[index]
            mode = int(mode)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, 
                                                                        dxl_id, 
                                                                        DXL_SERIES[model].addr_operation_mode, 
                                                                        mode)
            dxl_error # not used
            if dxl_comm_result != COMM_SUCCESS:
                self.logger.log(f"[ID:{dxl_id}] Failed to change the mode {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return 0
            elif dxl_error != 0:
                self.logger.log(f"[ID:{dxl_id}] Failed to change the mode {self.packetHandler.getRxPacketError(dxl_error)}")
                return 0
            else:
                self.logger.log(f"[ID:{dxl_id}] successfully changed to mode {mode}")
                self.deviceMode[index] = mode
                return 1


    def enable(self, dxl_id, val):
        with lock:
            time.sleep(0.1)
            index = self.deviceID.index(dxl_id)
            model = self.deviceModel[index]
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, 
                                                                            dxl_id, 
                                                                            DXL_SERIES[model].addr_torque_enable, 
                                                                            val)
            text1 = {True:  "enable",   False:  "disable"}
            text2 = {True:  "enabled",  False:  "disabled"}
            if dxl_comm_result != COMM_SUCCESS:
                self.logger.log(f"[ID:{dxl_id}] motor {text1[val]} failed {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return 0
            elif dxl_error != 0:
                self.logger.log(f"[ID:{dxl_id}] motor {text1[val]} failed {self.packetHandler.getRxPacketError(dxl_error)}")
                return 0
            else:
                self.logger.log(f"[ID:{dxl_id}] motor successfully {text2[val]}")
                return 1

    def bulkread(self):
        with lock:
            dxl_comm_result = self.groupBulkRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("[bulkRead] %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            for index, (dxl_id, model) in enumerate(zip(self.deviceID, self.deviceModel)):
                # Read angle in radian (-3.14 ~ 3.14)
                addr = DXL_SERIES[model].addr_present_position
                len  = DXL_SERIES[model].len_present_position
                if self.groupBulkRead.isAvailable(dxl_id, addr, len):
                    self.position[index] = np.array(self.groupBulkRead.getData(dxl_id, addr, len)).astype(np.int32)
                    if DXL_SERIES[model] == DXL_PRO:
                        self.position[index] -= DXL_MAXINCRE[model]
                    self.position[index] = self.position[index]/DXL_MAXINCRE[model]*np.pi
                
                # Read angular velocity in radian/s
                addr = DXL_SERIES[model].addr_present_velocity
                len  = DXL_SERIES[model].len_present_velocity
                if self.groupBulkRead.isAvailable(dxl_id, addr, len):
                    self.velocity[index] = np.array(self.groupBulkRead.getData(dxl_id, addr, len)).astype(np.int32)
                # Read current mA (~mNm)
                addr = DXL_SERIES[model].addr_present_current
                len  = DXL_SERIES[model].len_present_current
                if self.groupBulkRead.isAvailable(dxl_id, addr, len):
                    self.current[index] = np.array(self.groupBulkRead.getData(dxl_id, addr, len)).astype(np.int16)

    def bulkwrite(self):
        with lock:
            num_bulkwrite4 = 0
            num_bulkwrite2 = 0
            for index, (dxl_id, model) in enumerate(zip(self.deviceID, self.deviceModel)):
                if self.deviceMode[index] == POSITION_CONTROL_MODE or self.deviceMode[index] == TORQUE_BASED_POSITION_MODE:
                    dxl_goal_position = (self.position[index]*DXL_MAXINCRE[model]/np.pi).astype(np.int32)
                    if DXL_SERIES[model] == DXL_PRO:
                        dxl_goal_position += DXL_MAXINCRE[model]
                    
                    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), 
                                            DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), 
                                            DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), 
                                            DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
                    dxl_addparam_result = self.groupBulkWrite4.addParam(dxl_id, 
                                            DXL_SERIES[model].addr_goal_position, 
                                            DXL_SERIES[model].len_goal_position,
                                            param_goal_position)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupBulkWrite addparam failed" % dxl_id)
                    else:
                        num_bulkwrite4 += 1

                elif self.deviceMode[index] == VELOCITY_CONTROL_MODE:
                    dxl_goal_velocity = self.velocity[index].astype(np.int32)
                    param_goal_velocity = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity)), 
                                        DXL_HIBYTE(DXL_LOWORD(dxl_goal_velocity)), 
                                        DXL_LOBYTE(DXL_HIWORD(dxl_goal_velocity)), 
                                        DXL_HIBYTE(DXL_HIWORD(dxl_goal_velocity))]
                    dxl_addparam_result = self.groupBulkWrite4.addParam(dxl_id,
                                            DXL_SERIES[model].addr_goal_velocity, 
                                            DXL_SERIES[model].len_goal_velocity,
                                            param_goal_velocity)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupBulkWrite addparam failed" % dxl_id)
                    else:
                        num_bulkwrite4 += 1

                elif self.deviceMode[index] == TORQUE_CONTROL_MODE:
                    dxl_goal_current = self.current[index].astype(np.int16)
                    param_goal_current = [DXL_LOBYTE(dxl_goal_current),
                                        DXL_HIBYTE(dxl_goal_current)]
                    dxl_addparam_result = self.groupBulkWrite2.addParam(dxl_id, 
                                            DXL_SERIES[model].addr_goal_current, 
                                            DXL_SERIES[model].len_goal_current,
                                            param_goal_current)
                    if dxl_addparam_result != True:
                        print("[ID:%03d] groupBulkWrite addparam failed" % dxl_id)
                    else:
                        num_bulkwrite2 += 1

            if num_bulkwrite4 != 0:
                dxl_comm_result = self.groupBulkWrite4.txPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print("[BulkWrite4] %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            
            if num_bulkwrite2 != 0:
                dxl_comm_result = self.groupBulkWrite2.txPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print("[BulkWrite2] %s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            self.groupBulkWrite4.clearParam()
            self.groupBulkWrite2.clearParam()
        
        
    