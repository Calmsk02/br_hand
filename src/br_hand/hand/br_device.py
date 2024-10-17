#!/usr/bin/python
#
# (Copyright) Bluerobin inc.
# Author: Minsu Kim < minsu309@bluerobin.co.kr, minsu309@snu.ac.kr >
# Description: Bluerobin Hand Class

from br_hand.hand.br_dxl_device import *

class BRhand():
    def __init__(self, port, motor_id, motor_type, motor_mode, description):
        # Init class variables  
        self.n_device       = len(port)
        self.device         = []

        self.position       = []
        self.velocity       = []
        self.current        = []
        
        for i in range(self.n_device):
            device = BRdxlDevice( port[i],
                                motor_id[i],
                                motor_type[i],
                                motor_mode[i],
                                description[i])
            device.bulkread()
            self.device.append(device)
            self.position.append(device.position)
            self.velocity.append(device.velocity)
            self.current.append(device.current)
        self.checkDuplicate()
            
    def checkDuplicate(self):
        _checklist = []
        for _d in self.device:
            for _id in _d.deviceID:
                if _id in _checklist:
                    print("""E:Motor ID duplication has been detected! 
                          Please use differnet IDs for each motor""")
                    quit()
                else:
                    _checklist.append(_id)
            
    def changeMode(self, _id, _mode):
        for _d in self.device:
            if _id in _d.deviceID:
                return _d.setMode(_id, _mode)

    def enable(self, _id, val):
        for _d in self.device:
            if _id in _d.deviceID: 
                return _d.enable(_id, val)
                
    def reboot(self, _id):
        for _d in self.device:
            if _id in _d.deviceID: 
                return _d.reboot(_id)

    def read(self):
        for i, _d in enumerate(self.device):
            _d.bulkread()
            self.position[i] = _d.position
            self.velocity[i] = _d.velocity
            self.current[i] = _d.current
    
    def write(self):
        for i, _d in enumerate(self.device):
            _d.position = self.position[i]
            _d.velocity = self.velocity[i]
            _d.current = self.current[i]
            _d.bulkwrite()
            
        
        