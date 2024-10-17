#!/usr/bin/python
#
# (Copyright) Bluerobin inc.
# Author: Minsu Kim < minsu309@bluerobin.co.kr, minsu309@snu.ac.kr >
# Description: DXL motor parameters for Bluerobin Hand

from enum import Enum

# Communication
PROTOCOL_VERSION                = 2
BAUDRATE                        = 4000000

# Operating mode
TORQUE_CONTROL_MODE             = 0
VELOCITY_CONTROL_MODE           = 1
POSITION_CONTROL_MODE           = 3
TORQUE_BASED_POSITION_MODE      = 5

# Motor Enable params
## 0 for disabling the torque
## 1 for enabling the torque
TORQUE_ENABLE                   = 1
TORQUE_DISABLE                  = 0

class BR_CMD_TYPE(Enum):
    CONNECT                     = 0
    ENABLE                      = 1
    CHANGEMODE                  = 2
    REBOOT                      = 3
    
class BR_ERROR(Enum):
    CONNECT                     = 0
    ENABLE                      = 1
    CHANGEMODE                  = 2
    REBOOT                      = 3
    COMMUNICATION               = 4

# PRO Series memory address information
class DXL_PRO:
    # memory addresses
    addr_operation_mode:int     = 11
    addr_torque_enable:int      = 562
    addr_goal_position:int      = 596
    addr_goal_velocity:int      = 600
    addr_goal_current:int       = 604
    addr_present_position:int   = 611
    addr_present_velocity:int   = 615
    addr_present_current:int    = 621

    # parameters
    len_goal_position:int       = 4
    len_goal_velocity:int       = 4
    len_goal_current:int        = 2
    len_present_position:int    = 4
    len_present_velocity:int    = 4
    len_present_current:int     = 2


# X Series memory address information
class DXL_X:
    # memory addresses
    addr_operation_mode:int     = 11
    addr_torque_enable:int      = 64
    addr_goal_position:int      = 116
    addr_goal_velocity:int      = 104
    addr_goal_current:int       = 102
    addr_present_position:int   = 132
    addr_present_velocity:int   = 128
    addr_present_current:int    = 126

    # parameters
    len_goal_position:int       = 4
    len_goal_velocity:int       = 4
    len_goal_current:int        = 2
    len_present_position:int    = 4
    len_present_velocity:int    = 4
    len_present_current:int     = 2
    

# Motor SERIES
# 0 for Pro series
# 1 for X series
DXL_SERIES  =  {'H54-100-S500-R':   DXL_PRO,              
                'H54-200-S500-R':   DXL_PRO,              
                'H42-20-S300-R':    DXL_PRO,
                'XH430-V210':       DXL_X,
                'XC330-T181':       DXL_X,
                'XC330-T288':       DXL_X}


# Encoder pulse info:
# H54-100-S500-R    -250961~250961  / (0 for 0 deg)
# H54-200-S500-R    -250961~250961  / (0 for 0 deg)
# H42-20-S300-R     -151875~151875  / (0 for 0 deg)
# XH430-V210        0~4095          / (2048 for 0 deg)
# XC330-T181        0~4095          / (2048 for 0 deg)
DXL_MAXINCRE =  {'H54-100-S500-R':   250961,         
                'H54-200-S500-R':   250961,         
                'H42-20-S300-R':    151875,         
                'XH430-V210':       2048,           
                'XC330-T181':       2048,
                'XC330-T288':       2048}


# Velocity info:
# H54-100-S500-R -17,000~17,000 / (0.00199234 rev/min)
# H54-200-S500-R -17,000~17,000 / (0.00199234 rev/min)
# H42-20-S300-R -10,300~10,300 / (0.00329218 rev/min)
# XH430-V210 -230~230 / (0.229 rev/min)
# XC330-T181 -230~230 / (0.229 rev/min)
DXL_VELUNIT =  {'H54-100-S500-R':   0.00199234,     
                'H54-200-S500-R':   0.00199234,     
                'H42-20-S300-R':    0.00329218,     
                'XH430-V210':       0.229,          
                'XC330-T181':       0.229,
                'XC330-T288':       0.229}

DXL_MAXVEL  =  {'H54-100-S500-R':   17000,
                'H54-200-S500-R':   17000,
                'H42-20-S300-R':    10300,
                'XH430-V210':       230,
                'XC330-T181':       510,
                'XC330-T288':       320,}

# Current info:
# H54-100-S500-R -310~310 / (16.11328 mA)
# H54-200-S500-R -620~620 / (16.11328 mA)
# H42-20-S300-R -465~465 / (4.02832 mA)
# XH430-V210 -689~689 / (1.34 mA)
# XC330-T181 -910~910 / (1 mA)
DXL_CURUNIT = {'H54-100-S500-R':    16.11328,       
                'H54-200-S500-R':   16.11328,       
                'H42-20-S300-R':    4.02832,        
                'XH430-V210':       1.34,           
                'XC330-T181':       1,
                'XC330-T288':       1}              

DXL_MAXCUR  =  {'H54-100-S500-R':   310,
                'H54-200-S500-R':   620,
                'H42-20-S300-R':    465,
                'XH430-V210':       689,
                'XC330-T181':       910,
                'XC330-T288':       910}
