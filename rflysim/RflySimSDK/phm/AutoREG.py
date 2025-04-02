## @file 
# 该文件定义了QGCCtrlAPI类，用于与QGroundControl进行通信。 QGCCtrlAPI类提供了发送MAVLink命令和处理QGroundControl日志的功能。
# @anchor AutoREG接口库文件

RFLYSIM_FRAME = {
    'Quadcopter' : 1,
    'Fixedwing'  : 2,
    'Vtol'       : 3
    }

'''
Configuration parameters, frame types and primary key definitions:
All frame types of simulated drones
UAV Frame type for v1 version
                            1:  Quadcopter
                            2:  Fixedwing
                            3:  Vtol
'''


QUAD_CMD_CTRL_MAP_REG = {
    '1': 'Armed!',
    '2': 'Disarmed!',
    '3': 'Send fixed-point flight command, fly target position [{},{},{}]',
    '4': 'Send fixed-speed flight command, fly target speed [{},{},{}]',
    '5': 'Send fixed-point landing command, landing position [{},{},{}]',
    '6': 'Send fault injection command \n Fault injection type:{} \n Fault injection parameters:{}'

}

FIXED_CMD_CTRL_MAP_REG = {
    '1': 'Armed!',
    '2': 'Disarmed!',
    '3': 'Send takeoff command, target takeoff location [{},{},{}]',
    '4': 'fixed-point flight command, fly target position [{},{},{}]',
    '5': 'Send fixed-point landing command, landing position [{},{},{}]',
    '6': 'Send fault injection command \n Fault injection type: {} \n Fault injection parameters: {}'

}

RFLYSIM_CMD = {
    'Quadcopter' : QUAD_CMD_CTRL_MAP_REG,
    'Fixedwing'  : FIXED_CMD_CTRL_MAP_REG,
    'Vtol'       : None
    }

MAV_NUM            = 0
MAV_QUADCOPTER_NUM = 0
MAV_FIXEXWING_NUM  = 0
MAV_VTOL_NUM       = 0
'''
The total number of simulated drones
'''
MAV_CASE_DISTRIBUTION_QUADCOPTER = 0
MAV_CASE_DISTRIBUTION_FIXEXWING  = 0
MAV_CASE_DISTRIBUTION_VTOL       = 0
MAV_CASE_LEN                     = 0
'''
Aircraft test case distribution, only used in multi-aircraft mode
'''
MAV_DATA_DISTRIBUTION_QUADCOPTER = 0
MAV_DATA_DISTRIBUTION_FIXEXWING  = 0
MAV_DATA_DISTRIBUTION_VTOL       = 0

'''
Aircraft data folder distribution, only used in multi-aircraft mode
'''


MAV_FRAME = None
MAV_FRAME_NUM = None
MAV_FRAME_DICT = {
    RFLYSIM_FRAME['Quadcopter']   : ['Quadcopter',  MAV_QUADCOPTER_NUM],
    RFLYSIM_FRAME['Fixedwing']    : ['Fixedwing' ,   MAV_FIXEXWING_NUM],
    RFLYSIM_FRAME['Vtol']         : ['Vtol'      ,        MAV_VTOL_NUM]
    }
MAV_TEST_CASE_REG = {
    RFLYSIM_FRAME['Quadcopter']   : MAV_CASE_DISTRIBUTION_QUADCOPTER,
    RFLYSIM_FRAME['Fixedwing']    : MAV_CASE_DISTRIBUTION_FIXEXWING,
    RFLYSIM_FRAME['Vtol']         : MAV_CASE_DISTRIBUTION_VTOL
}
MAV_CASE_INDEX = {
    RFLYSIM_FRAME['Quadcopter']   : 0,
    RFLYSIM_FRAME['Fixedwing']    : 0,
    RFLYSIM_FRAME['Vtol']         : 0
}

MAV_DATA_FOLDER_REG = {
    RFLYSIM_FRAME['Quadcopter']   : MAV_DATA_DISTRIBUTION_QUADCOPTER,
    RFLYSIM_FRAME['Fixedwing']    : MAV_DATA_DISTRIBUTION_FIXEXWING,
    RFLYSIM_FRAME['Vtol']         : MAV_DATA_DISTRIBUTION_VTOL
}
DIND_REG = {
        'Quadcopter': 0,
        'Fixedwing':  0,
        'Vtol':       0
    }


'''
All frame types of simulated drones
UAV Frame type for v1 version
                            1:  Quadcopter
                            2:  Fixedwing
                            3:  Vtol
'''

MAV_CONF = None

TEST_MODE = None
'''
TEST_MODE is used to identify the frame and number of instances for testing!

    Single Frame Single Instance  : TEST_MODE = 1
    Single Frame Multi  Instance  : TEST_MODE = 2
    Multi  Frame Single Instance  : TEST_MODE = 3
    Multi  Frame Multi  Instance  : TEST_MODE = 4
'''

SIM_WAIT_TIME = 25
SIM_WAIT_TIME_REG = SIM_WAIT_TIME

'''Waiting time for opening simulation software'''

LOOP_KEY_DOWN = False
'''LOOP_KEY_DOWN is used to determine whether a round of testing has ended, LOOP_KEY_DOWN = False means it is not over, LOOP_KEY_DOWN = True means it is over, start the next round of testing / end the test'''
SIM_START_PRO_ALL_DOWM = False
'''When the simulation environment of all aircraft is opened, it becomes True'''
SIM_END_PRO_ALL_DOWM = False
'''When the simulation environment of all aircraft is closed, it becomes True'''
SIM_END_PRO_ALL_DOWM_KEY = False

DATA_ALL_DOWN = False

ARMED_WARN        = False
BREAK_DOWN        = False
WARN_FLAG         = False
ALL_FINISHED      = False


TEST_RESULT = {
    'Is_Fall'    : 'No',
    'Fall Time'  : 'No',
    'Fall Vel':'No',
    'Fall Energy':'No',
    'Failsafe Trigger' : 'No',
    'Flight Status After Fault Injection' : 'None',
    'Deviation From Expected Speed After Fault Injection' : 'None',
    'Deviation From Expected Position After Fault Injection' : 'None',
    'Failure Safety Score':'None',
    'Failure Safety Level':'None'
}

LOG_TIMEOUT = 180
RFLY_SPVO_MODE = False
