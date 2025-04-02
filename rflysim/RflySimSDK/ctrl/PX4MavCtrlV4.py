import socket
import threading
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import struct
import math
import sys
import copy
import os
import numpy as np
from DllSimCtrlAPI import Data3D
isEnableRdis=False


## @file
#  这是用于RflySim工具链开发的无人系统外部控制接口模块。
#  @anchor PX4MavCtrlV4接口库文件

try:
    import redis
    isEnableRdis=True
except ImportError:
    print("No Redis labs")

try:
    import EarthModel
except ImportError:
    from . import EarthModel

## @class RedisKey
#  @brief RedisKey结构体类
#  @details
class RedisKey:
    """ Redis 通道的 Key 类型
    通过类型 ID, 可以从redisKey中查出通道的字符串名称
    """
    SIL = "SIL"  # 20100
    SIL_TRUE = "SIL_TRUE"  # 30100
    R3D = "RFLYSIM_3D"  # Rflysim 3D
    SIL_RECV = "SIL_RECV"  # 20101
    SIL_TRUE_RECV = "SIL_TRUE_RECV"  # 30101

    @staticmethod
    def GetRedisKey(port):
        return "key" + str(port)


## @class PX4_CUSTOM_MAIN_MODE
#  @brief PX4_CUSTOM_MAIN_MODE结构体类
#  @details
# PX4 main mode enumeration
class PX4_CUSTOM_MAIN_MODE:
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1
    PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
    PX4_CUSTOM_MAIN_MODE_POSCTL = 3
    PX4_CUSTOM_MAIN_MODE_AUTO = 4
    PX4_CUSTOM_MAIN_MODE_ACRO = 5
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
    PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
    PX4_CUSTOM_MAIN_MODE_RATTITUDE = 8
    PX4_CUSTOM_MAIN_MODE_SIMPLE = 9

## @class PX4_CUSTOM_SUB_MODE_AUTO
#  @brief PX4_CUSTOM_SUB_MODE_AUTO结构体类
#  @details
# PX4 sub mode enumeration
class PX4_CUSTOM_SUB_MODE_AUTO:
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
    PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6
    PX4_CUSTOM_SUB_MODE_AUTO_RTGS = 7
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8
    PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND = 9

## @class fifo
#  @brief fifo结构体类
#  @details
# define a class for MAVLink initialization
class fifo(object):
    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)

## @class PosTypeMask
#  @brief PosTypeMask结构体类
#  @details
class PosTypeMask(object):
    """ Obtain the bitmap for offboard message
    https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
    """
    def __init__(self, ignore_all=False):
        """
        默认初始化为速度+偏航角速率模式，当ignore_all为True时，忽略所有offboard控制量
        """
        self.enPos_x = False
        self.enPos_y = False
        self.enPos_z = False
        if ignore_all:
            self.enVel_x = False
            self.enVel_y = False
            self.enVel_z = False
            self.enYawRate = False
        else:
            self.enVel_x = True
            self.enVel_y = True
            self.enVel_z = True
            self.enYawRate = True
        self.enAcc_x = False
        self.enAcc_y = False
        self.enAcc_z = False
        self.enForce = False
        self.enYaw = False

    def mask(self):
        y = int(0)
        if not self.enPos_x:
            y = y | 1
        if not self.enPos_y:
            y = y | (1 << 1)
        if not self.enPos_z:
            y = y | (1 << 2)
        if not self.enVel_x:
            y = y | (1 << 3)
        if not self.enVel_y:
            y = y | (1 << 4)
        if not self.enVel_z:
            y = y | (1 << 5)
        if not self.enAcc_x:
            y = y | (1 << 6)
        if not self.enAcc_y:
            y = y | (1 << 7)
        if not self.enAcc_z:
            y = y | (1 << 8)
        if self.enForce:
            y = y | (1 << 9)
        if not self.enYaw:
            y = y | (1 << 10)
        if not self.enYawRate:
            y = y | (1 << 11)

        return y

## @class AttTypeMask:
#  @brief AttTypeMask结构体类
#  @details
class AttTypeMask(object):
    """ Obtain the bitmap for offboard message
    https://mavlink.io/en/messages/common.html#ATTITUDE_TARGET_TYPEMASK
    """
    def __init__(self, ignore_all=False):
        """
            默认初始化为姿态+油门模式，当ignore_all为True时，忽略所有offboard控制量
        """
        self.enRollRate = False
        self.enPitchRate = False
        self.enYawRate = False
        self.enThrustBody = False
        if ignore_all:
            self.enThrottle = False
            self.enAtt = False
        else:
            self.enThrottle = True
            self.enAtt = True

    def mask(self):
        y = int(0)
        if not self.enRollRate:
            y = y | 1
        if not self.enPitchRate:
            y = y | (1 << 1)
        if not self.enYawRate:
            y = y | (1 << 2)
        if not self.enAtt:
            y = y | (1 << 7)
        return y

## @class PX4ExtMsg:
#  @brief PX4ExtMsg结构体类
#  @details
class PX4ExtMsg:
    def __init__(self):
        self.checksum=0
        self.CopterID=0
        self.runnedTime=0
        self.controls=[0,0,0,0,0,0,0,0]

## @class PX4MavCtrler:
#  @brief PX4MavCtrler结构体类
#  @details
# PX4 MAVLink listen and control API and RflySim3D control API
class PX4MavCtrler:

    """创建一个通信实例
    ID: 如果ID<=10000则表示飞机的CopterID号。如果ID>10000，例如20100这种，则表示通信端口号port。按平台规则，port=20100+CopterID*2-2（为了兼容旧接口的过渡定义，将来ID只表示CopterID）。
    ip: 数据向外发送的IP地址。默认是发往本机的127.0.0.1的IP；在分布式仿真时，也可以指定192.168打头的局域网电脑IP；也可以使用255.255.255.255的广播地址（会干扰网络其他电脑）
    Com: 与Pixhawk的连接模式。
        Com='udp'，表示使用默认的udp模式接收数据，这种模式下，是接收CopterSim转发的PX4的MAVLink消息（或UDP_full,simple）消息包
                 使用port+1端口收和port端口发（例如，1号飞机是20101端口收，20100端口发，与CopterSim对应）。
        Com='COM3'（Widnows下）或 Com='/dev/ttyUSB0'（Linux系统，也可能是ttyS0、ttyAMA0等），表示通过USB线（或者数传）连接飞控，使用默认57600的波特率。注意：波特率使用port口设置，默认port=0，会重映射为57600
        Com='Direct'，表示UDP直连模式（对应旧版接口的真机模式），这种模式下使用使用同一端口收发（端口号有port设置），例如Com='Direct'，port=15551，表示通过15551这一个端口来收发数据
        注意：COM模式和Direct模式下，ID只表示飞机的ID号，而不表示端口号
        Com='redis':使用Redis模式通信，服务器地址为ip，服务器端口为port
    port: UDP模式下默认情况下设为0，会自动根据IP填充，按平台规则，port=20100+CopterID*2-2。如果这里赋值大于0，则会强制使用port定义的端口。
          COM模式下，Port默认表示波特率self.baud=port。如果port=0，则会设置self.baud=57600
          Direct模式下，Port默认表示收发端口号（使用相同端口）
          redis模式下，Port对应服务器端口号self.redisPort = port。如果port=0，则self.redisPort=6379为平台默认值。

    接口示例：
    UDP模式
    PX4MavCtrler(1) # 默认IP
    PX4MavCtrler(1,'192.168.31.24') # 指定IP，用于远程控制

    串口模式
    PX4MavCtrler(1,'127.0.0.1','com1',57600) # 指定串口，并设置波特率
    
    PX4MavCtrler(1,'192.168.31.105','Direct',15552) # 真机直连Direct方式，远端IP为'192.168.31.105'，端口15552

    """
    """
    注意：下面这里是旧版接口，便于用户参考旧规则进行接口升级
    For hardware connection PX4MavCtrler('COM:baud','IP:CopterID') format
    Windows use format PX4MavCtrler('COM3') or PX4MavCtrler('COM3:115200') for Pixhawk USB port connection
    Windows use format 'COM4:57600' for Pixhawk serial port connection
    Linux use format PX4MavCtrler('/dev/ttyUSB0') or PX4MavCtrler('/dev/ttyUSB0:115200') for USB, or '/dev/ttyAMA0:57600' for Serial port (RaspberryPi example)
    PX4MavCtrler('COM3:115200:2'): the second input is the CopterID, in this case CopterID = 2

    For real flight
    PX4MavCtrler(port,'IP:CopterID:Flag'), Flag set to 1 to enable real flight com mode
    for example PX4MavCtrler(15551,'192.168.1.123:1:1') to set IP=192.168.1.123, port=15551,CopterID=1,Flag=1

    """
    # constructor function
    # @brief PX4MavCtrler的构造函数
    # @param ID（默认值为1） 飞机的ID号 
    # @param ip（默认值为'127.0.0.1'）飞机的ip地址
    # @param Com（默认值为'udp'）飞机与Pixhawk的连接模式
    # @param port（默认值为0）端口号
    # @return 无
    def __init__(self, ID=1, ip='127.0.0.1',Com='udp',port=0):
        global isEnableRdis
        self.isInPointMode = False
        self.isCom = False
        self.Com = Com
        self.baud = 115200
        self.isRealFly = 0
        self.ip = ip
        self.isRedis = False

        # 这里是为了兼容之前的PX4MavCtrler('COM3:115200')串口协议，将来会取消
        if type(ID) == str: #如果ID是字符串输入
            Com=ID
            ID=1


        self.CopterID = ID
        self.port = 20100+self.CopterID*2-2


        # UDP模式解析
        if (Com=='udp' or Com=='UDP' or Com=='Udp') and ID>10000: # 如果是UDP通信模式
            # 兼容旧版协议，如果ID是20100等端口输入，则自动计算CopterID
            self.port=ID
            self.CopterID = int((ID-20100)/2)+1

        # 串口连接模式解析
        self.ComName = 'COM3' # 默认值，串口名字

        if Com[0:3]=='COM' or Com[0:3]=='com'  or Com[0:3]=='Com' or Com[0:3]=='/de': # 如果是串口连接方式
            self.isCom = True # 串口通信模式
            strlist = Com.split(':')
            if port==0: # 默认值57600
                self.baud = 57600
            else:
                self.baud = int(port)
            if(len(strlist) >= 2): # 串口号:波特率 协议解析，为了兼容旧接口
                if strlist[1].isdigit():
                    self.baud = int(strlist[1])
            self.ComName = strlist[0] # 串口名字

        # 网络直连模式解析
        if Com[0:6]=='Direct' or Com[0:6]=='direct': # 如果UDP直连的真机模式
            strlist = Com.split(':')
            if(len(strlist) >= 2):
                if strlist[1].isdigit():
                    self.port = int(strlist[1])
            self.isRealFly=1

        # Redis模式解析
        if Com[0:5]=='redis' or Com[0:5]=='Redis' or Com[0:5]=='REDIS':

            if isEnableRdis:
                # Redis相关参数
                self.isRedis = True
                self.redisCon = None
                self.redisHost = ip
                if port <= 0 :
                    self.redisPort = 6379
                else:
                    self.redisPort = port
                self.redisPass = None
                self.pubsub = None
                self.redisKey = {
                    RedisKey.SIL: RedisKey.GetRedisKey(self.port),  # 控制端口：20100+2*(copterId-1)
                    RedisKey.SIL_TRUE: RedisKey.GetRedisKey(self.port + 10000),  # 控制端口：30100+2*(copterId-1)
                    RedisKey.R3D: "GLOBAL_MESSAGE_REDIS_KEY",  # R3D: R3D 接收外部输入的 redis key
                    RedisKey.SIL_RECV: RedisKey.GetRedisKey(self.port + 1),  # 回传端口：20101+2*(copterId-1)
                    RedisKey.SIL_TRUE_RECV: RedisKey.GetRedisKey(self.port + 10000 + 1),  # 回传端口：30101+2*(copterId-1)
                }


        self.f = fifo
        self.stopFlag = False
        self.mav0 = mavlink2.MAVLink(self.f)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_socketUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socketUDP.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_socketTrue = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socketTrue.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.udp_socketPX4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socketPX4.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.udp_socketUE4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socketUE4.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)

        # self.ip = ip  # IP address and port number to send data
        # self.port = port

        self.uavTimeStmp=0
        self.trueTimeStmp=0
        self.uavAngEular = [0, 0, 0]  # Estimated Eular angles from PX4
        self.trueAngEular = [0, 0, 0] # True simulated Eular angles from CopterSim's DLL model
        self.uavAngRate = [0, 0, 0]  # Estimated angular rate from PX4
        self.trueAngRate = [0, 0, 0] # True simulated angular rate from CopterSim's DLL model
        self.uavPosNED = [0, 0, 0] # Estimated Local Pos (related to takeoff position) from PX4 in NED frame
        self.truePosNED = [0, 0, 0] # True simulated position (related to UE4 map center) from CopterSim's DLL model
        self.uavVelNED = [0, 0, 0] # Estimated local velocity from PX4 in NED frame
        self.trueVelNED = [0, 0, 0] # True simulated speed from CopterSim's DLL model  in NED frame
        self.isVehicleCrash=False # is the vehicle crashing with other
        self.isVehicleCrashID=-10 # the vehicle to collide
        self.uavPosGPS = [0, 0, 0,0, 0, 0,0,0,0] # Estimated GPS position from PX4 in NED frame,lat lon alt relative_alt vx vy vz hdg
        self.uavPosGPSHome = [0, 0, 0] # Estimated GPS home (takeoff) position from PX4 in NED frame
        self.uavGlobalPos = [0, 0, 0] # Estimated global position from PX4 that transferred to UE4 map
        self.trueAngQuatern = [0, 0, 0, 0] # True simulated AngQuatern from CopterSim's DLL model
        self.uavAngQuatern = [0, 0, 0, 0] # Estimated AngQuatern from PX4
        self.trueMotorRPMS = [0, 0, 0, 0, 0, 0, 0, 0] # True simulated MotorRPMS from CopterSim's DLL model
        self.uavMotorRPMS = [0, 0, 0, 0, 0, 0, 0, 0] # Estimated MotorPWMs from PX4
        self.trueAccB = [0, 0, 0] # True simulated acc from CopterSim's DLL model
        self.uavAccB = [0, 0, 0] # Estimated acc from PX4
        self.uavGyro = [0, 0, 0] # Estimated Gyro from PX4
        self.uavMag = [0, 0, 0] # Estimated Gyro from PX4
        self.uavVibr = [0, 0, 0] # Estimated vibration xyz from PX4

        # 三维仿真数据真值，对应true**系列数据
        self.out3Ddata=Data3D()

        self.truePosGPS = [0, 0, 0] # True simulated PosGPS from CopterSim's DLL model
        self.trueSimulinkData=[0]*32 #create 32D data，来自DLL模型的OutCopterData接口
        self.useCustGPSOri=False
        self.trueGpsUeCenter=[40.1540302,116.2593683,50]
        self.GpsOriOffset=[0,0,0]
        self.uavThrust=0
        self.zInt=0

        self.EnList = [0,1,0,0,0,1]
        self.type_mask = self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yaw=0
        self.yawrate = 0
        self.isInOffboard = False
        self.isArmed = False
        self.hasSendDisableRTLRC = False
        self.UDPMode=2
        self.startTime= time.time()
        self.MaxSpeed=5
        self.stopFlagTrueData=False
        self.hasTrueDataRec=False
        self.isPX4Ekf3DFixed = False
        self.isArmerror = 0
        self.truegyro = 0
        self.truemag = 0

        self.RCPWMs=[0, 0, 0,0, 0, 0,0, 0]
        self.isRCLoop=False
        self.tRCHz=0.03

        self.isFailsafeEn = False
        self.FailsafeInfo = None
        #print("MAV Instance Created！")


        self.px4ext = PX4ExtMsg()
        self.px4extTrue=False

        self.offMode=0
        self.ctrlMode=0 # Simple模式下飞机的飞行模式

        # For droneyee flight test
        self.cmdVel=[0]*7
        self.system_status = 0
        self.connected = False
        self.batInfo=[0]*2
        self._active = False
        self.nId = 0
        self.isOffBoard = 0

        self.hasMsgDict={}
        self.trigMsgVect=[]
        self.hasMsgEvent=threading.Event()
        self.trueMsgEvent=threading.Event()

        self.geo = EarthModel.EarthModel()


        # 用于网络通信的事件信号
        self.netEvent = threading.Event()

        # 用于MAVLink消息接收的事件信号
        self.uavEvent = threading.Event()
        self.uavMsg=0

        self.t1 = None


    ## @brief 填充或裁剪一个列表
    # - @anchor fillList
    #  @param data 数据
    #  @param inLen 指定数据长度
    #  @param fill（默认值为0） 填充值
    #  @return 裁剪或者填充到长度为inLen的data
    def fillList(self,data,inLen,fill=0):
        if isinstance(data, np.ndarray):
            data = data.tolist()
            
        if isinstance(data, list) and len(data)==inLen:
            return data
        else:
            if isinstance(data, list):
                datLen = len(data)
                if datLen<inLen:
                    data = data + [fill]* (inLen-datLen)
                    
                if datLen>inLen:
                    data = data[0:inLen]
            else:
                data = [data] + [fill]* (inLen-1)
        return data
    
    ## @brief 向无人机发送启动信号
    # - @anchor sendStartMsg
    #  @param copterID（默认值为-1） 无人机ID
    #  - -1表示所有飞机
    #  @return 无
    def sendStartMsg(self,copterID=-1):
        """ send start signals to the network for copters calling waitForStartMsg()
        if copterID=-1, then all copters will start to run
        if copterID>0, then only the copter with specified copterID will start to run
        """
        buf = struct.pack("3i",1234567890,1,copterID)
        """ The struct form is
        struct startSignal{
            int checksum; // set to 1234567890 to verify the data
            int isStart; // should start to run
            int copterID; // the copter's ID to start
        }
        """
        self.udp_socket.sendto(buf, ('224.0.0.10', 20007)) # multicast address '224.0.0.10' and port 20007 are adopted here
        print("Send start Msg")
        time.sleep(0.03)

    ## @brief 等待来自 sendStartMsg 函数发送的启动信号
    # - @anchor waitForStartMsg
    #  @param 无
    #  @return 无
    def waitForStartMsg(self):
        """ Program will block until the start signal from sendStartMsg() is received
        """
        MYPORT = 20007
        MYGROUP = '224.0.0.10'
        ANY = '0.0.0.0'
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        sock.bind((ANY,MYPORT))
        try:
            status = sock.setsockopt(socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                socket.inet_aton(MYGROUP) + socket.inet_aton(ANY))
        except:
            print('Failed to Init multicast!')
        sock.setblocking(1)
        #ts = time.time()

        print("Waiting for start Msg")
        while True:
            try:
                buf,addr = sock.recvfrom(65500)
                if len(buf)==12:
                    """The struct form is
                        struct startSignal{
                            int checksum; // set to 1234567890 to verify the data
                            int isStart; // should start to run
                            int copterID; // the copter's ID to start
                        }
                    """
                    checksum,isStart,ID=struct.unpack('3i',buf)
                    if checksum==1234567890 and isStart:
                        if ID<0 or ID==self.CopterID:
                            print('Got start Msg, continue to run.')
                            break
            except:
                print("Error to listen to Start Msg!")
                sys.exit(0)

    ## @brief 初始化并启动用于无人机控制
    # - @anchor initPointMassModel
    #  @param intAlt（默认值为0） 无人机的初始高度
    #  @param intState（默认值为[0,0,0]）无人机的X坐标、Y坐标和偏航角
    #  @return 无
    def initPointMassModel(self,intAlt=0,intState=[0,0,0]):
        """ Init and start the point mass model for UAV control
        intAlt (unit m) is the init height of the vehicle on the UE4 map, which can be obtained from the CopterSim or UE4
        intState contains the PosX (m), PosY (m) and Yaw (degree) of the vehicle, which denotes the initial state of the vehicle
        it is the same as the value on CopterSim UI.
        """
        if self.isCom or self.isRealFly:
            print('Cannot run Pixhawk in this PointMass Mode!')
            sys.exit(0)
        self.isInPointMode=True
        self.intAlt=intAlt # Init altitude from CopterSim ground height of current map
        self.intStateX=intState[0] # Init PosX of CopterSim
        self.intStateY=intState[1] # Init PosY of CopterSim
        self.intStateYaw=intState[2] # Init Yaw angle of CopterSim
        self.t3 = threading.Thread(target=self.PointMassModelLoop, args=())
        self.t3.start()

    ## @brief 终止模型
    # - @anchor EndPointMassModel
    #  @param 无
    #  @return 无
    def EndPointMassModel(self):
        """ End the point mass model
        """
        self.isInPointMode=False
        time.sleep(0.5)
        self.t3.join()

    ## @brief 调整偏航角在-π到π之间
    # - @anchor yawSat
    #  @param yaw 偏航角
    #  @return 调整后的偏航角
    def yawSat(self,yaw):
        """ satuate the yaw angle from -pi to pi
        """
        if yaw>math.pi:
            yaw = yaw-math.pi*2
            yaw=self.yawSat(yaw)
        elif yaw <-math.pi:
            yaw = yaw+math.pi*2
            yaw=self.yawSat(yaw)
        return yaw

    ## @brief 控制无人机在点质量模型下循环运行
    # - @anchor PointMassModelLoop
    #  @param 无
    #  @return 无
    def PointMassModelLoop(self):
        """ This is the dead loop for point mass model
        """
        # Offboard message sending loop, 100Hz
        self.startTime3 = time.time()
        self.startTime= time.time()
        self.lastTime3 = self.startTime3
        velE=[0,0,0]
        velOff=[0,0,0]
        yawOff=0
        yawRateOff=0
        iNum=0
        while True:
            if not self.isInPointMode:
                break
            self.startTime3 = self.startTime3 + 0.01
            sleepTime = self.startTime3 - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.startTime3 = time.time()

            dt=time.time()-self.lastTime3

            velOff=list(self.vel)
            yawOff=self.yaw
            yawRateOff=self.yawrate

            # Calculate desired speed according to target Position
            if self.EnList[1]!=0: # if speed mode
                velOff=list(self.vel)
                yawRateOff=self.yawrate
            elif self.EnList[0]!=0: # if position mode
                targetPosE=list(self.pos)
                if self.coordinate_frame == mavlink2.MAV_FRAME_BODY_NED:
                    targetPosE[0] = self.pos[0]*math.cos(self.uavAngEular[2])+self.pos[1]*math.sin(self.uavAngEular[2])
                    targetPosE[1] = self.pos[0]*math.sin(self.uavAngEular[2])+self.pos[1]*math.cos(self.uavAngEular[2])
                velOff[0]=self.sat((targetPosE[0]-self.uavPosNED[0])*0.5,self.MaxSpeed)
                velOff[1]=self.sat((targetPosE[1]-self.uavPosNED[1])*0.5,self.MaxSpeed)
                velOff[2]=self.sat((targetPosE[2]-self.uavPosNED[2])*0.5,self.MaxSpeed)
                yawRateOff=self.sat((yawOff-self.uavAngEular[2])*2,math.pi/4)
            else:
                velOff=[0,0,0]
                yawOff=0
                yawRateOff=0

            # Calulate vehicle motion according to desired speed
            velE=list(velOff)
            if self.coordinate_frame == mavlink2.MAV_FRAME_BODY_NED:
                velE[0] = velOff[0]*math.cos(self.uavAngEular[2])+velOff[1]*math.sin(self.uavAngEular[2])
                velE[1] = velOff[0]*math.sin(self.uavAngEular[2])+velOff[1]*math.cos(self.uavAngEular[2])

            self.uavVelNED[0]=self.sat(self.uavVelNED[0]*0.97+velE[0]*0.03,15)
            self.uavVelNED[1]=self.sat(self.uavVelNED[1]*0.97+velE[1]*0.03,15)
            self.uavVelNED[2]=self.sat(self.uavVelNED[2]*0.97+velE[2]*0.03,10)

            self.uavAngRate[2]=self.sat(self.uavAngRate[2]*0.97+yawRateOff*0.03,math.pi)

            # if reach ground
            if self.uavPosNED[2]>0 and velOff[2]>0:
                self.uavVelNED=list([0,0,0])
                self.uavAngRate[2]=0

            self.uavPosNED[0]=self.uavVelNED[0]*dt+self.uavPosNED[0]
            self.uavPosNED[1]=self.uavVelNED[1]*dt+self.uavPosNED[1]
            self.uavPosNED[2]=self.uavVelNED[2]*dt+self.uavPosNED[2]
            self.uavAngEular[2]=self.uavAngRate[2]*dt+self.uavAngEular[2]

            self.uavAngEular[2]=self.yawSat(self.uavAngEular[2])
            bodyVx = self.uavVelNED[0]*math.cos(-self.uavAngEular[2])+self.uavVelNED[1]*math.sin(-self.uavAngEular[2])
            bodyVy = self.uavVelNED[0]*math.sin(-self.uavAngEular[2])+self.uavVelNED[1]*math.cos(-self.uavAngEular[2])


            # Calulate desired angle according to speed
            self.uavAngEular[0]=bodyVy/15*math.pi/3
            self.uavAngEular[1]=-bodyVx/15*math.pi/3


            # calculate vehicle state for UE4
            if self.uavPosNED[2]<-0.01:
                MotorRPMS=[1000,1000,1000,1000,1000,1000,1000,1000]
            else:
                MotorRPMS=[0,0,0,0,0,0,0,0]

            self.trueVelNED=list(self.uavVelNED)
            self.trueAngRat=list(self.uavAngRate)
            self.truePosNED[0]=self.intStateX+self.uavPosNED[0]
            self.truePosNED[1]=self.intStateY+self.uavPosNED[1]
            self.truePosNED[2]=self.intAlt+self.uavPosNED[2]
            self.trueAngEular[0]=self.uavAngEular[0]
            self.trueAngEular[1]=self.uavAngEular[1]
            self.trueAngEular[2]=self.yawSat(self.uavAngEular[2]+self.intStateYaw)
            self.uavGlobalPos=list(self.truePosNED)
            self.uavTimeStmp=time.time()
            self.trueTimeStmp=time.time()

            isSendUe4msg=True
            if self.CopterID>4:
                isSendUe4msg=False
                iNum=iNum+1
                if iNum%3==0:
                    isSendUe4msg=True
            # if vehicle number<=4, send UE4 msg with 100Hz
            # if vehicle number is too large, send UE4 msg with 33Hz to save network
            if isSendUe4msg:
                # Send vehicle to UE4
                #sendUE4PosNew(self,copterID,vehicleType,PosE,AngEuler,Vel,PWMs,runnedTime
                runnedTime = time.time()-self.startTime
                self.sendUE4PosNew(self.CopterID,3,self.truePosNED,self.trueAngEular,self.trueVelNED,MotorRPMS,runnedTime)
            self.lastTime3=time.time()
        #print("Point Mode Stoped.")

    ## @brief 发送无人机位姿和状态到Rflysim3D
    #  - @anchor sendUE4PosNew
    #  @param copterID（默认值为1） 无人机ID
    #  @param vehicleType（默认值为3） 无人机类型
    #  @param PosE（默认值为[0, 0, 0]） 无人机坐标
    #  @param AngEuler（默认值为[0, 0, 0]） 欧拉角
    #  @param VelE（默认值为[0, 0, 0]） XYZ方向的速度
    #  @param PWMs（默认值为[0] * 8） 电机转速
    #  @param runnedTime（默认值为-1） 当前时间戳
    #  @param windowID（默认值为-1） 窗口号
    #  @return 无    
    def sendUE4PosNew(
        self,
        copterID=1,
        vehicleType=3,
        PosE=[0, 0, 0],
        AngEuler=[0, 0, 0],
        VelE=[0, 0, 0],
        PWMs=[0] * 8,
        runnedTime=-1,
        windowID=-1,
    ):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        # //输出到模拟器的数据
        # struct SOut2SimulatorSimpleTime {
        #     int checkSum; //1234567891
        #     int copterID;  //Vehicle ID
        #     int vehicleType;  //Vehicle type
        #     int PosGpsInt[3];   //lat*10^7,lon*10^7,alt*10^3，int型发放节省空间
        #     float MotorRPMS[8];
        #     float VelE[3];
        #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
        #     double PosE[3];   //NED vehicle position in earth frame (m)
        #     double runnedTime; //Current Time stamp (s)
        # }6i14f4d
        """
        # if runnedTime<0:
        #     runnedTime = time.time()-self.startTime
        
        PosE=self.fillList(PosE,3)
        AngEuler=self.fillList(AngEuler,3)
        VelE=self.fillList(VelE,3)
        PWMs=self.fillList(PWMs,8)
        
        PosGpsInt=[0,0,0]
        checkSum = 1234567891
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack(
            "6i14f4d",
            checkSum,
            copterID,
            vehicleType,
            *PosGpsInt,
            *PWMs,
            *VelE,
            *AngEuler,
            *PosE,
            runnedTime
        )
        # print(len(buf))
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send
        # print('Message Send')


    # //输出到CopterSim DLL模型的SILints和SILFloats数据
    # struct PX4SILIntFloat{
    #     int checksum;//1234567897
    #     int CopterID;
    #     int inSILInts[8];
    #     float inSILFLoats[20];
    # };
    #struct.pack 10i20f

    ## @brief 初始化真实数据接收机制
    #  - @anchor InitTrueDataLoop
    #  @param 无
    #  @return 无
    def InitTrueDataLoop(self):
        """ Initialize UDP True data listen loop from CopterSim through 30100 series ports
        """
        if self.isRedis:
            self.pubsub.subscribe(self.redisKey[RedisKey.SIL_TRUE_RECV])
        else:
            self.udp_socketTrue.bind(('0.0.0.0', self.port+1+10000))
        self.stopFlagTrueData=False
        self.tTrue = threading.Thread(target=self.getTrueDataMsg, args=())
        self.tTrue.start()

        self.udp_socketPX4.bind(('0.0.0.0', self.port+1+20000))
        self.stopFlagPX4Data=False
        self.tPX4 = threading.Thread(target=self.getPX4DataMsg, args=())
        self.tPX4.start()

    ## @brief 结束真实数据接收的循环
    #  - @anchor EndTrueDataLoop
    #  @param 无
    #  @return 无
    def EndTrueDataLoop(self):
        """ End the true data mode
        """
        self.stopFlagTrueData=True
        time.sleep(0.5)
        self.tTrue.join()
        self.hasTrueDataRec=False

        if self.Com[0:5]=='redis' or self.Com[0:5]=='Redis' or self.Com[0:5]=='REDIS':
            if self.isRedis:
                self.udp_socketTrue.close()
            else:
                self.redisCon.close()
        else:
            self.udp_socketTrue.close()

        time.sleep(0.5)
        self.stopFlagPX4Data=True
        # join需要加一个参数，否则会一直阻塞在此
        self.tPX4.join(1)
        self.udp_socketPX4.close()

    ## @brief 设置和准备发送无人机的位置信息和控制参数
    #  - @anchor SendOffAll
    #  @param type_mask 类型掩码
    #  @param coordinate_frame（默认值为1） 坐标系
    #  @param pos（默认值为[0, 0, 0]） 位置
    #  @param vel（默认值为[0, 0, 0]） 速度
    #  @param acc（默认值为[0, 0, 0]） 加速度
    #  @param yaw（默认值为0） 偏航角
    #  @param yawrate（默认值为0） 偏航率
    #  @return 无
    def SendOffAll(self, type_mask=PosTypeMask(), coordinate_frame=1, pos=[0, 0, 0], vel=[0, 0, 0], acc=[0, 0, 0], yaw=0, yawrate=0):
        
        pos=self.fillList(pos,3)
        vel=self.fillList(vel,3)
        acc=self.fillList(acc,3)
        
        self.type_mask = type_mask.mask()
        self.coordinate_frame = coordinate_frame
        self.pos = [pos[0], pos[1], pos[2]]
        self.vel = [vel[0], vel[1], vel[2]]
        self.acc = [acc[0], acc[1], acc[2]]
        self.yaw = yaw
        self.yawrate = yawrate

    ## @brief 结束真实数据接收的循环
    #  - @anchor SendAttAll
    #  @param type_mask 类型掩码
    #  @param q（默认值为[0, 0, 0]） 四元数
    #  @param body_rate（默认值为[0, 0, 0]） 机体速率列表，表示机体各个轴向的旋转速率
    #  @param thrust（默认值为0） 推力
    #  @return 无
    def SendAttAll(self, type_mask=AttTypeMask(), q=[1, 0, 0, 0], body_rate=[0,0,0], thrust=0):
        q=self.fillList(q,4)
        body_rate=self.fillList(body_rate,3)
        
        self.offMode = 2
        self.type_mask = type_mask.mask()
        self.yawrate = q[0]
        self.pos[0] = q[1]
        self.pos[1] = q[2]
        self.pos[2] = q[3]
        self.yaw = thrust
        self.vel = [body_rate[0], body_rate[1], body_rate[2]]

    ## @brief 初始化 MAVLink 通信循环，并与coptersim进行通信
    #  - @anchor InitMavLoop
    #  @param UDPMode（默认值为2） UDP连接模式
    #  - 0表示UDP_Full Mode
    #  - 1表示UDP_Simple Mode
    #  - 2表示MAVLink_Full Mode  
    #  - 3表示MAVLink_Simple Mode
    #  - 4表示MAVLink_NoSend Mode
    #  - 5表示MAVLink_NoGPS Mode
    #  - 6表示MAVLink_NoVision Mode
    #  @return 无
    def InitMavLoop(self,UDPMode=2):
        """ Initialize MAVLink listen loop from CopterSim
            0 and 1 for UDP_Full and UDP_Simple Modes, 2 and 3 for MAVLink_Full and MAVLink_Simple modes, 4 for MAVLink_NoSend, 5 for MAVLink_NoGPS, 6 for MAVLink_NoVision
            The default mode is 2 i.e., MAVLink_Full
        """

        # 这里匹配UDP模式
        if UDPMode>=7:
            if not self.isRedis:
                print('Error')
                # 这里要报错
                return
            else:
                UDPMode=UDPMode-7

        if UDPMode==4 or UDPMode==5 or UDPMode==6:
            UDPMode=2

        self.UDPMode=UDPMode
        if UDPMode>1.5: # UDPMode should lisen to PX4
            if self.isCom:
                self.the_connection = mavutil.mavlink_connection(self.ComName,self.baud)
            else:
                if self.isRealFly:
                    self.the_connection = mavutil.mavlink_connection('udpout:' + self.ip + ':'+str(self.port))

                    time.sleep(0.5)
                    # 发送心跳消息
                    self.the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    time.sleep(1)
                    print("heartbeat has been sent")
                    self.the_connection.wait_heartbeat()
                    print("init success")

                else:
                    self.the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:'+str(self.port+1))
                    # MAVLINK Redis模式下，上面函数要保留
                    # redis 也要收
                    if self.isRedis:
                        self.redisCon = redis.StrictRedis(host=self.redisHost, port=self.redisPort, db=0,
                                                        password=self.redisPass)
                        self.pubsub = self.redisCon.pubsub()
                        self.pubsub.subscribe(self.redisKey[RedisKey.SIL_RECV])
                        # 再启动一个线程，单独接收Redis Mavlink消息
                        # 收到redis包之后，立刻转发buf到127.0.0.1 , self.port+1

        else:
            if not self.isCom:
                if not self.isRedis:
                    self.udp_socketUDP.bind(('0.0.0.0', self.port+1))
                else:
                    self.redisCon = redis.StrictRedis(host=self.redisHost, port=self.redisPort, db=0,
                                                      password=self.redisPass)
                    self.pubsub = self.redisCon.pubsub()
                    self.pubsub.subscribe(self.redisKey[RedisKey.SIL_RECV])
        self.lastTime = 0
        self.t1 = threading.Thread(target=self.getMavMsg, args=())
        self.t1.start()
        
        self.netEvent.clear()
        self.t2 = threading.Thread(target=self.OffboardSendMode, args=())
        self.startTime = time.time()
        self.lastTime2 = 0
        self.startTime2 = time.time()
        self.isFailsafeEn = False
        self.SendMavArm(False) # 发送一条上锁命令给CopterSim

    ## @brief 结束通信
    #  - @anchor endMavLoop
    #  @param 无
    #  @return 无
    def endMavLoop(self):
        """ The same as stopRun(), stop message listenning from 20100 or serial port
        """
        self.stopRun()
      
    ## @brief 发送视觉定位数据给 MAVLink 连接
    #  - @anchor SendVisionPosition
    #  @param x 飞机位置 x 坐标
    #  @param y 飞机位置 y 坐标
    #  @param z 飞机位置 z 坐标
    #  @param yaw 偏航角
    #  @return 无
    def SendVisionPosition(self, x, y, z, yaw):
        """
        x,y,z 是动捕或视觉测量的飞机位置，yaw是动捕或视觉测量的偏航角
        """
        time_boot_ms = int((time.time() - self.startTime) * 1000)
        self.the_connection.mav.vision_position_estimate_send(time_boot_ms, x, y, z, 0, 0, yaw)
        
    ## @brief 发送GPS仿真数据给 MAVLink 连接
    #  - @anchor SendHILGps
    #  @param lat 飞机GPS坐标纬度值，单位度
    #  @param lon 飞机GPS坐标经度值，单位度
    #  @param alt 飞机GPS坐标高度值，单位米
    #  @param vel 飞机速度大小，单位m/s
    #  @param vn 飞机朝北飞速度大小，单位m/s
    #  @param ve 飞机朝东飞速度大小，单位m/s
    #  @param vd 飞机朝下飞速度大小，单位m/s
    #  @param yaw 飞机当前机头偏航角朝向，单位度
    #  @param cog 飞行速度航向角，单位度
    #  @time_usec 当前的时间戳，UNIX时间
    #  @param eph 飞机水平定位精度指标，范围0~1，无单位
    #  @param epv 飞机垂直定位精度指，范围0~1，无单位
    #  @param fix_type GPS当前状态，0-1: no fix, 2: 2D fix, 3: 3D fix.
    #  @param satellites_visible 当前卫星数量
    #  @return 无
    def SendHILGps(self,lat=40.1540302,lon=116.2593683,alt=50,vel=0,vn=0,ve=0,vd=0,cog=0,time_usec=-1,eph=0.3,epv=0.4,fix_type=3,satellites_visible=15):
        lat=int(lat*10000000) # [degE7] Latitude (WGS84)
        lon=int(lon*10000000) # [degE7] Longitude (WGS84)
        alt=int(alt*1000) # [mm] Altitude (MSL). Positive for up.
        eph = int(eph*100) # GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
        epv = int(epv*100) # GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
        vel = int(vel*100) # [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
        vn = int(vn*100) # [cm/s] GPS velocity in north direction in earth-fixed NED frame
        ve = int(ve*100) # [cm/s] GPS velocity in east direction in earth-fixed NED frame
        vd = int(vd*100) # [cm/s] GPS velocity in down direction in earth-fixed NED frame
        if cog<0: # 如果为负值，则加上360，变为正值
            cog = cog+360
        cog = int(cog*100) # [cdeg] (deg*100) Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        fix_type = int(fix_type) # 0-1: no fix, 2: 2D fix, 3: 3D fix.
        satellites_visible = int(satellites_visible) # Number of satellites visible.
        #id=int(id) # GPS ID (zero indexed). Used for multiple GPS inputs
        # if yaw>180: # 如果超过180度，换成负偏航
        #     yaw = yaw-360
        # yaw = int(yaw*100) # [cdeg] Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
        
        time_usec=int(time_usec)
        if time_usec<0 :
            time_usec=int(time.time()*1000000)
        
        self.the_connection.mav.hil_gps_send(time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible)


    ## @brief 发送数据到redis
    #  - @anchor SendRedisData
    #  @param key redis的密钥
    #  @param buf 发送的数据
    #  @return 无
    def SendRedisData(self, key, buf):
        if key not in self.redisKey:
            print("Error: Failed to send data to redis, No redis key: %s" % key)
            return

        self.redisCon.publish(self.redisKey[key], buf)

    ## @brief 发送数据
    #  - @anchor SendBuf
    #  @param buf 发送的数据
    #  @return 无
    def SendBuf(self, buf):
        if self.isRedis:
            self.SendRedisData(RedisKey.SIL, buf)
        else:
            # if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
            #     self.udp_socket.sendto(buf, ('127.0.0.1', self.port))
            self.udp_socket.sendto(buf, (self.ip, self.port))

    ## @brief 发送数据
    #  - @anchor SendBufTrue
    #  @param buf 发送的数据
    #  @param port（默认值为None） 端口号
    #  @return 无
    def SendBufTrue(self, buf, port=None):
        if port is None:
            port = self.port + 10000
        if self.isRedis:
            self.SendRedisData(RedisKey.SIL_TRUE, buf)
        else:
            # if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
            #     self.udp_socket.sendto(buf, ('127.0.0.1', port))
            self.udp_socket.sendto(buf, (self.ip, port))

    # saturation function
    ## @brief 饱和操作的函数，确保输入值在正负阈值内
    #  - @anchor sat
    #  @param inPwm（默认值为0） 输入值
    #  @param thres（默认值为1） 阈值
    #  @return 输出值
    def sat(self,inPwm=0,thres=1):
        """Saturation function for value inPwm with range thres
        if inPwm>thres, then inPwm=thres
        if inPwm<-thres,then inPwm=-thres
        """
        outPwm= inPwm
        if inPwm>thres:
            outPwm = thres
        elif inPwm<-thres:
            outPwm = -thres
        return outPwm

    # send MAVLink command long message to Pixhawk (routed through CopterSim)
    ## @brief 将 MAVLink 的长指令消息发送到 Pixhawk 系统
    #  - @anchor SendMavCmdLong
    #  @param command 消息
    #  @param param1~param7（默认值均为0） 参数1~参数7
    #  @return 无
    def SendMavCmdLong(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        """ Send command long message to PX4, the mavlink command definitions can be found at
        https://mavlink.io/en/messages/common.html#COMMAND_LONG
        https://mavlink.io/en/messages/common.html#MAV_CMD
        """
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom or self.isRealFly:
            self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component,
                                            command, 0,
                                            param1, param2, param3, param4, param5, param6, param7)
        else:
            buf = self.mav0.command_long_encode(self.the_connection.target_system, self.the_connection.target_component,
                                                command, 0,
                                                param1, param2, param3, param4, param5, param6, param7).pack(self.mav0)
            self.SendBuf(buf)


           #self.udp_socket.sendto(buf, (self.ip, self.port))

    ## @brief 将 MAVLink 的长指令消息发送到 QGC
    #  - @anchor SendQgcCmdLong
    #  @param command 消息
    #  @param param1~param7（默认值均为0） 命令相关参数
    #  @return 无
    def SendQgcCmdLong(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
            buf = self.mav0.command_long_encode(255, 0,
                                                command, 0,
                                                param1, param2, param3, param4, param5, param6, param7).pack(self.mav0)
            self.udp_socket.sendto(buf, (self.ip, 18570))

    # send command to make Pixhawk enter Offboard mode
    ## @brief 发送Offboard命令到PX4
    #  - @anchor sendMavOffboardCmd
    #  @param type_mask 掩码类型
    #  @param coordinate_frame 坐标轴类型
    #  @param x x坐标
    #  @param y y坐标
    #  @param z z坐标
    #  @param vx x方向的速度
    #  @param vy y方向的速度
    #  @param vz z方向的速度
    #  @param afx x方向的加速度
    #  @param afy y方向的加速度
    #  @param afz z方向的加速度
    #  @param yaw 偏航角
    #  @param yaw_rate 偏航角速率
    #  @return 无
    def sendMavOffboardCmd(self,type_mask,coordinate_frame, x,  y,  z,  vx,  vy,  vz,  afx,  afy,  afz,  yaw, yaw_rate):
        """ send offboard command to PX4, the definition of the mavlink message can be found at
        https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        """
        time_boot_ms = int((time.time()-self.startTime)*1000)
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom or self.isRealFly:
            self.the_connection.mav.set_position_target_local_ned_send(time_boot_ms,self.the_connection.target_system,
                                                                    self.the_connection.target_component,
                                                                    coordinate_frame,type_mask,x,  y,  z,  vx,  vy,  vz,  afx,
                                                                    afy,  afz,  yaw, yaw_rate)
        else:
            buf = self.mav0.set_position_target_local_ned_encode(time_boot_ms,self.the_connection.target_system,
                                                                    self.the_connection.target_component,
                                                                    coordinate_frame,type_mask,x,  y,  z,  vx,  vy,  vz,  afx,
                                                                    afy,  afz,  yaw, yaw_rate).pack(self.mav0)
            self.SendBuf(buf)
            #self.udp_socket.sendto(buf, (self.ip, self.port))

    ## @brief 获取offboard消息的位图
    #  - @anchor TypeMask
    #  @param EnList 包含位姿，速度，加速度，推力，偏航角，偏航速率的列表
    #  @return 位图转化后的int值
    def TypeMask(self,EnList):
        """ Obtain the bitmap for offboard message
        https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
        """
        enPos = EnList[0]
        enVel = EnList[1]
        enAcc = EnList[2]
        enForce = EnList[3]
        enYaw = EnList[4]
        EnYawrate= EnList[5]
        y=int(0)
        if not enPos:
            y = y | 7

        if not enVel:
            y = y | (7<<3)

        if not enAcc:
            y = y | (7<<6)

        if not enForce:
            y = y | (1<<9)

        if not enYaw:
            y = y | (1<<10)

        if not EnYawrate:
            y = y|(1<<11)
        type_mask = y
        return int(type_mask)

    # set the control sigals for Offboard sending loop
    ## @brief 发送Offboard命令到PX4
    #  - @anchor sendMavOffboardAPI
    #  @param type_mask（默认值为0） 掩码类型
    #  @param coordinate_frame（默认值为0） 坐标轴类型
    #  @param pos（默认值为[0,0,0]） 位置
    #  @param vel（默认值为[0,0,0]） 速度
    #  @param acc（默认值为[0,0,0]） 加速度
    #  @param yaw（默认值为0） 偏航角
    #  @param yaw_rate（默认值为0） 偏航角速率
    #  @return 无
    def sendMavOffboardAPI(self,type_mask=0,coordinate_frame=0,pos=[0,0,0],vel=[0,0,0],acc=[0,0,0],yaw=0,yawrate=0):
        """send offboard command to PX4, the definition of the mavlink message can be found at
        https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        """
        
        if self.isInPointMode:
            return
        time_boot_ms = int((time.time()-self.startTime)*1000)
        if self.isCom or self.isRealFly:

            if self.offMode==0:
                self.the_connection.mav.set_position_target_local_ned_send(int(time_boot_ms),self.the_connection.target_system,
                                                                    self.the_connection.target_component,
                                                                    coordinate_frame,type_mask,pos[0],  pos[1],  pos[2],
                                                                    vel[0],  vel[1],  vel[2],  acc[0],
                                                                    acc[1],  acc[2],  yaw, yawrate)
            elif self.offMode==1:
                lat_int=int(pos[0]*1e7)
                lon_int=int(pos[1]*1e7)
                self.the_connection.mav.set_position_target_global_int_send(int(time_boot_ms),self.the_connection.target_system,
                                                                    self.the_connection.target_component,
                                                                    coordinate_frame,type_mask,lat_int,  lon_int,  pos[2],
                                                                    vel[0],  vel[1],  vel[2],  acc[0],
                                                                    acc[1],  acc[2],  yaw, yawrate)
            elif self.offMode==2:
                self.the_connection.mav.set_attitude_target_send(int(time_boot_ms),self.the_connection.target_system,
                                                                        self.the_connection.target_component,type_mask,
                                                                        [yawrate,pos[0],pos[1],pos[2]],vel[0],  vel[1], vel[2],yaw)

            elif self.offMode==3:

                desiredAlt  = yaw
                currAlt = self.uavPosNED[2]
                dAlt = (desiredAlt -currAlt)/5.0
                dAlt = self.sat(dAlt,0.5)
                self.zInt = self.zInt - dAlt*0.001

                if self.zInt >0.8:
                    self.zInt=0.8

                if self.zInt<0:
                    self.zInt=0

                dvel = self.uavVelNED[2]/2.0
                dvel = self.sat(dvel,0.5)

                dAlt = -dAlt*1 + dvel*1

                dAlt = self.sat(dAlt,0.5)

                dThr = self.zInt + dAlt

                if dThr>1:
                    dThr=1

                if dThr<0:
                    dThr=0

                self.the_connection.mav.set_attitude_target_send(int(time_boot_ms),self.the_connection.target_system,
                                                                        self.the_connection.target_component,type_mask,
                                                                        [yawrate,pos[0],pos[1],pos[2]],vel[0],  vel[1], vel[2],dThr)

        else:
            if self.UDPMode>1.5:
                if self.offMode==0:
                    buf = self.mav0.set_position_target_local_ned_encode(int(time_boot_ms),self.the_connection.target_system,
                                                                        self.the_connection.target_component,
                                                                        coordinate_frame,type_mask,pos[0],  pos[1],  pos[2],
                                                                        vel[0],  vel[1],  vel[2],  acc[0],
                                                                        acc[1],  acc[2],  yaw, yawrate).pack(self.mav0)
                elif self.offMode==1:
                    lat_int=int(pos[0]*1e7)
                    lon_int=int(pos[1]*1e7)
                    buf = self.mav0.set_position_target_global_int_encode(int(time_boot_ms),self.the_connection.target_system,
                                                                        self.the_connection.target_component,
                                                                        coordinate_frame,type_mask,lat_int,  lon_int,  pos[2],
                                                                        vel[0],  vel[1],  vel[2],  acc[0],
                                                                        acc[1],  acc[2],  yaw, yawrate).pack(self.mav0)
                elif self.offMode==2:
                    buf = self.mav0.set_attitude_target_encode(int(time_boot_ms),self.the_connection.target_system,
                                                                        self.the_connection.target_component,type_mask,
                                                                        [yawrate,pos[0],pos[1],pos[2]],vel[0],  vel[1], vel[2],yaw).pack(self.mav0)
                elif self.offMode==3:

                    desiredAlt  = yaw
                    currAlt = self.uavPosNED[2]
                    dAlt = (desiredAlt -currAlt)/5.0
                    dAlt = self.sat(dAlt,0.5)
                    self.zInt = self.zInt - dAlt*0.001

                    if self.zInt >0.8:
                        self.zInt=0.8

                    if self.zInt<0:
                        self.zInt=0

                    dvel = self.uavVelNED[2]/2.0
                    dvel = self.sat(dvel,0.5)

                    dAlt = -dAlt*1 + dvel*1

                    dAlt = self.sat(dAlt,0.5)

                    dThr = self.zInt + dAlt

                    if dThr>1:
                        dThr=1

                    if dThr<0:
                        dThr=0

                    buf = self.mav0.set_attitude_target_encode(int(time_boot_ms),self.the_connection.target_system,
                                                                        self.the_connection.target_component,type_mask,
                                                                        [yawrate,pos[0],pos[1],pos[2]],vel[0],  vel[1], vel[2],dThr).pack(self.mav0)
                self.SendBuf(buf)
                #self.udp_socket.sendto(buf, (self.ip, self.port))
            else:
                # UDP_Full Mode
                if self.UDPMode==0:
                    # struct inHILCMDData{
                    #     uint32_t time_boot_ms;
                    #     uint32_t copterID;
                    #     uint32_t modes;
                    #     uint32_t flags;
                    #     float ctrls[16];
                    # };
                    # typedef struct _netDataShortShort {
                    #     TargetType tg;
                    #     int        len;
                    #     char       payload[PAYLOAD_LEN_SHORT_SHORT];
                    # }netDataShortShort;
                    ctrls=pos+vel+acc+[yaw,yawrate]+[0,0,0,0,0]
                    buf0 = struct.pack("4I16f",time_boot_ms,self.CopterID,type_mask,coordinate_frame,*ctrls)
                    # buf for remaining 192-152=40bytes of payload[192] of netDataShort
                    buf1 = bytes([0]*(112-len(buf0)))
                    # buf for tg and len in netDataShort
                    buf2 = struct.pack("2i",3,len(buf0))
                    # buf for netDataShort
                    buf=buf2+buf0+buf1
                    self.SendBuf(buf)
                else: # Simple Mode

                    if self.ctrlMode==0 or self.ctrlMode==1:
                        ctrls=vel+[yawrate]
                    elif self.ctrlMode==2 or self.ctrlMode==3:
                        ctrls=pos+[yaw]
                    elif self.ctrlMode==8 or self.ctrlMode==7 or self.ctrlMode==6:
                        if self.ctrlMode==6:
                            ctrls=acc+[0]
                        elif self.ctrlMode==7:
                            ctrls=acc+[yawrate]
                        else:
                            ctrls=acc+[yaw]

                    elif self.offMode == 13: # 如果是速度高度偏航固定翼模式
                        ctrls=[vel[1],pos[2],vel[0],0]
                    # else:
                    #     ctrlMode=0
                    #     ctrls=vel+[yawrate]

                    #     if self.EnList[0]==1 and coordinate_frame == 8: # POS, MAV_FRAME_BODY_NED
                    #         ctrlMode=3
                    #         ctrls=pos+[yaw]
                    #     elif self.EnList[0]==0 and coordinate_frame == 8: # Vel, MAV_FRAME_BODY_NED
                    #         ctrlMode=1
                    #         ctrls=vel+[yawrate]
                    #         #print(ctrlMode,ctrls)
                    #     elif self.EnList[0]==1 and coordinate_frame == 1: # POS, MAV_FRAME_LOCAL_NED
                    #         ctrlMode=2
                    #         ctrls=pos+[yaw]
                    #     else: # Vel, MAV_FRAME_LOCAL_NED
                    #         ctrlMode=0
                    #         ctrls=vel+[yawrate]
                    self.sendUDPSimpData(self.ctrlMode,ctrls)
                    #print(ctrlMode,ctrls)

    # struct inOffboardShortData{
    #     int checksum;
    #     int ctrlMode;
    #     float controls[4];
    # };
    ## @brief 发送UDP简单数据
    #  - @anchor sendUDPSimpData
    #  @param ctrlMode 控制模式
    #  @param ctrls 控制信号
    #  @return 无
    def sendUDPSimpData(self,ctrlMode,ctrls):
        ctrls=self.fillList(ctrls,4)
        checksum=1234567890
        buf = struct.pack("2i4f",checksum,ctrlMode,ctrls[0],ctrls[1],ctrls[2],ctrls[3])

        self.SendBuf(buf)

    # struct PX4UorbRflyCtrl {
    #     int checksum; #1234567896
    #     int CopterID;
    #     uint32_t modes;
    #     uint32_t flags;
    #     float data[16];
    # }2i2I16f
    ## @brief 发送UDP简单数据
    #  - @anchor sendPX4UorbRflyCtrl
    #  @param data（默认值为[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]） 数据
    #  @param modes（默认值为1） 模式
    #  @param flags（默认值为1） 控制信号
    #  @return 无
    def sendPX4UorbRflyCtrl(self,data=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],modes=1,flags=1):
        data=self.fillList(data,16)
        checksum=1234567896
        buf = struct.pack("2i2I16f",checksum,self.CopterID,modes,flags,*data)

        self.SendBufTrue(buf)
        #self.udp_socket.sendto(buf, (self.ip, self.port+10000))

    # send velocity control signal in earth north-east-down (NED) frame to Pixhawk
    ## @brief 北东地坐标系下操控飞机
    #  - @anchor SendVelNED
    #  @param vx（默认值为0） x方向的速度
    #  @param vy（默认值为0） y方向的速度
    #  @param vz（默认值为0） z方向的速度
    #  @param yawrate（默认值为0） 偏航角速度
    #  @return 无
    def SendVelNED(self,vx=math.nan,vy=math.nan,vz=math.nan,yawrate=math.nan):
        """ Send targe vehicle speed (m/s) to PX4 in the earth north-east-down (NED) frame with yawrate (rad/s)
        when the vehicle fly upward, the vz < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.ctrlMode=0 #地球速度控制模式
        self.EnList = [0,1,0,0,0,1]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate

    # send velocity control signal in earth north-east-down (NED) frame to Pixhawk
    ## @brief 北东地坐标系下操控飞机
    #  - @anchor SendVelNEDNoYaw
    #  @param vx x方向的速度
    #  @param vy y方向的速度
    #  @param vz z方向的速度
    #  @return 无
    def SendVelNEDNoYaw(self,vx,vy,vz):
        """ Send targe vehicle speed (m/s) to PX4 in the earth north-east-down (NED) frame without yaw control
        when the vehicle fly upward, the vz < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.ctrlMode=0 #地球速度控制模式
        self.EnList = [0,1,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = 0

    # send velocity control signal in body front-right-down (FRD) frame
    ## @brief 前右下坐标系下操控飞机
    #  - @anchor SendVelFRD
    #  @param vx（默认值为NaN，表示不控制本通道） x方向的速度
    #  @param vy（默认值为NaN，表示不控制本通道） y方向的速度
    #  @param vz（默认值为NaN，表示不控制本通道） z方向的速度
    #  @param yawrate（默认值为NaN，表示不控制本通道） 偏航角速率
    #  @return 无
    def SendVelFRD(self,vx=math.nan,vy=math.nan,vz=math.nan,yawrate=math.nan):
        """ Send vehicle targe speed (m/s) to PX4 in the body forward-rightward-downward (FRD) frame with yawrate control (rad/s)
        when the vehicle fly upward, the vz < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.ctrlMode=1 #机体速度控制模式
        self.EnList = [0,1,0,0,0,1]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate

    # send velocity control signal in body front-right-down (FRD) frame
    ## @brief 前右下坐标系以目标姿态控制飞机
    #  - @anchor SendAttPX4
    #  @param att（默认值为 [0,0,0,0]） 飞行器姿态
    #  @param thrust（默认值为 0.5） 推力值
    #  @param CtrlFlag（默认值为 0） 姿态控制模式
    #  - 0表示 att 是一个三维向量，表示欧拉角，滚转、俯仰、偏航，单位为度
    #  - 1表示 att 是一个三维向量，表示欧拉角，滚转、俯仰、偏航，单位为弧度
    #  - 2表示 att 是一个四维向量，表示四元数
    #  - 3表示 att 是一个三维向量，表示旋转速率，滚转、俯仰、偏航，单位为弧度/秒
    #  - 4表示 att 是一个三维向量，表示旋转速率，滚转、俯仰、偏航，单位为度/秒
    #  @param AltFlg（默认值为 0） 偏航角速率
    #  - 0表示 thrust 被定义为 "集体推力，归一化到 0 .. 1（对于能够反向推力的飞行器则为 -1 .. 1）"
    #  - 1表示 thrust 为期望的高度
    #  @return 无
    def SendAttPX4(self,att=[0,0,0,0],thrust=0.5,CtrlFlag=0,AltFlg=0):
        """ Send vehicle targe attitude to PX4 in the body forward-rightward-downward (FRD) frame
        """
        #CtrlFlag is a flag to determine the definition of input att
        #CtrlFlag 0 : att is a 3D vector for euler angles, roll,pitch,yaw, unit is degree
        #CtrlFlag 1 : att is a 3D vector for euler angles, roll,pitch,yaw, unit is rad
        #CtrlFlag 2 : att is a 4D vector for quaternion
        #CtrlFlag 3 : att is a 3D vector for rotation rate, roll,pitch,yaw, unit is rad/s
        #CtrlFlag 4 : att is a 3D vector for rotation rate, roll,pitch,yaw, unit is degree/s

        # if AltFlg ==0
        # thrust is defined as "Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)" from PX4 web
        # if AltFlg > 0
        # thrust is the desired Altitude

        # https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
        
        att=self.fillList(att,4)
        
        self.offMode=2 # SET_ATTITUDE_TARGET
        self.ctrlMode=4 #姿态控制模式
        self.EnList = [0,0,0,0,0,0]
        if self.uavThrust<0.5:
            self.zInt=0.5
        else:
            self.zInt= self.uavThrust
        if AltFlg>0.5:
            self.offMode=3 # SET_ATTITUDE_TARGET, with desired altitude

        y=int(0)
        if CtrlFlag<3: # Ignore body rate
            y = y | 7
        else:
            y = y | (1<<7) # Ignore body attitude

        self.type_mask=y

        quat=[1,0,0,0] #Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        if CtrlFlag==0 or CtrlFlag==1:
            roll=att[0]
            pitch=att[1]
            yaw=att[2]
            if CtrlFlag==0: # convert to unit rad
                roll=att[0]/180.0*math.pi
                pitch=att[1]/180.0*math.pi
                yaw=att[2]/180.0*math.pi

            sp2=math.sin(pitch/2)
            sy2=math.sin(yaw/2)
            cr2=math.cos(roll/2)
            cp2=math.cos(pitch/2)
            cy2=math.cos(yaw/2)
            sr2=math.sin(roll/2)

            qx=sp2*sy2*cr2+cp2*cy2*sr2
            qy=sp2*cy2*cr2+cp2*sy2*sr2
            qz=cp2*sy2*cr2-sp2*cy2*sr2
            qw=cp2*cy2*cr2-sp2*sy2*sr2
            quat=[qw,qx,qy,qz]
        if CtrlFlag==2:
            quat=att

        self.pos=[quat[1],quat[2],quat[3]]
        self.yawrate = quat[0]

        rate=[0,0,0]
        if CtrlFlag==3:
            rate=[att[0],att[1],att[2]]
        if CtrlFlag==4:
            rate=[att[0]/180.0*math.pi,att[1]/180.0*math.pi,att[2]/180.0*math.pi]
        self.vel = rate
        self.acc = [0, 0, 0]
        self.yaw = thrust

    ## @brief 将欧拉角转化为四元数
    #  - @anchor EulerToQuat
    #  @param Euler 欧拉角
    #  @return 四元数
    def EulerToQuat(self,Euler):
        roll=Euler[0]
        pitch=Euler[1]
        yaw=Euler[2]
        sp2=math.sin(pitch/2)
        sy2=math.sin(yaw/2)
        cr2=math.cos(roll/2)
        cp2=math.cos(pitch/2)
        cy2=math.cos(yaw/2)
        sr2=math.sin(roll/2)

        qx=sp2*sy2*cr2+cp2*cy2*sr2
        qy=sp2*cy2*cr2+cp2*sy2*sr2
        qz=cp2*sy2*cr2-sp2*cy2*sr2
        qw=cp2*cy2*cr2-sp2*sy2*sr2
        quat=[qw,qx,qy,qz]

        return quat

    ## @brief 发送目标加速度给PX4
    #  - @anchor SendAccPX4
    #  @param afx（默认值为NaN，表示不控制本通道） x方向的加速度
    #  @param afy（默认值为NaN，表示不控制本通道） y方向的加速度
    #  @param afz（默认值为NaN，表示不控制本通道） z方向的加速度
    #  @param yawValue（默认值为 0） 偏航角速度
    #  @param yawType（默认值为 0） 姿态控制模式
    #  - 0表示没有无偏航和偏航速率
    #  - 1表示偏航角控制
    #  - 2表示偏航速率控制
    #  @param frameType（默认值为 0） 偏航角速率
    #  - 0表示地球 NED（北东地）坐标系
    #  - 1表示机体 FRD（前右下）坐标系
    #  @return 无
    def SendAccPX4(self,afx=math.nan,afy=math.nan,afz=math.nan,yawValue=math.nan,yawType=0,frameType=0):
        """ Send a targe acceleration (m/s^2) to PX4
        """
        # From PX4 Web: Acceleration setpoint values are mapped to create a normalized
        # thrust setpoint (i.e. acceleration setpoints are not "properly" supported).
        # yawType 0: No yaw and yawrate
        # yawType 1: Yaw control
        # yawType 2: yaw Rate Ctrl
        # frameType 0: earth NED frame
        # frameType 1: body forward-rightward-downward (FRD) frame
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.ctrlMode=8 #加速度控制模式
        if yawType==0:
            self.EnList = [0,0,1,1,0,0]
            self.yaw = 0
            self.yawrate = 0
            self.ctrlMode=6 #加速度控制模式，无偏航控制
        elif yawType==1:
            self.EnList = [0,0,1,1,1,0]
            self.yaw = yawValue
            self.yawrate = 0
            self.ctrlMode=7 #加速度控制模式，偏航角控制
        else:
            self.EnList = [0,0,1,1,0,1]
            self.yaw = 0
            self.yawrate = yawValue
            self.ctrlMode=8 #加速度控制模式，偏航角速度控制

        self.type_mask=self.TypeMask(self.EnList)
        if frameType==0:
            self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        else:
            self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [afx, afy, afz]

    # send velocity control signal in body front-right-down (FRD) frame
    ## @brief 机体前右下坐标系下控制飞机
    #  - @anchor SendVelNoYaw
    #  @param vx（默认值为NaN，表示不控制本通道） x方向的速度
    #  @param vy（默认值为NaN，表示不控制本通道） y方向的速度
    #  @param vz（默认值为NaN，表示不控制本通道） z方向的速度
    #  @return 无
    def SendVelNoYaw(self,vx=math.nan,vy=math.nan,vz=math.nan):
        """ Send vehicle targe speed (m/s) to PX4 in the body forward-rightward-downward (FRD) frame without yawrate control (rad)
        when the vehicle fly upward, the vz < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.ctrlMode=1 #机体速度控制模式
        self.EnList = [0,1,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = 0

    # send target position in earth NED frame
    ## @brief 地球北东地坐标系下控制飞机到指定位置
    #  - @anchor SendVelNoYaw
    #  @param x（默认值为NaN，表示不控制本通道） x方向的位置
    #  @param y（默认值为NaN，表示不控制本通道） y方向的位置
    #  @param z（默认值为NaN，表示不控制本通道） z方向的位置
    #  @return 无
    def SendPosNED(self,x=math.nan,y=math.nan,z=math.nan,yaw=math.nan):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.ctrlMode=2 #地球位置控制模式
        self.EnList = [1,0,0,0,1,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = yaw

    # send target position in earth NED frame
    ## @brief 地球北东地坐标系下控制飞机
    #  - @anchor SendVelNoYaw
    #  @param vel（默认值为 10） 目标速度
    #  @param yaw（默认值为 6.28） 偏航角
    #  @param alt（默认值为 -100） 目标高度
    #  @return 无
    def SendVelYawAlt(self,vel=10,yaw=6.28,alt=-100):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """

        if abs(yaw)<0.00001:
            yaw = 6.28
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.ctrlMode=13 #速度高度偏航控制模式
        self.type_mask=int("000111000000", 2)
        self.coordinate_frame = 1
        self.pos=[0,0,alt]
        self.vel = [yaw,vel,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = yaw

    # send target position in earth NED frame
    ## @brief 地球北东地坐标系下控制飞机到目标全球位置
    #  - @anchor SendPosGlobal
    #  @param lat（默认值为 0） 目标维度
    #  @param lon（默认值为 0） 目标经度
    #  @param alt（默认值为 0） 目标高度
    #  @param yawValue（默认值为 0） 目标偏航角或偏航速率
    #  @param yawType（默认值为 0） 控制类型
    #  - 0表示不控制偏航角和偏航速率
    #  - 1表示偏航角控制
    #  - 2表示偏航速率控制
    #  @return 无
    def SendPosGlobal(self,lat=0,lon=0,alt=0,yawValue=0,yawType=0):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        # yawType 0: No yaw and yawrate
        # yawType 1: Yaw control
        # yawType 2: yaw Rate Ctrl
        self.offMode=1 # SET_POSITION_TARGET_GLOBAL_INT
        #self.ctrlMode=2 #地球位置控制模式
        if yawType==0:
            self.EnList = [1,0,0,0,0,0]
            self.yaw = 0
            self.yawrate = 0
        elif yawType==1:
            self.EnList = [1,0,0,0,1,0]
            self.yaw = yawValue
            self.yawrate = 0
        else:
            self.EnList = [1,0,0,0,0,1]
            self.yaw = 0
            self.yawrate = yawValue
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_GLOBAL_INT
        self.pos=[lat,lon,alt]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]

    # send target position in earth NED frame
    ## @brief 地球北东地坐标系下控制飞机到目标位置
    #  - @anchor SendPosNEDNoYaw
    #  @param x（默认值为NaN，表示不控制本通道） x方向的位置
    #  @param y（默认值为NaN，表示不控制本通道） y方向的位置
    #  @param z（默认值为NaN，表示不控制本通道） z方向的位置
    #  @return 无
    def SendPosNEDNoYaw(self,x=math.nan,y=math.nan,z=math.nan):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame without yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.ctrlMode=2 #地球位置控制模式
        self.EnList = [1,0,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0

    # send target position in body FRD frame
    ## @brief 机体前右下坐标系下控制飞机到目标位置
    #  - @anchor SendPosFRD
    #  @param x（默认值为NaN，表示不控制本通道） x方向的位置
    #  @param y（默认值为NaN，表示不控制本通道） y方向的位置
    #  @param z（默认值为NaN，表示不控制本通道） z方向的位置
    #  @param yaw（默认值为NaN，表示不控制本通道） 偏航角
    #  @return 无
    def SendPosFRD(self,x=math.nan,y=math.nan,z=math.nan,yaw=math.nan):
        """ Send vehicle targe position (m) to PX4 in the body forward-rightward-downward (FRD) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.ctrlMode=3 #机体位置控制模式
        self.EnList = [1,0,0,0,1,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = yaw

    # send target position in body FRD frame
    ## @brief 机体前右下坐标系下控制飞机到目标位置
    #  - @anchor SendPosFRDNoYaw
    #  @param x（默认值为NaN，表示不控制本通道） x方向的位置
    #  @param y（默认值为NaN，表示不控制本通道） y方向的位置
    #  @param z（默认值为NaN，表示不控制本通道） z方向的位置
    #  @return 无   
    def SendPosFRDNoYaw(self,x=math.nan,y=math.nan,z=math.nan):
        """ Send vehicle targe position (m) to PX4 in the body forward-rightward-downward (FRD) frame without yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.ctrlMode=3 #机体位置控制模式
        self.EnList = [1,0,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0

    ## @brief 机体前右下坐标系下控制飞机到目标位置
    #  - @anchor SendPosNEDExt
    #  @param x（默认值为NaN，表示不控制本通道） x方向的位置
    #  @param y（默认值为NaN，表示不控制本通道） y方向的位置
    #  @param z（默认值为NaN，表示不控制本通道） z方向的位置
    #  @param mode（默认值为 3） 控制模式
    #  - 0表示滑行设定点，仅用于固定翼
    #  - 1表示起飞设定点，仅用于固定翼
    #  - 2表示降落设定点，仅用于固定翼
    #  - 3表示盘旋设定点，对于 ROVER: 盘旋设定点（飞机接近设定点时停止）；对于固定翼: 盘旋设定点（围绕设定点飞行一个圆圈）。
    #  - 4表示空闲设定点，仅用于固定翼
    #  @param isNED（默认值为 True） 是否为地球北东地模式
    #  - True表示地球北东地模式
    #  - False表示机体前右下模式
    #  @return 无   
    def SendPosNEDExt(self,x=math.nan,y=math.nan,z=math.nan,mode=3,isNED=True):
        """ Send vehicle targe position (m) to PX4
        when the vehicle fly above the ground, then z < 0
        """
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [1,0,0,0,1,0]
        self.type_mask=self.TypeMask(self.EnList)
        if mode==0:
            # Gliding setpoint
            self.type_mask=int(292) # only for fixed Wing
        elif mode==1:
            # Takeoff setpoints
            self.type_mask=int(4096) # only for fixed Wing
        elif mode==2:
            # Land setpoints
            self.type_mask=int(8192) # only for fixed Wing
        elif mode==3:
            # Loiter setpoints
            # for Rover:  Loiter setpoint (vehicle stops when close enough to setpoint).
            # for fixed wing:  Loiter setpoint (fly a circle centred on setpoint).
            self.type_mask=int(12288)
        elif mode==4:
            # Idle setpoint
            # only for fixed wing
            # Idle setpoint (zero throttle, zero roll / pitch).
            self.type_mask=int(16384)
        if isNED:
            self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        else:
            self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0

    ## @brief 发送指令使飞机起飞
    #  - @anchor enFixedWRWTO
    #  @param 无
    #  @return 无   
    def enFixedWRWTO(self):
        """ Send command to enable takeoff on Runway of the aircraft
        """
        self.sendMavSetParam('RWTO_TKOFF'.encode(),  1/7.13622e+44, mavlink2.MAV_PARAM_TYPE_INT32)

    ## @brief 发送指定改变飞机的巡航速度
    #  - @anchor SendCruiseSpeed
    #  @param Speed（默认值为 0） 速度
    #  @return 无   
    def SendCruiseSpeed(self,Speed=0):
        """ Send command to change the Cruise speed (m/s) of the aircraft
        """
        #def SendCruiseSpeed(self,Speed,Type=1,Throttle=-1,Relative=0):
        #  type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed); min:0 max:3 increment:1
        #  Speed (-1 indicates no change); min: -1; Unit: m/s
        #  Throttle (-1 indicates no change); min: -1; Unit: %
        #  Relative	0: absolute, 1: relative; min:0 max:1 increment:1
        #self.SendMavCmdLong(mavlink2.MAV_CMD_DO_CHANGE_SPEED, Type,Speed,Throttle,Relative,0,0,0)
        #self.sendMavSetParam('NAV_LOITER_RAD'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        if self.UDPMode>1.5:
            self.sendMavSetParam('FW_AIRSPD_TRIM'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        else:
            ctrls=[Speed,0,0,0]
            self.sendUDPSimpData(10,ctrls)

    ## @brief 发送设定多旋翼飞行器的最大速度
    #  - @anchor SendCopterSpeed
    #  @param Speed（默认值为 0） 速度
    #  @return 无  
    def SendCopterSpeed(self,Speed=0):
        """ send command to set the maximum speed of the multicopter
        """
        # 最小3，最大20，默认5
        self.sendMavSetParam('MPC_XY_VEL_MAX'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        self.MaxSpeed=Speed

    ## @brief 发送指定改变飞机的地面速度
    #  - @anchor SendGroundSpeed
    #  @param Speed（默认值为 0） 速度
    #  @return 无  
    def SendGroundSpeed(self,Speed=0):
        """ Send command to change the ground speed (m/s) of the aircraft
        """
        self.sendMavSetParam('GND_SPEED_TRIM'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        self.MaxSpeed=Speed

    ## @brief 发送指定改变飞机的巡航半径
    #  - @anchor SendCruiseRadius
    #  @param rad（默认值为 0） 半径
    #  @return 无  
    def SendCruiseRadius(self,rad=0):
        """ Send command to change the Cruise Radius (m) of the aircraft
        """
        if self.UDPMode>1.5:
            self.sendMavSetParam('NAV_LOITER_RAD'.encode(), rad, mavlink2.MAV_PARAM_TYPE_REAL32)
        else:
            ctrls=[0,rad,0,0]
            self.sendUDPSimpData(10,ctrls)

    ## @brief 发送指定使飞机起飞
    #  - @anchor sendTakeoffMode
    #  @param alt（默认值为 0） 高度
    #  @return 无  
    def sendTakeoffMode(self,alt=0):
        """ Send command to make the aircraft takeoff
        """
        if alt<-0.1:
            self.sendMavSetParam('MIS_TAKEOFF_ALT'.encode(), -alt, mavlink2.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.1)
            self.sendMavSetParam('MIS_TAKEOFF_ALT'.encode(), -alt, mavlink2.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.5)
        self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO,PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF)
        if not self.isArmed:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    # initialize Offboard in Pixhawk and start sending data loop in Python
    ## @brief 发送命令进入offboard模式，消息发送频率为30HZ
    #  - @anchor initOffboard
    #  @param 无
    #  @return 无 
    def initOffboard(self):
        """ Send command to make px4 enter offboard mode, and start to send offboard message 30Hz
        """

        # if not self.isPX4Ekf3DFixed:
        #     print('CopterSim still not 3DFxied, please wait and try again.');
        #     sys.exit(0)

        self.EnList = [0,1,0,0,0,1]
        self.type_mask = self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yaw=0
        self.yawrate = 0
        self.isInOffboard = True
        if self.UDPMode>1.5:
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO,PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
            time.sleep(0.5)
            self.t2.start()
            if not self.hasSendDisableRTLRC:
                self.sendMavSetParam('NAV_RCL_ACT'.encode(),0,mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('COM_RCL_EXCEPT'.encode(), 4/7.13622e+44, mavlink2.MAV_PARAM_TYPE_INT32)
                #self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, True)
                self.hasSendDisableRTLRC = True
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            time.sleep(0.1)
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            time.sleep(0.1)

            if not self.isArmed:
                self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            time.sleep(0.1)
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            time.sleep(0.1)
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        else:

            self.t2.start()
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)

    ## @brief 发送命令进入offboard模式，消息发送频率为30HZ
    #  - @anchor initOffboardAcc
    #  @param 无
    #  @return 无 
    def initOffboardAcc(self):
        """ Send command to make px4 enter offboard mode, and start to send offboard message 30Hz
        """
        # 使能加速度和偏航角速率控制
        self.EnList = [0, 0, 1, 0, 0, 1]
        self.type_mask = self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = 0
        self.isInOffboard = True
        if self.UDPMode > 1.5:
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO,
                             PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
            time.sleep(0.5)
            self.t2.start()
            if not self.hasSendDisableRTLRC:
                self.sendMavSetParam('NAV_RCL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('COM_RCL_EXCEPT'.encode(), 4 / 7.13622e+44, mavlink2.MAV_PARAM_TYPE_INT32)
                # self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, True)
                self.hasSendDisableRTLRC = True
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            time.sleep(0.1)
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            time.sleep(0.1)

            if not self.isArmed:
                self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            time.sleep(0.1)
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            time.sleep(0.1)
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            time.sleep(0.1)
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            time.sleep(0.1)
        else:
            self.t2.start()
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)

    ## @brief 发送命令进入offboard模式，消息发送频率为30HZ
    #  - @anchor initOffboardAcc
    #  @param 无
    #  @return 无 
    def initOffboardAtt(self):
        """ Send command to make px4 enter offboard mode, and start to send offboard message 30Hz
        """
        self.isInOffboard = True
        if self.UDPMode > 1.5:
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO,
                             PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
            time.sleep(0.5)
            self.SendAttPX4(att=[0, 0, 0, 0], thrust=0.11, CtrlFlag=1, AltFlg=0)
            self.t2.start()
            if not self.hasSendDisableRTLRC:
                self.sendMavSetParam('NAV_RCL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('COM_RCL_EXCEPT'.encode(), 4 / 7.13622e+44, mavlink2.MAV_PARAM_TYPE_INT32)
                # self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, True)
                self.hasSendDisableRTLRC = True
            if not self.isArmed:
                self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            time.sleep(0.5)
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        else:
            self.t2.start()
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)
            self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                    self.yawrate)

    ## @brief 发送命令进入offboard模式，消息发送频率为30HZ
    #  - @anchor initOffboardAcc
    #  @param 无
    #  @return 无 
    def initOffboard2(self):
        """ Send command to make px4 enter offboard mode, and start to send offboard message 30Hz
        """
        self.isInOffboard = True
        self.t2.start()
        self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                self.yawrate)
        self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                self.yawrate)
        if self.UDPMode>1.5:
            if not self.hasSendDisableRTLRC:
                self.sendMavSetParam('NAV_RCL_ACT'.encode(),0,mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
                self.sendMavSetParam('COM_RCL_EXCEPT'.encode(), 4/7.13622e+44, mavlink2.MAV_PARAM_TYPE_INT32)
                #self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, True)
                self.hasSendDisableRTLRC = True
            if not self.isArmed:
                self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            time.sleep(0.5)
            self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD)

    ## @brief 发送命令让飞行器起飞到指定位置
    #  - @anchor sendMavTakeOff
    #  @param xM（默认值为0）x方向的位置
    #  @param yM（默认值为0）y方向的位置
    #  @param zM（默认值为0）z方向的位置
    #  @param YawRad（默认值为0）起飞时的偏航角
    #  @param PitchRad（默认值为0）起飞时的俯仰角
    #  @return 无 
    def sendMavTakeOff(self,xM=0,yM=0,zM=0,YawRad=0,PitchRad=0):
        """ Send command to make aircraft takeoff to the desired local position (m)
        """
        if self.UDPMode>1.5:
            lla = self.geo.ned2lla([xM,yM,zM],self.uavPosGPSHome)
            lat=lla[0]
            lon=lla[1]
            alt=lla[2]
            self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_TAKEOFF, PitchRad/math.pi*180,0,0,YawRad/math.pi*180,lat,lon,alt)
        else:
            ctrls=[xM,yM,zM,0]
            self.sendUDPSimpData(11,ctrls)
    
    ## @brief 发送指令使飞机从指定的本地位置起飞，并允许设置俯仰角、偏航角和上升速率
    #  - @anchor sendMavTakeOffLocal
    #  @param xM（默认值为0）x方向的位置
    #  @param yM（默认值为0）y方向的位置
    #  @param zM（默认值为0）z方向的位置
    #  @param YawRad（默认值为0）起飞时的偏航角
    #  @param PitchRad（默认值为0）起飞时的俯仰角
    #  @param AscendRate（默认值为2）起飞时的上升速率
    #  @return 无 
    def sendMavTakeOffLocal(self,xM=0,yM=0,zM=0,YawRad=0,PitchRad=0,AscendRate=2):
        """ Send command to make aircraft takeoff to the desired local position (m)
        """
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_TAKEOFF_LOCAL, PitchRad,0,AscendRate,YawRad,yM,xM,zM)

    ## @brief 发送指令使飞机从指定的本地位置起飞，并允许设置俯仰角、偏航角和上升速率
    #  - @anchor sendMavTakeOffGPS
    #  @param lat 目标位置的经度
    #  @param lon 目标位置的维度
    #  @param alt 目标位置的高度
    #  @param YawRad（默认值为0）起飞时的偏航角
    #  @param pitchDeg（默认值为15）起飞时的俯仰角
    #  @return 无 
    def sendMavTakeOffGPS(self,lat,lon,alt,yawDeg=0,pitchDeg=15):
        """ Send command to make aircraft takeoff to the desired global position (degree)
        """
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_TAKEOFF, pitchDeg,0,0,yawDeg,lat,lon,alt)

    ## @brief 发送指令使飞机降落到指定位置
    #  - @anchor sendMavLand
    #  @param xM x轴位置
    #  @param yM y轴位置
    #  @param zM z轴位置
    #  @return 无 
    def sendMavLand(self,xM,yM,zM):
        """ Send command to make aircraft land to the desired local position (m)
        """
        yawRad=0
        lat=self.uavPosGPSHome[0]+xM/6381372/math.pi*180
        lon=self.uavPosGPSHome[1]+yM/6381372/math.pi*180
        alt=self.uavPosGPSHome[2]-zM
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_LAND , 0,0,0,yawRad/math.pi*180,lat,lon,alt)

    ## @brief 发送指令使飞机降落到指定全球位置
    #  - @anchor sendMavLandGPS
    #  @param lat 目标位置的经度
    #  @param lon 目标位置的维度
    #  @param alt 目标位置的高度
    #  @return 无 
    def sendMavLandGPS(self,lat,lon,alt):
        """ Send command to make aircraft land to the desired global position (degree)
        """
        yawRad=0
        self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_LAND , 0,0,0,yawRad/math.pi*180,lat,lon,alt)

    ## @brief 退出offboard模式，停止消息发送
    #  - @anchor endOffboard
    #  @return 无 
    # stop Offboard mode
    def endOffboard(self):
        """ Send command to px4 to out offboard mode, and stop the message sending loop
        """
        self.isInOffboard = False
        if self.UDPMode>1.5 and self.hasSendDisableRTLRC:
            self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, False)
            self.hasSendDisableRTLRC = False
        self.t2.join()

    # send command pixhawk to modify its parameters
    ## @brief 向 Pixhawk 发送命令更改其参数
    #  - @anchor sendMavSetParam
    #  @param param_id 修改的参数ID
    #  @param param_value 新的参数值
    #  @param param_type 参数的数据类型
    #  @return 无 
    def sendMavSetParam(self,param_id, param_value, param_type):
        """ Send command to px4 to change desired parameter
        the following mavlink message is adopted
        https://mavlink.io/en/messages/common.html#PARAM_SET
        the parameter list can be found in QGC
        """
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom or self.isRealFly:
            self.the_connection.mav.param_set_send(self.the_connection.target_system,self.the_connection.target_component,param_id,param_value, param_type)
        else:
            buf = self.mav0.param_set_encode(self.the_connection.target_system,self.the_connection.target_component,param_id,param_value, param_type).pack(self.mav0)
            self.SendBuf(buf)
            #self.udp_socket.sendto(buf, (self.ip, self.port))

    # send hil_actuator_controls message to Pixhawk (for rfly_ctrl uORB message)
    ## @brief 发送 hil_actuator_controls 消息发送到 Pixhawk
    #  - @anchor SendHILCtrlMsg
    #  @param ctrls（默认值为[0]*16） 执行器控制值的列表
    #  @param idx（默认值为0） 消息的索引
    #  @param flags（默认值为1） 一个flags标志位，给Simulink控制器用于模式切换
    #  @return 无 
    def SendHILCtrlMsg(self,ctrls=[0]*16,idx=0,flags=1):
        """ Send hil_actuator_controls command to PX4, which will be transferred to uORB message rfly_ctrl
        https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS
        """
        mode=1
        if idx<0.5:
            mode=1 # rfly_ctrl消息
        elif idx<1.5: # rfly_ctrl1消息
            mode=101
        elif idx<2.5: # rfly_ctrl2消息
            mode=201
        else:
            mode=1 # rfly_ctrl消息
        
        time_boot_ms = int((time.time()-self.startTime)*1000)
        controls = self.fillList(ctrls,16,0)
        
        if self.isCom or self.isRealFly:
            self.the_connection.mav.hil_actuator_controls_send(time_boot_ms,controls,mode,flags)
        else:
            buf = self.mav0.hil_actuator_controls_encode(time_boot_ms,controls,mode,flags).pack(self.mav0)
            self.SendBuf(buf)
        #print("Msg Send.")

    # send debug_vect message to Pixhawk to update rfly_ctrl uORB message
    ## @brief 发送 debug_vect 消息发送到 Pixhawk
    #  - @anchor SendHILCtrlMsg1
    #  @param 无
    #  @return 无 
    def SendHILCtrlMsg1(self):
        """  Send debug_vect command to PX4, which will be transferred to uORB message rfly_ctrl
        https://mavlink.io/en/messages/common.html#DEBUG_VECT
        """
        time_boot_ms = int((time.time()-self.startTime)*1000)
        name = b'hello'
        if self.isCom or self.isRealFly:
            self.the_connection.mav.debug_vect_send(name, time_boot_ms, 1100, 1500, 1700)
        else:
            buf = self.mav0.debug_vect_encode(name, time_boot_ms, 1100, 1500, 1700).pack(self.mav0)
            self.SendBuf(buf)
        #print("Msg1 Send.")

    # send MAVLink command to Pixhawk to Arm/Disarm the drone
    ## @brief 发送指令到 Pixhawk 使其上锁/解锁
    #  - @anchor SendMavArm
    #  @param isArm（默认值为1） 是否上锁
    #  - 0表示上锁
    #  - 1表示解锁
    #  @return 无 
    def SendMavArm(self, isArm=1):
        """ Send command to PX4 to arm or disarm the drone
        """
        if self.UDPMode>1.5:
            if (isArm):
                self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            else:
                self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196.0)
        else:
            ctrls=[isArm,0,0,0]
            self.sendUDPSimpData(9,ctrls)

    ## @brief 初始化遥控器发送循环
    #  - @anchor initRCSendLoop
    #  @param Hz（默认值为30） 发送频率
    #  @return 无 
    def initRCSendLoop(self, Hz=30):
        self.isRCLoop=True
        self.tRCHz=Hz
        self.sendMavSetParam('NAV_RCL_ACT'.encode(),0,mavlink2.MAV_PARAM_TYPE_INT32)
        self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
        time.sleep(0.2)
        self.tRC = threading.Thread(target=self.RcSendLoop, args=())
        self.tRC.start()

    ## @brief 结束遥控器发送循环
    #  - @anchor endRCSendLoop
    #  @param 无
    #  @return 无 
    def endRCSendLoop(self):
        self.isRCLoop=False
        time.sleep(0.5)
        self.tRC.join()

    ## @brief 循环发送遥控器信号
    #  - @anchor RcSendLoop
    #  @param 无
    #  @return 无 
    def RcSendLoop(self):
        lastTime = time.time()
        while True:
            if not self.isRCLoop:
                break
            lastTime = lastTime + (1.0/self.tRCHz)
            sleepTime = lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime = time.time()

            #print('RC send')
            self.SendRcOverride(self.RCPWMs[0],self.RCPWMs[1],self.RCPWMs[2],self.RCPWMs[3],self.RCPWMs[4],self.RCPWMs[5],self.RCPWMs[6],self.RCPWMs[7])
            self.sendMavManualCtrl((self.RCPWMs[1]-1500)*2,(self.RCPWMs[0]-1500)*2,(self.RCPWMs[2] - 1000),(self.RCPWMs[3]-1500)*2)

    ## @brief 更新遥控器 PWM 信号
    #  - @anchor SendRCPwms
    #  @param Pwms 新的 PWM 信号值
    #  @return 无 
    def SendRCPwms(self, Pwms):
        for i in range(len(Pwms)):
            if i<len(self.RCPWMs):
                self.RCPWMs[i]=Pwms[i]

    # send MAVLink rc_channels_override message to override the RC signals
    ## @brief 通过 MAVLink 协议向 PX4 发送遥控信号覆盖命令
    #  - @anchor SendRcOverride
    #  @param ch1（默认值为 1500） 通道1的信号值
    #  @param ch2（默认值为 1500） 通道2的信号值
    #  @param ch3（默认值为 1100） 通道3的信号值
    #  @param ch4（默认值为 1500） 通道4的信号值
    #  @param ch5（默认值为 1100） 通道5的信号值
    #  @param ch6（默认值为 1100） 通道6的信号值
    #  @param ch7（默认值为 1500） 通道7的信号值
    #  @param ch8（默认值为 1500） 通道8的信号值
    #  @return 无   
    def SendRcOverride(self, ch1=1500, ch2=1500, ch3=1100, ch4=1500, ch5=1100, ch6=1100, ch7=1500, ch8=1500):
        """ Send MAVLink command to PX4 to override the RC signal
        ch1~ch8 range from 1000 to 2000
        https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        """
        if self.isCom or self.isRealFly:
            self.the_connection.mav.rc_channels_override_send(self.the_connection.target_system,
                                                        self.the_connection.target_component, ch1, ch2,
                                                        ch3, ch4, ch5, ch6, ch7, ch8)
        else:
            buf = self.mav0.rc_channels_override_encode(self.the_connection.target_system,
                                                        self.the_connection.target_component, ch1, ch2,
                                                        ch3, ch4, ch5, ch6, ch7, ch8).pack(self.mav0)
            self.SendBuf(buf)
            #self.udp_socket.sendto(buf, (self.ip, self.port))

    # send MAVLink message manual_control to send normalized and calibrated RC sigals to pixhawk
    ## @brief 通过 MAVLink 协议向 PX4 发送手动控制信号
    #  - @anchor sendMavManualCtrl
    #  @param x（默认值为 0） 手动控制的横向信号
    #  @param y（默认值为 0） 手动控制的纵向信号
    #  @param z（默认值为 0） 手动控制的垂直控制信号
    #  @param r（默认值为 0） 手动控制的旋转信号
    #  @return 无 
    def sendMavManualCtrl(self, x=0,y=0,z=0,r=0):
        """ Send MAVLink command to PX4 to override the manual control signal
        x,y,z,r range from -1000 to 1000
        https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        """
        if self.isInPointMode or self.UDPMode<1.5:
            return
        if self.isCom or self.isRealFly:
            # self.the_connection.mav.manual_control_encode(self.the_connection.target_system, x, y, z, r, 0)
            self.the_connection.mav.manual_control_send(self.the_connection.target_system, x, y, z, r, 0)
        else:
            buf = self.mav0.manual_control_encode(self.the_connection.target_system, x,y,z,r,0).pack(self.mav0)
            self.SendBuf(buf)
            #self.udp_socket.sendto(buf, (self.ip, self.port))

    # send MAVLink command to change current flight mode
    ## @brief 发送 MAVLink 命令到 PX4 以更改飞行模式
    #  - @anchor SendSetMode
    #  @param mainmode 主模式
    #  @param cusmode（默认值为 0） 可选参数，用户自定义模式
    #  @return 无 
    def SendSetMode(self,mainmode,cusmode=0):
        """ Send MAVLink command to PX4 to change flight mode
        https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE
        """
        basemode = mavlink2.MAV_MODE_FLAG_HIL_ENABLED | mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        self.SendMavCmdLong(mavlink2.MAV_CMD_DO_SET_MODE, basemode, mainmode, cusmode)

    # Stop MAVLink data listening loop
    ## @brief 停止 MAVLink 的监听循环
    #  - @anchor stopRun
    #  @param 无
    #  @return 无 
    def stopRun(self):
        """ stop mavlink listening loop for InitMavLoop(), the same as endMavLoop()
        """

        if self.isArmed:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196.0)
        self.stopFlag=True
        time.sleep(0.5)
        if self.t1 is not None:
            self.t1.join()
        if(self.isInOffboard):
            self.endOffboard()
        if self.UDPMode>1.5:
            self.the_connection.close()
        else:
            if not self.isCom:
                if self.isRedis:
                    self.redisCon.close()
                else:
                    self.udp_socketUDP.close()

    # Update Pixhawk states from MAVLink for 100Hz
    ## @brief 从30100串口监听真是数据，更新Pixhawk状态
    #  - @anchor getTrueDataMsg
    #  @param 无
    #  @return 无 
    def getTrueDataMsg(self):
        """ Start loop to listen True data from 30100 serial data
        """

        while True:
            if self.stopFlagTrueData:
                break

            try:
                sock = None
                if not self.isRedis:
                    sock = self.udp_socketTrue
                buf = self.GetUDPRedisBuf(sock)
                if buf is None:
                    continue
                
                # struct SOut2SimulatorOld {
                #     int copterID;  //Vehicle ID
                #     int vehicleType;  //Vehicle type
                #     double runnedTime; //Current Time stamp (s)
                #     float VelE[3];   //NED vehicle velocity in earth frame (m/s)
                #     float PosE[3];   //NED vehicle position in earth frame (m)
                #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
                #     float AngQuatern[4]; //Vehicle attitude in Quaternion
                #     float MotorRPMS[8];  //Motor rotation speed (RPM)
                #     float AccB[3];       //Vehicle acceleration in body frame x y z (m/s/s)
                #     float RateB[3];      //Vehicle angular speed in body frame x y z (rad/s)
                #     double PosGPS[3];    //vehicle longitude, latitude and altitude (degree,degree,m)

                # }
                # typedef struct _netDataShort {
                #     int tg;
                #     int        len;
                #     char       payload[192];
                # }netDataShort;
                
                if len(buf)==192+8:
                    #print(len(buf[0:8]))
                    tg,strLen = struct.unpack('ii',buf[0:8])
                    if strLen==152:
                        UIV=struct.unpack('2i1d27f3d',buf[8:8+152])
                        #print(self.uavGlobalPos[2])
                        self.trueTimeStmp = UIV[2]
                        self.trueAngEular[0]=UIV[9]
                        self.trueAngEular[1]=UIV[10]
                        self.trueAngEular[2]=UIV[11]
                        self.truePosNED[0]=UIV[6]
                        self.truePosNED[1]=UIV[7]
                        self.truePosNED[2]=UIV[8]
                        self.trueVelNED[0]=UIV[3]
                        self.trueVelNED[1]=UIV[4]
                        self.trueVelNED[2]=UIV[5]
                        self.trueAngRate[0]=UIV[27]
                        self.trueAngRate[1]=UIV[28]
                        self.trueAngRate[2]=UIV[29]
                        self.trueAngQuatern[0]=UIV[12]
                        self.trueAngQuatern[1]=UIV[13]
                        self.trueAngQuatern[2]=UIV[14]
                        self.trueAngQuatern[3]=UIV[15]
                        self.trueMotorRPMS[0]=UIV[16]
                        self.trueMotorRPMS[1]=UIV[17]
                        self.trueMotorRPMS[2]=UIV[18]
                        self.trueMotorRPMS[3]=UIV[19]
                        self.trueMotorRPMS[4]=UIV[20]
                        self.trueMotorRPMS[5]=UIV[21]
                        self.trueMotorRPMS[6]=UIV[22]
                        self.trueMotorRPMS[7]=UIV[23]
                        self.trueAccB[0]=UIV[24]
                        self.trueAccB[1]=UIV[25]
                        self.trueAccB[2]=UIV[26]
                        self.truePosGPS[0]=UIV[30]
                        self.truePosGPS[1]=UIV[31]
                        self.truePosGPS[2]=UIV[32]
                        if not self.hasTrueDataRec:
                            self.hasTrueDataRec=True
                        self.hasMsgDict['TrueDataUDP']=True
                    if tg>-0.5:
                        self.isVehicleCrash=True
                        self.isVehicleCrashID=tg
                # struct Ue4CMDGPS{
                #     int checksum;//校验码，用于确认数据正确性，这里取值 1234567890
                #     int CopterID;
                #     double GPS[3];
                # } ii3d
                if len(buf)==32:
                    UIV=struct.unpack('ii3d',buf)
                    checksum=UIV[0]
                    CopterID=UIV[1]
                    if checksum==1234567890:
                        if not self.useCustGPSOri:
                            self.trueGpsUeCenter[0]=UIV[2]
                            self.trueGpsUeCenter[1]=UIV[3]
                            self.trueGpsUeCenter[2]=UIV[4]

                            # 如果获取到了GPS坐标，则重新计算位置偏差
                            if not (abs(self.uavPosGPSHome[0]<0.01) and abs(self.uavPosGPSHome[1]<0.01)):
                                # 计算地图偏差
                                self.GpsOriOffset=self.geo.lla2ned(self.uavPosGPSHome,self.trueGpsUeCenter)
                            else:
                                self.GpsOriOffset=[0,0,0]
                        self.hasMsgDict['Ue4CMDGPS']=True

                if len(buf)==12:
                    checksum,CopterID,targetID = struct.unpack('iii',buf[0:12])
                    if checksum==1234567890:
                        if targetID>-0.5:
                            self.isVehicleCrash=True
                            self.isVehicleCrashID=targetID
                # struct outCopterStruct{
                #     int checksum; //1234567890
                #     int CopterID;
                #     double data[32]; //data
                # } ii32d  4+4+8*32 264
                if len(buf)==264:
                    UIV=struct.unpack('ii32d',buf)
                    checksum=UIV[0]
                    CopterID=UIV[1]
                    if checksum==1234567890:
                        self.trueSimulinkData=UIV[2:34]
                        self.hasMsgDict['outCopter']=True


                # //输出到模拟器的数据
                # struct SOut2Simulator {
                #     int checksum; // 123456789
                #     int copterID;  //Vehicle ID
                #     int vehicleType;  //Vehicle type
                #     int reserv; //备用标志位
                #     float VelE[3];   //NED vehicle velocity in earth frame (m/s)
                #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
                #     float AngQuatern[4]; //Vehicle attitude in Quaternion
                #     float MotorRPMS[8];  //Motor rotation speed (RPM)
                #     float AccB[3];       //Vehicle acceleration in body frame x y z (m/s/s)
                #     float RateB[3];      //Vehicle angular speed in body frame x y z (rad/s)
                #     double runnedTime; //Current  stamp (s)
                #     double PosE[3];   //NED vehicle position in earth frame (m)
                #     double PosGPS[3];    //vehicle longitude, latitude and altitude (degree,degree,m)
                # } 4i24f7d
                if len(buf)==168:
                    UIV=struct.unpack('4i24f7d',buf)
                    checksum=UIV[0]
                    if checksum==123456789:
                        self.trueVelNED[0]=UIV[4]
                        self.trueVelNED[1]=UIV[5]
                        self.trueVelNED[2]=UIV[6]
                        self.trueAngEular[0]=UIV[7]
                        self.trueAngEular[1]=UIV[8]
                        self.trueAngEular[2]=UIV[9]
                        self.trueAngQuatern[0]=UIV[10]
                        self.trueAngQuatern[1]=UIV[11]
                        self.trueAngQuatern[2]=UIV[12]
                        self.trueAngQuatern[3]=UIV[13]
                        self.trueMotorRPMS[0]=UIV[14]
                        self.trueMotorRPMS[1]=UIV[15]
                        self.trueMotorRPMS[2]=UIV[16]
                        self.trueMotorRPMS[3]=UIV[17]
                        self.trueMotorRPMS[4]=UIV[18]
                        self.trueMotorRPMS[5]=UIV[19]
                        self.trueMotorRPMS[6]=UIV[20]
                        self.trueMotorRPMS[7]=UIV[21]
                        self.trueAccB[0]=UIV[22]
                        self.trueAccB[1]=UIV[23]
                        self.trueAccB[2]=UIV[24]
                        self.trueAngRate[0]=UIV[25]
                        self.trueAngRate[1]=UIV[26]
                        self.trueAngRate[2]=UIV[27]
                        self.trueTimeStmp = UIV[28]
                        self.truePosNED[0]=UIV[29]
                        self.truePosNED[1]=UIV[30]
                        self.truePosNED[2]=UIV[31]
                        self.truePosGPS[0]=UIV[32]
                        self.truePosGPS[1]=UIV[33]
                        self.truePosGPS[2]=UIV[34]
                        self.out3Ddata.updateUdp(UIV)

                # struct SOut2SimulatorSimpleTime {
                #     int checkSum; //1234567891
                #     int copterID;  //Vehicle ID
                #     int vehicleType;  //Vehicle type
                #     int PosGpsInt[3];   //lat*10^7,lon*10^7,alt*10^3，int型发放节省空间
                #     float MotorRPMS[8];
                #     float VelE[3];
                #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
                #     double PosE[3];   //NED vehicle position in earth frame (m)
                #     double runnedTime; //Current Time stamp (s)
                # }6i14f4d
                if len(buf)==112:
                    UIV=struct.unpack('6i14f4d',buf)
                    checksum=UIV[0]
                    if checksum==1234567891:
                        self.truePosGPS[0]=UIV[3]/1e7
                        self.truePosGPS[1]=UIV[4]/1e7
                        self.truePosGPS[2]=UIV[5]/1e3
                        self.trueMotorRPMS[0]=UIV[6]
                        self.trueMotorRPMS[1]=UIV[7]
                        self.trueMotorRPMS[2]=UIV[8]
                        self.trueMotorRPMS[3]=UIV[9]
                        self.trueMotorRPMS[4]=UIV[10]
                        self.trueMotorRPMS[5]=UIV[11]
                        self.trueMotorRPMS[6]=UIV[12]
                        self.trueMotorRPMS[7]=UIV[13]
                        self.trueVelNED[0]=UIV[14]
                        self.trueVelNED[1]=UIV[15]
                        self.trueVelNED[2]=UIV[16]
                        self.trueAngEular[0]=UIV[17]
                        self.trueAngEular[1]=UIV[18]
                        self.trueAngEular[2]=UIV[19]
                        self.truePosNED[0]=UIV[20]
                        self.truePosNED[1]=UIV[21]
                        self.truePosNED[2]=UIV[22]
                        self.trueTimeStmp = UIV[23]
                    
                    
                self.trueMsgEvent.set()
            except Exception as e:
                print(e)
                self.stopFlagTrueData=True
                break

    # Update Pixhawk states from MAVLink for 100Hz
    ## @brief 从40100串口监听真是数据，更新Pixhawk状态
    #  - @anchor getPX4DataMsg
    #  @param 无
    #  @return 无 
    def getPX4DataMsg(self):
        """ Start loop to listen True data from 40100 serial data
        """
        # print(time.time())

        # struct PX4ExtMsg {
        #     int checksum;  //1234567898
        #     int CopterID;
        #     double runnedTime; //Current  stamp (s)
        #     float controls[8];
        # }iid8f
        while True:
            if self.stopFlagPX4Data:
                break

            try:
                buf,addr = self.udp_socketPX4.recvfrom(65500)
                if len(buf)==48:
                    #print('Data Receved!')
                    UIV = struct.unpack('iid8f',buf)
                    checksum=UIV[0]
                    if checksum==1234567898:
                        #print('Data Receved!')
                        self.px4ext.checksum=checksum
                        self.px4ext.CopterID=UIV[1]
                        self.px4ext.runnedTime=UIV[2]
                        self.px4ext.controls=UIV[3:11]
                        self.px4extTrue=True
            except:
                self.stopFlagPX4Data=True
                break

    ## @brief 设置消息字典的状态
    #  - @anchor setMsgDict
    #  @param stName 状态名称
    #  @return 无 
    def setMsgDict(self,stName):
        self.hasMsgDict[stName]=True
        trig=False
        if (stName in self.trigMsgVect):
            trig=True
        return trig

    ## @brief 用于网络通信的事件信号
    #  - @anchor netForwardData
    #  @param 无
    #  @return 无
    def netForwardData(self):
        self.netEvent.set()

    ## @brief 从套接字或 Redis 订阅中获取数据
    #  - @anchor GetUDPRedisBuf
    #  @param sock 套接字
    #  @return buf 接收到的数据
    def GetUDPRedisBuf(self, sock):
        if self.isRedis:
            data = self.pubsub.get_message()
            if data is None:
                return
            if data['type'] == 'subscribe' or not isinstance(data['data'], bytes):
                return
            buf = data['data']
        else:
            buf, addr = sock.recvfrom(65500)
        return buf

    # Update Pixhawk states from MAVLink for 100Hz
    ## @brief 开始循环监听来自20100系列端口或COM端口的MAVLink数据，更新Pixhawk状态
    #  - @anchor getMavMsg
    #  @param 无
    #  @return 无
    def getMavMsg(self):
        """ Start loop to listen mavlink data from 20100 series port or COM port
        """

        while True:
            if self.stopFlag:
                break

            shouldTrig=False

            #print('Get msg')

            if self.UDPMode>1.5: # If Use MAVLink Mode
                msg = self.the_connection.recv_match(
                    blocking=True)
                if msg is not None:
                    
                    # 如果收到消息
                    self.uavEvent.set()
                    self.uavMsg=msg

                    if msg.get_type() == "ATTITUDE":
                        self.uavTimeStmp = msg.time_boot_ms/1000.0
                        self.uavAngEular[0] = msg.roll
                        self.uavAngEular[1] = msg.pitch
                        self.uavAngEular[2] = msg.yaw
                        self.uavAngRate[0] = msg.rollspeed
                        self.uavAngRate[1] = msg.pitchspeed
                        self.uavAngRate[2] = msg.yawspeed
                        shouldTrig=self.setMsgDict('ATTITUDE')

                    if msg.get_type() == "ATTITUDE_QUATERNION":
                        self.uavAngQuatern[0] = msg.q1
                        self.uavAngQuatern[1] = msg.q2
                        self.uavAngQuatern[2] = msg.q3
                        self.uavAngQuatern[3] = msg.q4

                    if msg.get_type() == "LOCAL_POSITION_NED":
                        self.uavPosNED[0] = msg.x
                        self.uavPosNED[1] = msg.y
                        self.uavPosNED[2] = msg.z
                        self.uavVelNED[0] = msg.vx
                        self.uavVelNED[1] = msg.vy
                        self.uavVelNED[2] = msg.vz
                        if not (abs(self.uavPosGPSHome[0])<0.001 and abs(self.uavPosGPSHome[1])<0.001):
                            self.uavGlobalPos[0]=self.GpsOriOffset[0]+self.uavPosNED[0]
                            self.uavGlobalPos[1]=self.GpsOriOffset[1]+self.uavPosNED[1]
                            self.uavGlobalPos[2]=self.GpsOriOffset[2]+self.uavPosNED[2]
                        shouldTrig=self.setMsgDict('LOCAL_POSITION_NED')
                        self.netForwardData() # 将消息转发出去

                    if msg.get_type() == "VIBRATION":
                        self.uavVibr[0]=msg.vibration_x
                        self.uavVibr[1]=msg.vibration_y
                        self.uavVibr[2]=msg.vibration_z

                    if msg.get_type() == "HIGHRES_IMU":
                        self.uavAccB[0] = msg.xacc
                        self.uavAccB[1] = msg.yacc
                        self.uavAccB[2] = msg.zacc
                        self.uavGyro[0] = msg.xgyro
                        self.uavGyro[1] = msg.ygyro
                        self.uavGyro[2] = msg.zgyro
                        self.uavMag[0] = msg.xmag
                        self.uavMag[1] = msg.ymag
                        self.uavMag[2] = msg.zmag

                    if msg.get_type() == "HOME_POSITION":
                        self.uavPosGPSHome[0] = msg.latitude/1e7
                        self.uavPosGPSHome[1] = msg.longitude/1e7
                        self.uavPosGPSHome[2] = msg.altitude/1e3

                        # 计算地图偏差
                        self.GpsOriOffset=self.geo.lla2ned(self.uavPosGPSHome,self.trueGpsUeCenter)

                        shouldTrig=self.setMsgDict('HOME_POSITION')

                    if msg.get_type() == "GLOBAL_POSITION_INT":
                        self.uavPosGPS[0] = msg.lat/1e7
                        self.uavPosGPS[1] = msg.lon/1e7
                        self.uavPosGPS[2] = msg.alt/1e3
                        self.uavPosGPS[3] = msg.time_boot_ms/1000
                        self.uavPosGPS[4] = msg.relative_alt/1e3
                        self.uavPosGPS[5] = msg.vx/100
                        self.uavPosGPS[6] = msg.vy/100
                        self.uavPosGPS[7] = msg.vz/100
                        hdg = msg.hdg/100
                        if hdg>180:
                            hdg = hdg-360
                        self.uavPosGPS[8] = hdg/180*math.pi
                        shouldTrig=self.setMsgDict('GLOBAL_POSITION_INT')

                    if msg.get_type() == 'BATTERY_STATUS':
                        self.batInfo[1] = msg.voltages[0]
                        self.batInfo[0] = msg.battery_remaining
                        shouldTrig=self.setMsgDict('BATTERY_STATUS')
                        #print("Mav %d: battery (%f,%f)" % (self.conn.target_system,m.voltages[0],m.battery_remaining))

                    if msg.get_type() == "HEARTBEAT":
                        # 避免qgc和python同时连接时相互干扰
                        if msg.custom_mode == 0:
                            continue
                        isArmed = msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED
                        if not self.isArmed and isArmed:
                            pass
                            #print("PX4 Armed!")
                        if self.isArmed and not isArmed:
                            self.isArmerror = 1
                            #print("PX4 DisArmed!")
                        self.isArmed = isArmed
                        #print("HeartBeat!")

                        self.system_status = msg.system_status
                        #print("MAV ID === %d,system_status === %f"%(self.nId,msg.system_status))
                        self._active = True
                        shouldTrig=self.setMsgDict('HEARTBEAT')

                    if msg.get_type() == "HIL_ACTUATOR_CONTROLS":
                        if msg.flags == 1234567890:
                            if not self.hasTrueDataRec:
                                if msg.mode == 2 and msg.controls[15]>-0.5:
                                    self.isVehicleCrash = True
                                    self.isVehicleCrashID=int(msg.controls[15])
                                self.trueTimeStmp = msg.time_usec/1000.0
                                self.trueAngEular[0]=msg.controls[0]
                                self.trueAngEular[1]=msg.controls[1]
                                self.trueAngEular[2]=msg.controls[2]
                                self.truePosNED[0]=msg.controls[3]
                                self.truePosNED[1]=msg.controls[4]
                                self.truePosNED[2]=msg.controls[5]
                                self.trueVelNED[0]=msg.controls[6]
                                self.trueVelNED[1]=msg.controls[7]
                                self.trueVelNED[2]=msg.controls[8]
                                self.trueAngRate[0]=msg.controls[9]
                                self.trueAngRate[1]=msg.controls[10]
                                self.trueAngRate[2]=msg.controls[11]
                            if not self.isPX4Ekf3DFixed and msg.mode != 2 and msg.controls[15]>0.5:
                                self.isPX4Ekf3DFixed=True
                            shouldTrig=self.setMsgDict('TrueDataRec')
                        else:
                            self.uavMotorRPMS[0] = msg.controls[0]
                            self.uavMotorRPMS[1] = msg.controls[1]
                            self.uavMotorRPMS[2] = msg.controls[2]
                            self.uavMotorRPMS[3] = msg.controls[3]
                            self.uavMotorRPMS[4] = msg.controls[4]
                            self.uavMotorRPMS[5] = msg.controls[5]
                            self.uavMotorRPMS[6] = msg.controls[6]
                            self.uavMotorRPMS[7] = msg.controls[7]

                    if msg.get_type() == "STATUSTEXT":
                        #print(msg.text)
                        if msg.text.find('Failsafe')!= -1:
                            self.isFailsafeEn=True
                            self.FailsafeInfo = msg.text
                            print(msg.text)
                        shouldTrig=self.setMsgDict('STATUSTEXT')

                    if msg.get_type() == "ATTITUDE_TARGET":
                        self.uavThrust = msg.thrust
                        #print(msg.thrust)
                        shouldTrig=self.setMsgDict('ATTITUDE_TARGET')

                    if msg.get_type() == "GPS_RAW_INT":
                        pass


                    if shouldTrig:
                        self.hasMsgEvent.set()
                # else:
                #     break
            else:
                shouldTrig=False
                if self.UDPMode==0:
                    # if use UDP Mode
                    #II3i3i3iiiiii3f3f3ffff
                    # struct outHILStateData{ // mavlink data forward from Pixhawk
                    #     uint32_t time_boot_ms; //Timestamp of the message
                    #     uint32_t copterID;     //Copter ID start from 1
                    #     int32_t GpsPos[3];     //Estimated GPS position，lat&long: deg*1e7, alt: m*1e3 and up is positive
                    #     int32_t GpsVel[3];     //Estimated GPS velocity, NED, m/s*1e2->cm/s
                    #     int32_t gpsHome[3];     //Home GPS position, lat&long: deg*1e7, alt: m*1e3 and up is positive
                    #     int32_t relative_alt;  //alt: m*1e3 and up is positive
                    #     int32_t hdg;           //Course angle, NED,deg*1000, 0~360
                    #     int32_t satellites_visible; //GPS Raw data, sum of satellite
                    #     int32_t fix_type;     //GPS Raw data, Fixed type, 3 for fixed (good precision)
                    #     int32_t resrveInit;       //Int, reserve for the future use
                    #     float AngEular[3];    //Estimated Euler angle, unit: rad/s
                    #     float localPos[3];    //Estimated locoal position, NED, unit: m
                    #     float localVel[3];    //Estimated locoal velocity, NED, unit: m/s
                    #     float pos_horiz_accuracy;   //GPS horizontal accuracy, unit: m
                    #     float pos_vert_accuracy; //GPS vertical accuracy, unit: m
                    #     float resrveFloat;      //float,reserve for the future use
                    # }
                    # typedef struct _netDataShortShort {
                    #     TargetType tg;
                    #     int        len;
                    #     char       payload[112];
                    # }netDataShortShort;
                    try:
                        sock = None
                        if not self.isRedis:
                            sock = self.udp_socketUDP
                        buf = self.GetUDPRedisBuf(sock)
                        if buf is None:
                            continue
                        if len(buf)==112+8:
                            #print(len(buf[0:8]))
                            tg,strLen = struct.unpack('ii',buf[0:8])
                            if strLen==112:

                                UIV=struct.unpack('2I14i12f',buf[8:120])

                                shouldTrig=self.setMsgDict('HILStateData')
                                #GpsPos[0]=
                                #time_boot_ms,copterID,GpsPos,GpsVel,gpsHome,relative_alt,hdg,satellites_visible,fix_type,resrveInit,AngEular,localPos,localVel,pos_horiz_accuracy,pos_vert_accuracy,resrveFloat
                                self.uavTimeStmp = UIV[0]/1000.0
                                for idx in range(3):
                                    self.uavAngEular[idx]=UIV[16+idx]
                                    self.uavPosNED[idx]=UIV[19+idx]
                                    self.uavVelNED[idx]=UIV[22+idx]
                                self.uavPosGPS[0] = UIV[2]/1e7
                                self.uavPosGPS[1] = UIV[3]/1e7
                                self.uavPosGPS[2] = UIV[4]/1e3
                                self.uavPosGPSHome[0] = UIV[8]/1e7
                                self.uavPosGPSHome[1] = UIV[9]/1e7
                                self.uavPosGPSHome[2] = UIV[10]/1e3
                                # 计算地图偏差
                                self.GpsOriOffset=self.geo.lla2ned(self.uavPosGPSHome,self.trueGpsUeCenter)

                                if not (abs(self.uavPosGPSHome[0])<0.001 and abs(self.uavPosGPSHome[1])<0.001):
                                    self.uavGlobalPos[0]=self.GpsOriOffset[0]+self.uavPosNED[0]
                                    self.uavGlobalPos[1]=self.GpsOriOffset[1]+self.uavPosNED[1]
                                    self.uavGlobalPos[2]=self.GpsOriOffset[2]+self.uavPosNED[2]
                                if not self.isPX4Ekf3DFixed and UIV[27]>0.5:
                                    self.isPX4Ekf3DFixed=True
                                #print(self.uavGlobalPos[2])
                                if tg>-0.5:
                                    self.isVehicleCrash=True
                                    self.isVehicleCrashID=tg

                                self.netForwardData() # 将消息转发出去

                    except Exception as e:
                        print(e)
                        self.stopFlag=True
                        break
                else:
                    try:
                        sock = None
                        if not self.isRedis:
                            sock = self.udp_socketUDP
                        buf = self.GetUDPRedisBuf(sock)
                        if buf is None:
                            continue
                        if len(buf)==52:

                            #struct outHILStateShort{
                            #    int checksum;
                            #    int32_t gpsHome[3];     //GPS原始数据，其中经纬度(度*1e7)，高度向上为正(单位m*1e3->mm)
                            #    float AngEular[3];    //滤波后的飞机欧拉角，单位rad
                            #    float localPos[3];    //滤波后的本地位置，单位m
                            #    float localVel[3];    //滤波后的本地位置，单位m/s
                            #}

                            #print(len(buf[0:8]))
                            UIV=struct.unpack('4i9f',buf)
                            checksum=UIV[0]
                            #GpsPos[0]=
                            #time_boot_ms,copterID,GpsPos,GpsVel,gpsHome,relative_alt,hdg,satellites_visible,fix_type,resrveInit,AngEular,localPos,localVel,pos_horiz_accuracy,pos_vert_accuracy,resrveFloat
                            if checksum==1234567890:
                                shouldTrig=self.setMsgDict('HILStateSimple')
                                self.uavTimeStmp =time.time()
                                for idx in range(3):
                                    self.uavAngEular[idx]=UIV[4+idx]
                                    self.uavPosNED[idx]=UIV[7+idx]
                                    self.uavVelNED[idx]=UIV[10+idx]
                                self.uavPosGPSHome[0] = UIV[1]/1e7
                                self.uavPosGPSHome[1] = UIV[2]/1e7
                                self.uavPosGPSHome[2] = UIV[3]/1e3

                                # 计算地图偏差
                                self.GpsOriOffset=self.geo.lla2ned(self.uavPosGPSHome,self.trueGpsUeCenter)

                                if not (abs(self.uavPosGPSHome[0])<0.001 and abs(self.uavPosGPSHome[1])<0.001):
                                    self.uavGlobalPos[0]=self.GpsOriOffset[0]+self.uavPosNED[0]
                                    self.uavGlobalPos[1]=self.GpsOriOffset[1]+self.uavPosNED[1]
                                    self.uavGlobalPos[2]=self.GpsOriOffset[2]+self.uavPosNED[2]
                                #print(self.uavGlobalPos[2])

                                self.netForwardData() # 将消息转发出去

                            elif checksum==1234567891:
                                if not self.isPX4Ekf3DFixed:
                                    self.isPX4Ekf3DFixed=True
                                shouldTrig=self.setMsgDict('PX4Ekf3DFixed')
                            elif checksum==1234567892 and UIV[2]>-0.5:
                                self.isVehicleCrash=True
                                self.isVehicleCrashID=UIV[2]
                                shouldTrig=self.setMsgDict('VehicleCrash')

                    except Exception as e:
                        print(e)
                        self.stopFlag=True
                        break

                if shouldTrig:
                    self.hasMsgEvent.set()
            # 触发数据接收事件

        #print("Mavlink Stoped.")

    # Offboard message sending loop, 100Hz 
    ## @brief 循环发送MAVLink的offboard模式控制命令
    #  - @anchor OffboardSendMode
    #  @param 无
    #  @return 无
    def OffboardSendMode(self):
        lastTime2 = time.time()

        interTime=0.01
        # If in Simple mode, we use 30Hz to reduce network load
        if self.UDPMode==1 or self.UDPMode==3 or self.UDPMode==4:
            interTime=1/30.0

        while True:
            if not self.isInOffboard:
                break
            lastTime2 = lastTime2 + interTime
            sleepTime = lastTime2 - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime2 = time.time()
            if self.isInOffboard:
                self.sendMavOffboardAPI(self.type_mask, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw, self.yawrate)
        #print("Offboard Stoped.")

    # Offboard message sending loop, 100Hz
    ## @brief 发送消息重启飞机仿真
    #  - @anchor sendRebootPix
    #  @param copterID 飞机编号
    #  @param delay（默认值为-1） 重启前的延迟时间
    #  @return 无
    def sendRebootPix(self,copterID,delay=-1):
        """ Send message to restart simulation to CopterID
        """
        checksum = 987654321 #checksum for reboot message
        buf = struct.pack("2i",checksum,delay)
        self.udp_socket.sendto(buf, ('255.255.255.255', 20100+copterID*2-2))

    # Send a 16D Message to Simulink or other Apps
    ## @brief 发送一个16维double型数据到指定端口，本接口可用于与Simulink通信
    #  - @anchor sendCustomData
    #  @param CopterID 飞机编号
    #  @param data（默认值为16维0向量） 要发送的数据
    #  @param checksum（默认值123456） 用于数据校验
    #  @param port（默认值50000） 数据发送的端口
    #  @param IP（默认值127.0.0.1） 数据发送的IP地址
    #  @return 无

    def sendCustomData(self,CopterID,data=[0]*16,checksum=123456,port=50000,IP='127.0.0.1'):
        """ Send 16d message to desired IP and port
        """
        # 数据结构体的定义如下
        # struct CustomData{
        #     int checksum;
        #     int CopterID;
        #     double data
        # } ii16d 136包长
        buf = struct.pack("ii16d",checksum,CopterID,*data)
        self.udp_socket.sendto(buf, (IP, port))   

    ## @brief 设置GPS的原点位置，并根据提供的GPS坐标计算位置偏差
    #  - @anchor setGPSOriLLA
    #  @param LonLatAlt 经度、纬度和高度
    #  @return 无
    def setGPSOriLLA(self,LonLatAlt):
        # lla -> 纬度，经度，高度
        self.useCustGPSOri=True
        self.trueGpsUeCenter=LonLatAlt
        self.GpsOriOffset=[0,0,0]

        # 如果获取到了GPS坐标，则重新计算位置偏差
        if not (abs(self.uavPosGPSHome[0]<0.01) and abs(self.uavPosGPSHome[1]<0.01)):
            # 计算地图偏差
            self.GpsOriOffset=self.geo.lla2ned(self.uavPosGPSHome,self.trueGpsUeCenter)

    # 注意：这个接口将来要移植到DllSimCtrlAPI.py里面去
    # 发送到DLL模型的inSILInts and inSILFLoats接口
    #    struct PX4SILIntFloat{
    #     int checksum;//1234567897
    #     int CopterID;
    #     int inSILInts[8];
    #     float inSILFLoats[20];
    #    }
    # inSILInts and inSILFLoats will send to DLL model's inputs
    # CopterID is the vehicle you want to send, if copterID=-1 then it will send to yourself.   
    def sendSILIntFloat(self,inSILInts=[0]*8,inSILFLoats=[0]*20,copterID=-1):
        checkSum=1234567897
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2
        buf = struct.pack("10i20f",checkSum,ID,*inSILInts,*inSILFLoats)
        self.SendBufTrue(buf,PortNum)
        #self.udp_socket.sendto(buf, (self.ip, PortNum))

