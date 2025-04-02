#!/usr/bin/env python
# ROS1 到 ROS2 的迁移可参考 https://docs.ros.org/en/crystal/Contributing/Migration-Guide-Python.html
# ROS1 demo: https://docs.px4.io/main/en/ros/mavros_offboard_python.html
# ROS2 demo: https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py
# https://docs.px4.io/main/en/ros/ros2_comm.html

from mavros_msgs.msg import State,PositionTarget,ParamValue,Mavlink
from mavros_msgs.srv import CommandBool, SetMode, ParamSet, CommandLong
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Header
import time
import math
import os
import socket
import threading
import struct
import sys
import math
import numpy as np
import EarthModel
#from mavros import mavlink as mavlink0
from pymavlink.dialects.v20 import common as mavlink2
from pymavlink import mavutil
import platform
import subprocess

#from pymavlink.dialects.v20 import common as mavlink2

## @file
#  @brief 该接口为RflySim工具链集成ROS环境开发的通信接口。
#  @anchor PX4MavCtrlV4ROS接口库文件

##  @brief 判断是否在Linux环境下
isLinux = False
##  @brief 判断ROS是否正常安装
isRosOk = False
##  @brief 判断ROS的版本
is_use_ros1 = False


##  @brief 根据ROS版本导入不同的Python库
if platform.system().lower() == "linux":
    isLinux = True

    ros_version = os.getenv("ROS_VERSION")
    print("current ros environment: ROS",ros_version,", ", os.getenv("ROS_DISTRO"))

    try:
        if ros_version == "1":
            import rospy
            is_use_ros1 = True
        else:
            import rclpy
            from mavros.base import SENSOR_QOS
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            is_use_ros1 = False
        isRosOk=True

    except ImportError:
        print("Failed to load ROS libs")

## @brief MAVLink初始化类
class fifo(object):
    ## @brief 构造函数。
    #  -@anchor __init__
    #  @param buf 初始化一个空列表。
    def __init__(self):
        self.buf = []
    
    ## @brief 将输入的数据添加到buf列表的末尾，并返回添加的数据长度。
    #  @param  data: 输入的数据，将接收到data添加到buf列表的末尾。
    #  @return  返回添加的数据长度。
    def write(self, data):
        self.buf += data
        return len(data)
    ## @brief 将buf列表的首位元素删除，并返回该列表。
    #  @return  返回buf列表。
    def read(self):
        return self.buf.pop(0)



##  @brief 无人机通信实例。
#  
#   此类通过UDP、COM方式与无人机模拟器进行通信，控制模拟器的操作。
class PX4MavCtrler:
    """
    ID: 表示飞机的CopterID号。按平台规则，port=20100+CopterID*2-2。
    ip: 数据向外发送的IP地址。默认是发往本机的127.0.0.1的IP；在分布式仿真时，也可以指定192.168打头的局域网电脑IP；也可以使用255.255.255.255的广播地址（会干扰网络其他电脑）
    Com: 与Pixhawk的连接模式。
        Com='udp'，表示使用默认的udp模式接收数据，这种模式下，是接收CopterSim转发的PX4的MAVLink消息（或UDP_full,simple）消息包
                 使用port+1端口收和port端口发（例如，1号飞机是20101端口收，20100端口发，与CopterSim对应）。
        Com='COM3'（Widnows下）或 Com='/dev/ttyUSB0'（Linux系统，也可能是ttyS0、ttyAMA0等），表示通过USB线（或者数传）连接飞控，使用默认57600的波特率。注意：波特率使用port口设置，默认port=0，会重映射为57600

    port: UDP模式下默认情况下设为0，会自动根据IP填充，按平台规则，port=20100+CopterID*2-2。如果这里赋值大于0，则会强制使用port定义的端口。
          COM模式下，Port默认表示波特率self.baud=port。如果port=0，则会设置self.baud=57600


    接口示例：
    UDP模式
    PX4MavCtrler(1) # 默认IP
    PX4MavCtrler(1,'192.168.31.24') # 指定IP，用于远程控制

    串口模式
    PX4MavCtrler(1,'127.0.0.1','com1',57600) # 指定串口，并设置波特率

    """
    ## @brief 构造函数。
    # @param CopterID 初始化时设置的无人机ID，默认为：1。
    # @param ip ip地址，默认为：127.0.0.1。
    # @param port IP端口，默认为：0。
    
    
    def __init__(self, CopterID=1,ip='127.0.0.1',Com='udp',port=0):
        ##  @var PX4MavCtrler.isCom
        # UDP模式解析
        self.isCom = False
        self.Com = Com
        ## @var PX4MavCtrler.baud
        # COM波特率。
        self.baud = 115200
        self.ip = ip
        self.CopterID = CopterID
        self.port = 20100+self.CopterID*2-2
        ## @var PX4MavCtrler.rosName 
        # ROS名称
        self.rosName="mavros"
        if self.CopterID>1:
            self.rosName="mavros"+str(self.CopterID)


        if (Com=='udp' or Com=='UDP' or Com=='Udp') and CopterID>10000: # 如果是UDP通信模式
            # 兼容旧版协议，如果ID是20100等端口输入，则自动计算CopterID
            self.port=CopterID
            self.CopterID = int((CopterID-20100)/2)+1

        ## @var PX4MavCtrler.ComName 
        # 串口名称，串口连接模式解析。
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
        
        ## @var PX4MavCtrler.imu 
        # IMU数据。
        ## @var PX4MavCtrler.gps 
        # GPS数据。
        ## @var PX4MavCtrler.local_pose 
        # 本地位置量
        ## @var PX4MavCtrler.mavros_state 
        # MAVROS状态
        ## @var PX4MavCtrler.current_heading 
        # 当前机头状态
        ## @var PX4MavCtrler.local_vel 
        # 本地速度量
        ## @var PX4MavCtrler.arm_state 
        # 解锁状态
        ## @var PX4MavCtrler.offboard_state 
        # Offboard模式状态
        ## @var PX4MavCtrler.received_imu 
        # 接收到的IMU数据
        ## @var PX4MavCtrler.frame 
        # 坐标系名称
        ## @var PX4MavCtrler.state 
        # 无人机状态量
        ## @var PX4MavCtrler.command 
        # 指令
        ## @var PX4MavCtrler.offCmd 
        # mavros_msgs包中的目标位置信息。
        ## @var PX4MavCtrler.isInOffboard 
        # 是否进入Offboard模式的标志位。
        self.imu = None
        self.gps = None
        self.local_pose = None
        self.mavros_state = None
        self.current_heading = None
        self.local_vel = None
        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"
        self.state = None
        self.command = TwistStamped()
        self.offCmd = PositionTarget()
        self.offCmd.header.frame_id = "world"
        self.isInOffboard = False
        
        ## @var PX4MavCtrler.uavAngEular 
        # 无人机欧拉角状态量
        ## @var PX4MavCtrler.uavAngRate 
        # 无人机欧拉角速率状态量
        ## @var PX4MavCtrler.uavPosNED 
        # 无人机北东地位置状态量
        ## @var PX4MavCtrler.uavVelNED 
        # 无人机北东地速度状态量
        ## @var PX4MavCtrler.uavAngQuatern 
        # 无人机姿态角四元数状态量
        ## @var PX4MavCtrler.uavGlobalPos 
        # 从PX4转移到UE4地图的估计全球位置
        ## @var PX4MavCtrler.trueGpsUeCenter 
        # 真实全球GPS位置
        ## @var PX4MavCtrler.geo 
        # 地理坐标转换的模块
        ## @var PX4MavCtrler.count 
        # 软件在环仿真计数
        ## @var PX4MavCtrler.countHil 
        # 硬件在环仿真计数
        ## @var PX4MavCtrler.hasInit 
        # 初始化是否完成标志位
        self.uavAngEular = [0, 0, 0]
        self.uavAngRate = [0, 0, 0]
        self.uavPosNED = [0, 0, 0]
        self.uavVelNED = [0, 0, 0]
        self.uavAngQuatern = [0,0,0,0]
        self.uavGlobalPos = [0, 0, 0] # Estimated global position from PX4 that transferred to UE4 map
        self.trueGpsUeCenter=[40.1540302,116.2593683,50]
        self.geo = EarthModel.EarthModel()
        self.count = 0
        self.countHil=0
        self.hasInit=False
        
        
        '''
        ros subscribers
        '''
        
        if is_use_ros1:
            
            ## @var PX4MavCtrler.local_pose_sub 
            # 订阅ROS中的本地位置
            ## @var PX4MavCtrler.local_vel_sub 
            # 订阅ROS中的本地速度
            ## @var PX4MavCtrler.mavros_sub 
            # 订阅mavros包中消息
            ## @var PX4MavCtrler.gps_sub 
            # 订阅ROS中的GPS信息
            ## @var PX4MavCtrler.imu_sub 
            # 订阅ROS中的IMU信息
            self.local_pose_sub = rospy.Subscriber("/"+self.rosName+"/local_position/pose", PoseStamped, self.local_pose_callback)
            self.local_vel_sub = rospy.Subscriber("/"+self.rosName+"/local_position/velocity", TwistStamped, self.local_vel_callback)
            self.mavros_sub = rospy.Subscriber("/"+self.rosName+"/state", State, self.mavros_state_callback)
            self.gps_sub = rospy.Subscriber("/"+self.rosName+"/global_position/global", NavSatFix, self.gps_callback)
            self.imu_sub = rospy.Subscriber("/"+self.rosName+"/imu/data", Imu, self.imu_callback)
            
            '''
            ros publishers
            '''
            ## @var PX4MavCtrler.vel_pub 
            # 发布期望速度数据到ROS中
            ## @var PX4MavCtrler.vel_raw_pub 
            # 发布本地原始期望速度数据到ROS中
            ## @var PX4MavCtrler.mav_raw_pub 
            # 发布原始MAVLink消息到ROS中
            self.vel_pub = rospy.Publisher("/"+self.rosName+"/setpoint_velocity/cmd_vel", TwistStamped,queue_size=10)
            self.vel_raw_pub = rospy.Publisher("/"+self.rosName+"/setpoint_raw/local", PositionTarget, queue_size=10)
            self.mav_raw_pub = rospy.Publisher("/mavlink/to",Mavlink, queue_size=10)
            
            '''
            ros services
            '''
            ## @var PX4MavCtrler.armService 
            # 创建解锁服务代理
            ## @var PX4MavCtrler.flightModeService 
            # 创建飞行模式服务代理
            ## @var PX4MavCtrler.setparamService 
            # 创建参数设置服务代理
            ## @var PX4MavCtrler.sendCmdLongService 
            # 创建发送长格式的命令给无人机服务代理
            self.armService = rospy.ServiceProxy("/"+self.rosName+"/cmd/arming", CommandBool)
            self.flightModeService = rospy.ServiceProxy("/"+self.rosName+"/set_mode", SetMode)
            self.setparamService = rospy.ServiceProxy("/"+self.rosName+"/param/set", ParamSet)
            self.sendCmdLongService = rospy.ServiceProxy("/"+self.rosName+"/cmd/command", CommandLong)
            print("Px4 Controller Initialized!")
            
        else:
            
            try:
                rclpy.init()
            except:
                if rclpy.ok():
                    print("Already init.")
                else:
                    print("init fail")
            ## @var PX4MavCtrler.ros_node 
            # 创建ROS节点
            ## @var PX4MavCtrler.executor 
            # 创建一个多线程执行器
            ## @var PX4MavCtrler.t1 
            # 创建一个线程并开始执行
            self.ros_node = Node("RflyRos"+str(self.CopterID))
            self.executor = rclpy.executors.MultiThreadedExecutor()
            self.executor.add_node(self.ros_node )

            self.t1 = threading.Thread(target=self.executor.spin, args=())
            self.t1.start()
            
            #rclpy.executors.MultiThreadedExecutor().add_node(self.ros_node)
            # Configure QoS profile for publishing and subscribing
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.local_pose_sub = self.ros_node.create_subscription(PoseStamped, "/"+self.rosName+"/local_position/pose", self.local_pose_callback, SENSOR_QOS)
            self.local_vel_sub = self.ros_node.create_subscription(TwistStamped, "/"+self.rosName+"/local_position/velocity_local", self.local_vel_callback, SENSOR_QOS)
            self.mavros_sub = self.ros_node.create_subscription(State, "/"+self.rosName+"/state", self.mavros_state_callback, qos_profile)
            self.gps_sub = self.ros_node.create_subscription(NavSatFix, "/"+self.rosName+"/global_position/global", self.gps_callback, SENSOR_QOS)
            self.imu_sub = self.ros_node.create_subscription(Imu, "/"+self.rosName+"/imu/data", self.imu_callback, SENSOR_QOS)  

            '''
            ros publishers
            '''
            self.vel_pub = self.ros_node.create_publisher(TwistStamped, "/"+self.rosName+"/setpoint_velocity/cmd_vel", 10)
            self.vel_raw_pub = self.ros_node.create_publisher(PositionTarget, "/"+self.rosName+"/setpoint_raw/local", 10)
            self.mav_raw_pub = self.ros_node.create_publisher(Mavlink, "/uas"+str(self.CopterID)+"/mavlink_sink", 10)

            '''
            ros services
            '''
            self.armService = self.ros_node.create_client(CommandBool,"/"+self.rosName+"/cmd/arming")
            self.flightModeService = self.ros_node.create_client(SetMode,"/"+self.rosName+"/set_mode")
            self.setparamService = self.ros_node.create_client(ParamSet,"/"+self.rosName+"/param/set")
            self.sendCmdLongService = self.ros_node.create_client(CommandLong,"/"+self.rosName+"/cmd/command")
            print("Px4 Controller Initialized!")

        ## @var PX4MavCtrler.f 
        # MAVLink初始化
        ## @var PX4MavCtrler.mav0 
        # 发送任意mavlink消息
        self.f = fifo
        self.mav0 = mavlink2.MAVLink(self.f,255,1)

    ## @brief 将传入的字节序列（payload_bytes）转换成MAVLink协议中的payload64格式。
    # @param payload_bytes 字节序列
    # @return MAVLink协议中的payload64格式。
    def convert_to_payload64(self, payload_bytes):
        payload_bytes = bytearray(payload_bytes)
        payload_len = len(payload_bytes)
        payload_octets = payload_len / 8
        if payload_len % 8 > 0:
            payload_octets += 1
            payload_bytes += b'\0' * (8 - payload_len % 8)

        return struct.unpack('<%dQ' % payload_octets, payload_bytes)

    ## @brief 将 pymavlink 库中的一个MAVLink消息对象转换为ROS消息格式。
    # @param mavmsg MAVLink消息对象
    # @param header 消息头对象
    # @return 创建的Mavlink对象。
    def convert_to_rosmsg(self, mavmsg,header):
        """
        Convert pymavlink message to Mavlink.msg

        Currently supports MAVLink v1.0 only.
        """
        hdr = mavmsg.get_header()
        
        mav_raw = Mavlink(
                header=header,
                framing_status=Mavlink.FRAMING_OK,
                magic=Mavlink.MAVLINK_V20,
                len=hdr.mlen,
                incompat_flags=hdr.incompat_flags,
                compat_flags=hdr.compat_flags,
                sysid=hdr.srcSystem,
                compid=hdr.srcComponent,
                msgid=hdr.msgId,
                checksum=mavmsg.get_crc(),
                payload64=self.convert_to_payload64(mavmsg.get_payload()),
                signature=[]
            )
        # if is_use_ros1:
        #     mav_raw.signature=[]
        # else:
        #     mav_raw.signature=[]
        
        return mav_raw
    
    ## @brief 设置GPS原点
    # @param LonLatAlt GPS的经度、纬度和高度
    def setGPSOriLLA(self,LonLatAlt=[40.1540302,116.2593683,50]):
        # lla -> 纬度，经度，高度
        self.trueGpsUeCenter=LonLatAlt
        
    ## @brief 给PX4发送解锁指令。
    # @param isArm 解锁标志位。
    # @return 返回解锁信息
    def arm_px4(self,isArm):
        if is_use_ros1:
            return self.armService(isArm)
        else:
            req = CommandBool.Request()
            req.value = isArm
            return self.armService.call_async(req)
        
    ## @brief 将输入数据填充或截断到指定的长度。
    # @param data 需要填充或截断的数据，可以是列表或NumPy数组。
    # @param inLen 目标长度，即填充或截断后的长度。
    # @param fill 填充值，当数据长度小于目标长度时，会用这个值来填充数据。默认值为0。
    # @return 返回填充或截断后的数据
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
    
    ## @brief 仿真初始化，启动无人机仿真循环
    def InitMavLoop(self):
        if self.isCom:
            the_connection = mavutil.mavlink_connection(self.port,self.baud)
            the_connection.recv_match(
                    type=['HEARTBEAT'],
                    blocking=True)
            self.tgtSys=the_connection.target_system
            the_connection.close()
            print(self.tgtSys)
            if is_use_ros1:
                cmdStr = 'roslaunch mavros px4.launch tgt_system:='+str(self.tgtSys)+' fcu_url:="serial://'+self.port+':'+str(self.baud)+'"'
            else:
                cmdStr = 'ros2 launch mavros px4.launch tgt_system:='+str(self.tgtSys)+' fcu_url:="serial://'+self.port+':'+str(self.baud)+'"'
        else:
            # the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:'+str(self.port+1))

            # the_connection.recv_match(
            #         type=['HEARTBEAT'],
            #         blocking=True)
            self.tgtSys=self.CopterID
            #the_connection.close()
            print(self.tgtSys)
            if is_use_ros1:
                cmdStr = 'roslaunch mavros px4.launch tgt_system:='+str(self.tgtSys)+' fcu_url:="udp://:'+str(int(self.port)+1)+'@'+self.ip+':'+str(self.port)+'"'
            else:
                cmdStr = 'ros2 launch mavros px4.launch tgt_system:='+str(self.tgtSys)+' fcu_url:="udp://:'+str(int(self.port)+1)+'@'+self.ip+':'+str(self.port)+'"'
                
        print(cmdStr)
        self.child = subprocess.Popen(cmdStr,
                         shell=True,
                         stdout=subprocess.PIPE)

        time.sleep(10)
        print(self.child.poll())
        
        if is_use_ros1 and not self.hasInit:
            self.hasInit=True
            try:
                rospy.init_node("RflyRos")
            except:
                if not rospy.is_shutdown():
                    print("Already init.")
                else:
                    print("init fail")
            time.sleep(1)
            
    ## @brief 发送开始仿真信息
    # @param copterID 无人机ID，默认为：-1。
    def sendStartMsg(self,copterID=-1):
        buf = struct.pack("3i",1234567890,1,copterID)
        self.udp_socket.sendto(buf, ('224.0.0.10', 20007)) 
        print("Send start Msg")
        time.sleep(0.03)
        
    ## @brief 等待个特定的启动消息，每个无人机或节点等待接收特定的启动信号以开始仿真
    def waitForStartMsg(self):
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
                    #print(len(buf[0:12]))
                    checksum,isStart,ID=struct.unpack('3i',buf)
                    if checksum==1234567890 and isStart:
                        if ID<0 or ID==self.CopterID:
                            print('Got start Msg, continue to run.')
                            break
            except:
                print("Error to listen to Start Msg!")
                sys.exit(0)
                
    ## @brief 发送解锁指令
    def SendMavArm(self, isArm):
        if self.arm_px4(isArm):
            return True
        else:
            if isArm:
                print("Vehicle disarming failed!")
            else:
                print("Vehicle arming failed!")
            return False

    ## @brief 初始化Offboard模式
    def initOffboard(self):

        if is_use_ros1 and not self.hasInit:
            self.hasInit=True
            try:
                rospy.init_node("RflyRos"+str(self.CopterID))
            except:
                if not rospy.is_shutdown():
                    print("Already init.")
                else:
                    print("init fail")
            time.sleep(1)
            
        self.isInOffboard = True
        print("Offboard Started.")
        
        self.SendVelNED(0, 0, 0, 0)

        # 发送指定mavros消息
        self.sendMavSetParam('NAV_RCL_ACT',0,'INT')
        self.sendMavSetParam('NAV_DLL_ACT', 0, 'INT')
        self.sendMavSetParam('COM_RCL_EXCEPT', 3, 'INT')
            
        self.t2 = threading.Thread(target=self.OffboardLoop, args=())
        self.t2.start()
        # 等待offboard消息发一阵，让PX4认为通信健康

        
        time.sleep(1)

        # 发送命令，且换Offboard模式
        self.offboard_state = self.offboard()

        # 发送命令，解锁飞控
        time.sleep(0.2)
        self.arm_state = self.arm()
        
        
    ## @brief 启动Offboard模式循环
    def OffboardLoop(self):
        
        # ROS1和ROS2创建rate的方式不同
        if is_use_ros1:
            rate = rospy.Rate(30)
        else:  
            rate = self.ros_node.create_rate(30)
            #self.timer = self.ros_node.create_timer(1/30.0, self.offboardSend) # 30Hz频率定时器
            #rclpy.spin(self.ros_node) # 开启定时器            
        
        while True:
            if not self.isInOffboard:
                break
            
            isRosOK = True
            if is_use_ros1:
                isRosOK = not rospy.is_shutdown()
            else:
                isRosOK = rclpy.ok()
            
            if isRosOK:
                if is_use_ros1:
                    self.offCmd.header.stamp = rospy.Time.now()
                    self.offCmd.header.seq = self.count # ROS2 没有header.seq字段
                    self.count = self.count+1
                else:
                    self.offCmd.header.stamp = self.ros_node.get_clock().now().to_msg()
                    #rclpy.spin_once(self.ros_node)
                
                self.vel_raw_pub.publish(self.offCmd)
                #self.SendHILCtrlMsg()
                rate.sleep()
            else:
                break
        print("Offboard Stoped.")
        
    ## @brief 结束Offboard模式
    def endOffboard(self):
        self.isInOffboard = False
        self.t2.join()
        if not is_use_ros1:
            self.t1.join()
            try:
                rclpy.shutdown()
            except:
                if rclpy.ok():
                    print('failed to shutdown')
                else:
                    print('Already shutdown')
                    
    ## @brief 停止仿真
    def stopRun(self):
        self.child.kill()
        self.child.terminate()
        print('Please close all Terminal windows to close')
        
    ## @brief 根据提供的布尔列表 EnList 计算一个类型掩码（type mask）。
    #  @param EnList 布尔列表
    #  @return 返回计算后的类型掩码
    def calcTypeMask(self,EnList):
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
        return y
    
    ## @brief 发送北东地坐标系下的速度指令。
    #  @param vx X轴速度
    #  @param vy Y轴速度
    #  @param vz Z轴速度
    #  @param yawrate 偏航角速率
    def SendVelNED(self,vx=math.nan,vy=math.nan,vz=math.nan,yawrate=math.nan):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_LOCAL_NED
        self.offCmd.type_mask = self.calcTypeMask([0,1,0,0,0,1])
        self.offCmd.velocity.x = float(vx)
        self.offCmd.velocity.y = float(-vy)
        self.offCmd.velocity.z = float(-vz)
        self.offCmd.yaw_rate = float(-yawrate)

    ## @brief 发送机体坐标系下的速度指令。
    #  @param vx X轴速度
    #  @param vy Y轴速度
    #  @param vz Z轴速度
    #  @param yawrate 偏航角速率
    def SendVelFRD(self,vx=math.nan,vy=math.nan,vz=math.nan,yawrate=math.nan):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_BODY_NED
        self.offCmd.type_mask = self.calcTypeMask([0,1,0,0,0,1])
        self.offCmd.velocity.x = float(vx)
        self.offCmd.velocity.y = float(-vy)
        self.offCmd.velocity.z = float(-vz)
        self.offCmd.yaw_rate = float(-yawrate)

    ## @brief 发送北东地坐标系下的位置指令。
    #  @param x X轴位置
    #  @param y Y轴位置
    #  @param z Z轴位置
    #  @param yaw 偏航角角度
    def SendPosNED(self,x=math.nan,y=math.nan,z=math.nan,yaw=math.nan):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_LOCAL_NED
        self.offCmd.type_mask = self.calcTypeMask([1,0,0,0,1,0])
        self.offCmd.position.x = float(x)
        self.offCmd.position.y = float(-y)
        self.offCmd.position.z = float(-z)
        if not math.isnan(yaw):
            self.offCmd.yaw = float(-yaw + math.pi/2)
        else:
            self.offCmd.yaw=float(yaw)
            
            
    ## @brief 发送北东地坐标系下的位置、速度指令，位置和速度共同控制接口，如果某个通道不想控制，设置为nan即可。
    #  @param PosE X、Y、Z轴位置指令
    #  @param VelE X、Y、Z轴速度指令
    #  @param yaw 偏航角角度
    #  @param yawrate 偏航角速率
    def SendPosVelNED(self,PosE=[math.nan]*3,VelE=[math.nan]*3,yaw=math.nan,yawrate=math.nan):
        # 位置和速度共同控制接口，如果某个通道不想控制，设置为nan即可。

        PosE=self.fillList(PosE,3,math.nan)
        VelE=self.fillList(VelE,3,math.nan)

        x=PosE[0]
        y=PosE[1]
        z=PosE[2]

        vx=VelE[0]
        vy=VelE[1]
        vz=VelE[2]

        self.offCmd.coordinate_frame = self.offCmd.FRAME_LOCAL_NED

        # 启用位置和速度共同控制
        self.offCmd.type_mask = self.calcTypeMask([1,1,0,0,1,1])
        self.offCmd.position.x = float(x)
        self.offCmd.position.y = float(-y)
        self.offCmd.position.z = float(-z)
        self.offCmd.velocity.x = float(vx)
        self.offCmd.velocity.y = float(-vy)
        self.offCmd.velocity.z = float(-vz)
        self.offCmd.yaw_rate = float(-yawrate)
        if not math.isnan(yaw):
            self.offCmd.yaw = float(-yaw + math.pi/2)
        else:
            self.offCmd.yaw=float(math.nan) # 设为nan表示不控制
    
    ## @brief 处理ROS中订阅的本地位置话题的消息，更新无人机的位置和姿态信息
    #  @param msg 从ROS中接收到的消息对象
    def local_pose_callback(self, msg):
        self.local_pose = msg
        ang = self.q2Euler(self.local_pose.pose.orientation)
        q=self.local_pose.pose.orientation
        self.uavAngQuatern = [q.w, q.x, q.y, q.z]
        self.uavAngEular[0] = ang[1]
        self.uavAngEular[1] = ang[0]
        self.uavAngEular[2] = self.yawSat(-ang[2]+math.pi/2)

        self.uavPosNED[0]=self.local_pose.pose.position.y
        self.uavPosNED[1]=self.local_pose.pose.position.x
        self.uavPosNED[2]=-self.local_pose.pose.position.z
    
    ## @brief 处理ROS中订阅的本地速度话题的消息，更新无人机的速度和角速率信息
    #  @param msg 从ROS中接收到的消息对象
    def local_vel_callback(self, msg):
        self.local_vel = msg

        self.uavVelNED[0] = self.local_vel.twist.linear.y
        self.uavVelNED[1] = self.local_vel.twist.linear.x
        self.uavVelNED[2] = -self.local_vel.twist.linear.z

        self.uavAngRate[0] = self.local_vel.twist.angular.y
        self.uavAngRate[1] = self.local_vel.twist.angular.x
        self.uavAngRate[2] = -self.local_vel.twist.angular.z
    
    ## @brief 处理ROS中订阅的MAVROS状态主题的消息，更新无人机的飞行模式信息。
    #  @param msg 从ROS中接收到的消息对象
    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode
        
    ## @brief 处理ROS中订阅的IMU话题的消息，更新无人机的IMU数据
    #  @param msg 从ROS中接收到的消息对象
    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation)
        self.received_imu = True
        
    ## @brief 处理ROS中订阅的GPS话题的消息，更新无人机的GPS数据
    #  @param msg 从ROS中接收到的消息对象
    def gps_callback(self, msg):
        self.gps = msg
        LLA=[msg.latitude,msg.longitude,msg.altitude]
        self.uavGlobalPos = self.geo.lla2ned(LLA,self.trueGpsUeCenter)

    ## @brief 从输入的四元数（q）来计算偏航角
    #  @param q 四元数
    def q2yaw(self, q):
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z
        math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
        
    ## @brief 从输入的四元数（q）来计算欧拉角
    #  @param q 四元数
    #  @return 返回计算后的姿态角。
    def q2Euler(self,q):
        w,x,y,z= q.w, q.x, q.y, q.z
        roll = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch = math.asin(2*(w*y-z*x))
        yaw = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
        return [roll,pitch,yaw]

    # send MAVLink command to Pixhawk to Arm/Disarm the drone
    ## @brief 通过MAVLink发送解锁指令给PX4
    #  @param isArm 解锁标志位
    #  - 0表示上锁
    #  - 1表示解锁
    def SendMavArm(self, isArm=1):
        if isArm:
            self.arm()
        else:
            self.disarm()
            
    ## @brief 检查PX4的解锁状态
    #  @return 返回解锁标准的真假值
    def arm(self):
        if self.arm_px4(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False
        
    ## @brief 检查PX4的上锁状态
    #  @return 返回上锁标准的真假值
    def disarm(self):
        if self.arm_px4(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False
    ## @brief 设置PX4无人机进入Offboard控制模式，尝试将无人机切换到Offboard模式，这是一种允许外部控制命令覆盖无人机内部控制的模式。
    # @return 返回是否成功进入Offboard控制模式的布尔值
    def offboard(self):
        
        if is_use_ros1:
            mode = self.flightModeService(custom_mode='OFFBOARD')
        else:
            req = SetMode.Request()
            req.custom_mode='OFFBOARD'
            mode = self.flightModeService.call_async(req)
            
        if mode:
            return True
        else:
            print("Vechile Offboard failed")
            return False
        
    def land(self):
        if is_use_ros1:
            mode = self.flightModeService(custom_mode='AUTO.LAND')
        else:
            req = SetMode.Request()
            req.custom_mode='AUTO.LAND'
            mode = self.flightModeService.call_async(req)
            
        if mode:
            return True
        else:
            print("Vechile LAND failed")
            return False
    ## @brief 对偏航角进行饱和处理，此函数接收一个偏航角值，并确保其在 -π/2 到 π/2 弧度（或等效的 -90 到 90 度）的范围内。如果输入的偏航角超出这个范围，函数会相应地调整它，使其不超过阈值。
    # @param yaw 输入的偏航角，以弧度为单位
    # @return 返回饱和后的偏航角值
    def yawSat(self,yaw):
        yawOut=yaw
        if yaw>math.pi/2:
            yawOut=yaw-math.pi/2
        if yaw<-math.pi/2:
            yawOut=yaw+math.pi/2
        return yawOut
    
    ## @brief 发送MAVLink设置参数请求，用于通过MAVLink协议设置无人机的参数。根据参数类型，它可以设置整型或实型参数。
    # @param param_id_str 参数的名称字符串标识符
    # @param param_value 参数的值，可以是整型或实型
    # @param param_type 参数的类型，'INT' 表示整型，其他值表示实型
    def sendMavSetParam(self,param_id_str, param_value, param_type):
        # param=ParamSet
        # print(param)
        # param.param_id=param_id_str
        # if param_type=='INT':
        #     param.value.integer=param_value
        # else:
        #     param.value.real=param_value
        # if param_type=='INT':
        #     #self.setparamService({"param_id":param_id_str,"value":{"integer":param_value}})
        #     self.setparamService({param_id=param_id_str,value.integer=param_value})
        # else:
        #     #self.setparamService({"param_id":param_id_str,"value":{"real":param_value}})
        #     self.setparamService({param_id=param_id_str,value.real=param_value})
        try:
            if param_type=='INT':
                val = ParamValue(integer=param_value, real=float(0))
            else:
                val = ParamValue(integer=0, real=float(param_value))

            if is_use_ros1:
                self.setparamService(param_id=param_id_str, value=val)
            else:
                req = ParamSet.Request()
                req.param_id = param_id_str
                req.value = val
                self.setparamService.call_async(req)
        except ImportError:
            print("Faild to SendCMD.")
    ## @brief 发送MAVLink长格式命令，该函数用于通过MAVLink协议发送长格式的命令给无人机。命令可以包括一系列的参数，用于指定命令的具体行为。
    # @param command 要发送的MAVLink命令的ID
    # @param param1 命令的第一个参数，默认为0
    # @param param2 命令的第二个参数，默认为0
    # @param param3 命令的第三个参数，默认为0
    # @param param4 命令的第四个参数，默认为0
    # @param param5 命令的第五个参数，默认为0
    # @param param6 命令的第六个参数，默认为0
    # @param param7 命令的第七个参数，默认为0
    def SendMavCmdLong(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        if is_use_ros1:
            self.sendCmdLongService(broadcast=False,confirmation=0,command=command,param1=param1,param2=param2,param3=param3,param4=param4,param5=param5,param6=param6,param7=param7)
        else:
            req=CommandLong.Request()
            req.broadcast=False
            req.confirmation=0
            req.command=command
            req.param1=param1
            req.param2=param2
            req.param3=param3
            req.param4=param4
            req.param5=param5
            req.param6=param6
            req.param7=param7
            self.sendCmdLongService.call_async(req)

    # send hil_actuator_controls message to Pixhawk (for rfly_ctrl uORB message)
    ## @brief 发送硬件在环仿真的hil_actuator_controls消息（消息定义可见：https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS）
    #   代码的核心是创建一个Mavlink消息，并转换为ros_mavlink消息，在Python中比较复杂
    # 在C语言中很简单，利用\opt\ros\[noetic或foxy]\include\mavros_msgs\mavlink_convert.h中的
    # bool convert(const Mavlink & rmsg, mavlink_message_t & mmsg)函数可以轻松转换，然后发布即可
    # RflySim平台需要借用hil_actuator_controls来将数据传到rfly_ctrl、rfly_ctrl1和rfly_ctrl2中
    # 通过mavros的接口，将一条mavlink的buf包消息，传给飞控
    # @param ctrls 16维控制量
    # @param idx 索引值，用于选择不同的uORB消息ID，默认为0
    def SendHILCtrlMsg(self,ctrls=[0]*16,idx=0):
        """ Send hil_actuator_controls command to PX4, which will be transferred to uORB message rfly_ctrl
        https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS
        """
        mode=1
        flag=1
        if idx<0.5:
            mode=1 # rfly_ctrl消息
        elif idx<1.5: # rfly_ctrl1消息
            mode=101
        elif idx<2.5: # rfly_ctrl2消息
            mode=201
        else:
            mode=1 # rfly_ctrl消息
        
        time_boot_ms = int(time.time()*1000)
        controls = self.fillList(ctrls,16,float(0))
        
        # 打包一个mavlink的包
        msg = self.mav0.hil_actuator_controls_encode(time_boot_ms,controls,int(mode),int(flag))
        msg.pack(self.mav0)

        if is_use_ros1:
            # 包头数据填充
            head = Header(stamp=rospy.get_rostime())
            head.seq = self.countHil # ROS2 没有header.seq字段
            self.countHil = self.countHil+1
            
            # 将mavlinkbuf包转为ros包
            rosmsg = self.convert_to_rosmsg(msg,head)
        else:
            # 包头数据填充
            stamp = self.ros_node.get_clock().now().to_msg()
            head=Header(stamp=stamp)
            
            # 将mavlinkbuf包转为ros包
            rosmsg = self.convert_to_rosmsg(msg,head)

        # 注意，上面代码的核心是创建一个Mavlink消息，并转换为ros_mavlink消息，在Python中比较复杂
        # 在C语言中很简单，利用\opt\ros\[noetic或foxy]\include\mavros_msgs\mavlink_convert.h中的
        # bool convert(const Mavlink & rmsg, mavlink_message_t & mmsg)函数可以轻松转换，然后发布即可
        # RflySim平台需要借用hil_actuator_controls来将数据传到rfly_ctrl、rfly_ctrl1和rfly_ctrl2中

        # 通过mavros的接口，将一条mavlink的buf包消息，传给飞控
        self.mav_raw_pub.publish(rosmsg)


        