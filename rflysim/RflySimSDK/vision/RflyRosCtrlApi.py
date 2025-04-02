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

#from mavros import mavlink as mavlink0
from pymavlink.dialects.v20 import common as mavlink2
from pymavlink import mavutil
import platform
import subprocess
#from pymavlink.dialects.v20 import common as mavlink2

## @file
#  这是一个通过ros系统与RflySim3D进行交互的模块。
#  @anchor RflyRosCtrlApi接口库文件
#  对应例程链接见

##  @brief 判断是否在Linux环境下
isLinux = False
##  @brief 判断ROS是否安装好
isRosOk = False
##  @brief 判断ROS的版本
s_use_ros1 = False
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
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            is_use_ros1 = False
        isRosOk=True

    except ImportError:
        print("Faild to load ROS labs")

# define a class for MAVLink initialization
## @class fifo
#  @brief fifo结构体类。用于MAVLink初始化
class fifo(object):
    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)


## @class RflyRosCtrlApi
#  @brief RflyRosCtrlApi结构体类。提供了与无人机进行通信和控制的完整接口，支持多种通信模式，并集成了ROS系统
class RflyRosCtrlApi:
    """创建一个通信实例
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
    
    ## @brief RflyRosCtrlApi的构造函数 
    #  @param 初始化传输模式，波特率，ip等等参数 
    #  
    # 分为ros1和ros2两种，定义一些ROS系统中的服务、订阅和发布
    def __init__(self, CopterID=1,ip='127.0.0.1',Com='udp',port=0):

        ## @var RflyRosCtrlApi.isCom
        #  @brief 用于标识当前通信模式是否为串口模式
        self.isCom = False
        ## @var RflyRosCtrlApi.Com
        #  @brief 存储传入的通信模式参数
        self.Com = Com
        ## @var RflyRosCtrlApi.baud
        #  @brief 波特率，默认设置为115200
        self.baud = 115200
        ## @var RflyRosCtrlApi.ip
        #  @brief 存储数据发送的IP地址
        self.ip = ip

        ## @var RflyRosCtrlApi.CopterID
        #  @brief 无人机的ID号，用于区分不同的无人机
        self.CopterID = CopterID
        ## @var RflyRosCtrlApi.port
        #  @brief  端口号，默认根据CopterID计算而得
        self.port = 20100+self.CopterID*2-2

        self.rosName="mavros"
        if self.CopterID>1:
            self.rosName="mavros"+str(self.CopterID)



        # UDP模式解析
        if (Com=='udp' or Com=='UDP' or Com=='Udp') and CopterID>10000: # 如果是UDP通信模式
            # 兼容旧版协议，如果ID是20100等端口输入，则自动计算CopterID
            self.port=CopterID
            self.CopterID = int((CopterID-20100)/2)+1

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

        ## @var RflyRosCtrlApi.imu
        #  @brief 存储无人机的IMU数据
        self.imu = None
        ## @var RflyRosCtrlApi.gps
        #  @brief 存储无人机的GPS数据
        self.gps = None
        ## @var RflyRosCtrlApi.local_pose
        #  @brief 存储无人机的本地位姿数据
        self.local_pose = None
        ## @var RflyRosCtrlApi.mavros_state
        #  @brief 存储无人机的当前状态信息
        self.mavros_state = None
        ## @var RflyRosCtrlApi.current_heading
        #  @brief 存储无人机当前的航向角信息
        self.current_heading = None
        ## @var RflyRosCtrlApi.local_vel
        #  @brief 存储无人机的本地速度数据
        self.local_vel = None
        ## @var RflyRosCtrlApi.arm_state
        #  @brief 表示无人机的解锁/上锁状态
        self.arm_state = False
        ## @var RflyRosCtrlApi.offboard_state
        #  @brief 表示无人机是否处于Offboard模式
        self.offboard_state = False
        ## @var RflyRosCtrlApi.received_imu
        #  @brief 表示是否接收到IMU数据
        self.received_imu = False
        ## @var RflyRosCtrlApi.frame
        #  @brief 定义数据坐标系的参考框架
        self.frame = "BODY"

        ## @var RflyRosCtrlApi.state
        #  @brief 存储自定义状态信息
        self.state = None
        ## @var RflyRosCtrlApi.command
        #  @brief 存储速度控制命令
        self.command = TwistStamped()
        ## @var RflyRosCtrlApi.offCmd
        #  @brief 存储位置控制命令
        self.offCmd = PositionTarget()
        ## @var RflyRosCtrlApi.header.frame_id
        #  @brief 设置位置控制命令的坐标系
        self.offCmd.header.frame_id = "world"

        ## @var RflyRosCtrlApi.isInOffboard
        #  @brief  表示是否处于Offboard模式中
        self.isInOffboard = False

        ## @var RflyRosCtrlApi.uavAngEular
        #  @brief  存储无人机的欧拉角信息
        self.uavAngEular = [0, 0, 0]
        ## @var RflyRosCtrlApi.uavAngRate
        #  @brief 存储无人机的角速度信息
        self.uavAngRate = [0, 0, 0]
        ## @var RflyRosCtrlApi.uavPosNED
        #  @brief 存储无人机的本地NED坐标系位置信息
        self.uavPosNED = [0, 0, 0]
        ## @var RflyRosCtrlApi.uavVelNED
        #  @brief 存储无人机的本地NED坐标系速度信息
        self.uavVelNED = [0, 0, 0]
        ## @var RflyRosCtrlApi.uavAngQuatern
        #  @brief 存储无人机的四元数姿态信息
        self.uavAngQuatern = [0,0,0,0]

        ## @var RflyRosCtrlApi.count
        #  @brief 计数器，初始化为0
        self.count = 0
        ## @var RflyRosCtrlApi.countHil
        #  @brief HIL（硬件在环）模式的计数器，初始化为0
        self.countHil=0
        '''
        ros subscribers
        '''
        
        if is_use_ros1:
            
            try:
                rospy.init_node("RflyRos")
            except:
                if not rospy.is_shutdown():
                    print("Already init.")
                else:
                    print("init fail")
            
            
            self.local_pose_sub = rospy.Subscriber("/"+self.rosName+"/local_position/pose", PoseStamped, self.local_pose_callback)
            self.local_vel_sub = rospy.Subscriber("/"+self.rosName+"/local_position/velocity", TwistStamped, self.local_vel_callback)
            self.mavros_sub = rospy.Subscriber("/"+self.rosName+"/state", State, self.mavros_state_callback)
            self.gps_sub = rospy.Subscriber("/"+self.rosName+"/global_position/global", NavSatFix, self.gps_callback)
            self.imu_sub = rospy.Subscriber("/"+self.rosName+"/imu/data", Imu, self.imu_callback)
            
            '''
            ros publishers
            '''
            self.vel_pub = rospy.Publisher("/"+self.rosName+"/setpoint_velocity/cmd_vel", TwistStamped,queue_size=10)
            self.vel_raw_pub = rospy.Publisher("/"+self.rosName+"/setpoint_raw/local", PositionTarget, queue_size=10)
            self.mav_raw_pub = rospy.Publisher("/mavlink/to",Mavlink, queue_size=10)
            
            '''
            ros services
            '''
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
            self.local_pose_sub = self.ros_node.create_subscription(PoseStamped, "/"+self.rosName+"/local_position/pose", self.local_pose_callback, qos_profile)
            self.local_vel_sub = self.ros_node.create_subscription(TwistStamped, "/"+self.rosName+"/local_position/velocity", self.local_vel_callback, qos_profile)
            self.mavros_sub = self.ros_node.create_subscription(State, "/"+self.rosName+"/state", self.mavros_state_callback, qos_profile)
            self.gps_sub = self.ros_node.create_subscription(NavSatFix, "/"+self.rosName+"/global_position/global", self.gps_callback, qos_profile)
            self.imu_sub = self.ros_node.create_subscription(Imu, "/"+self.rosName+"/imu/data", self.imu_callback, qos_profile)  

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

        # 为了发送任意mavlink消息而设定
        self.f = fifo
        self.mav0 = mavlink2.MAVLink(self.f,255,1)


    ## @brief 将payload字节转换为Mavlink的payload64格式
    # # - @anchor convert_to_payload64
    # @param payload_bytes 输入的payload字节
    # @return 返回Mavlink.payload64格式的payload
    def convert_to_payload64(self, payload_bytes):
        """
        Convert payload bytes to Mavlink.payload64
        """
        payload_bytes = bytearray(payload_bytes)
        payload_len = len(payload_bytes)
        payload_octets = payload_len / 8
        if payload_len % 8 > 0:
            payload_octets += 1
            payload_bytes += b'\0' * (8 - payload_len % 8)

        return struct.unpack('<%dQ' % payload_octets, payload_bytes)


    ## @brief 将pymavlink消息转换为ROS的Mavlink消息
    # # - @anchor convert_to_rosmsg
    # @param mavmsg pymavlink消息
    # @param header ROS消息头
    # @return mav_raw Mavlink消息
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

    ## @brief 解锁或锁定PX4无人机
    # # - @anchor arm_px4
    # @param isArm 是否解锁
    # @return 上锁或者解锁的结果    
    def arm_px4(self,isArm):
        if is_use_ros1:
            return self.armService(isArm)
        else:
            req = CommandBool.Request()
            req.value = isArm
            return self.armService.call_async(req)

    ## @brief 填充或截断列表，使其长度达到指定值  
    # # - @anchor fillList
    # @param data 输入列表
    # @param inLen 输出列表的长度
    # @param fill（默认为 0） 填充的值
    # @return 填充或截断后的列表  
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

    ## @brief  初始化Mavlink连接，并启动Mavros
    # # - @anchor InitMavLoop
    # @param 无
    # @return 无
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

    ## @brief  发送解锁或锁定命令到无人机
    # # - @anchor SendMavArm
    # @param isArm 是否解锁
    # @return 是否发送成功
    def SendMavArm(self, isArm):
        if self.arm_px4(isArm):
            return True
        else:
            if isArm:
                print("Vehicle disarming failed!")
            else:
                print("Vehicle arming failed!")
            return False

    ## @brief 初始化Offboard模式并发送初始命令
    # # - @anchor initOffboard
    # @param 无
    # @return 无
    def initOffboard(self):

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

    ## @brief 持续发送Offboard命令以维持Offboard模式
    # # - @anchor OffboardLoop
    # @param 无
    # @return 无
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

    ## @brief 结束Offboard模式，停止相关线程，并关闭ROS2节点
    # # - @anchor endOffboard
    # @param 无
    # @return 无
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
                
    ## @brief 停止运行，杀死子进程并提醒用户关闭所有终端窗口
    # # - @anchor stopRun
    # @param 无
    # @return 无
    def stopRun(self):
        self.child.kill()
        self.child.terminate()
        print('Please close all Terminal windows to close')

    ## @brief 根据输入的启用列表计算Type Mask
    # # - @anchor calcTypeMask
    # @param EnList 启用列表
    # @return 掩膜类型
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

    ## @brief 发送NED（北、东、下）坐标系下的速度命令
    # - @anchor SendVelNED
    # @param vx（默认值为 nan） x方向速度
    # @param vy（默认值为 nan） y方向速度
    # @param vz（默认值为 nan） z方向速度
    # @param yawrate（默认值为 nan） 偏航角速率
    # @return 无
    def SendVelNED(self,vx=math.nan,vy=math.nan,vz=math.nan,yawrate=math.nan):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_LOCAL_NED
        self.offCmd.type_mask = self.calcTypeMask([0,1,0,0,0,1])
        self.offCmd.velocity.x = float(vx)
        self.offCmd.velocity.y = float(-vy)
        self.offCmd.velocity.z = float(-vz)
        self.offCmd.yaw_rate = float(-yawrate)


    ## @brief 发送FRD（前、右、下）坐标系下的速度命令
    # - @anchor SendVelFRD
    # @param vx（默认值为 nan） x方向速度
    # @param vy（默认值为 nan） y方向速度
    # @param vz（默认值为 nan） z方向速度
    # @param yawrate（默认值为 nan） 偏航角速率
    # @return 无
    def SendVelFRD(self,vx=math.nan,vy=math.nan,vz=math.nan,yawrate=math.nan):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_BODY_NED
        self.offCmd.type_mask = self.calcTypeMask([0,1,0,0,0,1])
        self.offCmd.velocity.x = float(vx)
        self.offCmd.velocity.y = float(-vy)
        self.offCmd.velocity.z = float(-vz)
        self.offCmd.yaw_rate = float(-yawrate)


    ## @brief 发送NED坐标系下的位置命令
    # - @anchor SendPosNED
    # @param x（默认值为 nan） x方向位置
    # @param y（默认值为 nan） y方向位置
    # @param z（默认值为 nan） z方向位置
    # @param yaw（默认值为 nan） 偏航角
    # @return 无
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

    ## @brief 发送NED坐标系下的位置和速度命令
    # - @anchor SendPosVelNED
    # @param PosE（默认值为 [nan]*3） 位置
    # @param VelE（默认值为 [nan]*3） 速度
    # @param yaw（默认值为 nan） 偏航角
    # @param yawrate（默认值为 nan） 偏航角速率
    # @return 无
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

    ## @brief 处理本地位姿消息
    # - @anchor local_pose_callback
    # @param msg 位姿消息
    # @return 无
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

    ## @brief 处理本地速度消息
    # - @anchor local_vel_callback
    # @param msg 速度消息
    # @return 无
    def local_vel_callback(self, msg):
        self.local_vel = msg

        self.uavVelNED[0] = self.local_vel.twist.linear.y
        self.uavVelNED[1] = self.local_vel.twist.linear.x
        self.uavVelNED[2] = -self.local_vel.twist.linear.z

        self.uavAngRate[0] = self.local_vel.twist.angular.y
        self.uavAngRate[1] = self.local_vel.twist.angular.x
        self.uavAngRate[2] = -self.local_vel.twist.angular.z

    ## @brief 处理Mavros状态消息
    # - @anchor mavros_state_callback
    # @param msg 状态消息
    # @return 无
    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    ## @brief 处理IMU消息
    # - @anchor imu_callback
    # @param msg IMU消息
    # @return 无
    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation)
        self.received_imu = True

    ## @brief 处理GPS消息
    # - @anchor gps_callback
    # @param msg GPS消息
    # @return 无
    def gps_callback(self, msg):
        self.gps = msg

    ## @brief 从四元数计算偏航角
    # - @anchor q2yaw
    # @param q 四元数
    # @return 无
    def q2yaw(self, q):
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z
        math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))

    ## @brief 从四元数计算欧拉角
    # - @anchor q2Euler
    # @param q 四元数
    # @return 滚转角、俯仰角、偏航角
    def q2Euler(self,q):
        w,x,y,z= q.w, q.x, q.y, q.z
        roll = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch = math.asin(2*(w*y-z*x))
        yaw = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
        return [roll,pitch,yaw]

    # send MAVLink command to Pixhawk to Arm/Disarm the drone
    ## @brief 发送解锁或锁定命令到无人机
    # - @anchor SendMavArm
    # @param isArm（默认值为 0） 是否解锁
    # @return 无
    def SendMavArm(self, isArm=0):
        if isArm:
           self.arm()
        else:
             self.disarm()

    ## @brief 解锁无人机
    # - @anchor arm
    # @param 无
    # @return 是否解锁成功
    def arm(self):
        if self.arm_px4(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    ## @brief 锁定无人机
    # - @anchor disarm
    # @param 无
    # @return 上锁成功
    def disarm(self):
        if self.arm_px4(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    ## @brief 切换飞行模式到Offboard模式
    # - @anchor offboard
    # @param 无
    # @return 是否进入offboard模式
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

    ## @brief 对偏航角进行限制，使其保持在[-π/2, π/2]范围内
    # - @anchor yawSat
    # @param yaw 偏航角
    # @return 矫正后的偏航角
    def yawSat(self,yaw):
        yawOut=yaw
        if yaw>math.pi/2:
            yawOut=yaw-math.pi/2
        if yaw<-math.pi/2:
            yawOut=yaw+math.pi/2
        return yawOut

    ## @brief 设置MAVLink参数
    ## @anchor sendMavSetParam
    # @param param_id_str 参数ID
    # @param param_value 参数值
    # @param param_type 参数类型
    # @return 无
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

    ## @brief 发送长命令到飞行控制器
    ## @anchor sendMavCmdLong
    # @param command 命令
    # @param param1（默认值为 0） 参数1
    # @param param2（默认值为 0） 参数2
    # @param param3（默认值为 0） 参数3
    # @param param4（默认值为 0） 参数4
    # @param param5（默认值为 0） 参数5
    # @param param6（默认值为 0） 参数6
    # @param param7（默认值为 0） 参数7
    # @return 无
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
    ## @brief 发送HIL（硬件在环）控制消息到Pixhawk，这些消息会转化为uORB消息rfly_ctrl
    ## @anchor SendHILCtrlMsg
    ## @param ctrls 16个控制量
    ## @param idx（默认值为 0） 0-3，用于选择不同的uORB消息rfly_ctrl，rfly_ctrl1，rfly_ctrl2，rfly_ctrl
    ## - 0表示rfly_ctrl消息
    ## - 1表示rfly_ctrl1消息
    ## - 2表示2表示rfly_ctrl2消息
    ## - 3表示rfly_ctrl消息
    ## @return 无
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
            head = Header(stamp=rospy.get_rostime())
            head.seq = self.countHil # ROS2 没有header.seq字段
            self.countHil = self.countHil+1
            rosmsg = self.convert_to_rosmsg(msg,head)
        else:
            stamp = self.ros_node.get_clock().now().to_msg()
            head=Header(stamp=stamp)
            rosmsg = self.convert_to_rosmsg(msg,head)

        # 注意，上面代码的核心是创建一个Mavlink消息，并转换为ros_mavlink消息，在Python中比较复杂
        # 在C语言中很简单，利用\opt\ros\[noetic或foxy]\include\mavros_msgs\mavlink_convert.hpp中的
        # bool convert(const Mavlink & rmsg, mavlink_message_t & mmsg)函数可以轻松转换，然后发布即可
        # RflySim平台需要借用hil_actuator_controls来将数据传到rfly_ctrl、rfly_ctrl1和rfly_ctrl2中

        self.mav_raw_pub.publish(rosmsg)


        