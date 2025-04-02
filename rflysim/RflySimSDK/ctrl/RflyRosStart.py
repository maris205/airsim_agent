#!/usr/bin/env python
import time
import math
import os
import socket
import threading
import struct
import sys

from pymavlink import mavutil
import platform
import subprocess
import psutil

#from pymavlink.dialects.v20 import common as mavlink2

## @file
#  这是一个用于无人机ROS与mavros通信的模块
#  @anchor RflyRosStart接口库文件
#  @ref 

isLinux = False
isRosOk = False
is_use_ros1 = False
if platform.system().lower() == "linux":
    isLinux = True

    ros_version = os.getenv("ROS_VERSION")
    print("current ros environment", os.getenv("ROS_DISTRO"))
    from logging import exception
    from typing import Any
    from xml.etree.ElementTree import tostring
    import yaml
    # 加入其他的ROS库
    from std_msgs.msg import String
    import sensor_msgs.msg as sensor
    import std_msgs.msg as std_msg
    from cv_bridge import CvBridge

    try:
        if ros_version == "1":
            import rospy
            is_use_ros1 = True
        else:
            import rclpy
            from rclpy.node import Node
            from rclpy.clock import Clock
            from rclpy.duration import Duration
            from rclpy.qos import qos_profile_sensor_data
            is_use_ros1 = False
        isRosOk=True

    except ImportError:
        print("Faild to load ROS labs")



# PX4 MAVLink listen and control API and RflySim3D control API
## @class RflyRosStart
# @brief RflyRosStart结构体类.   
# @details
class RflyRosStart:

    """创建一个通信实例
    ID: 如果ID<=10000则表示飞机的CopterID号。如果ID>10000，例如20100这种，则表示通信端口号port。按平台规则，port=20100+CopterID*2-2（为了兼容旧接口的过渡定义，将来ID只表示CopterID）。
    ip: 数据向外发送的IP地址。默认是发往本机的127.0.0.1的IP；在分布式仿真时，也可以指定192.168打头的局域网电脑IP；也可以使用255.255.255.255的广播地址（会干扰网络其他电脑）
    Com: 与Pixhawk的连接模式。
        Com='udp'，表示使用默认的udp模式接收数据，这种模式下，是接收CopterSim转发的PX4的MAVLink消息（或UDP_full,simple）消息包
                 使用port+1端口收和port端口发（例如，1号飞机是20101端口收，20100端口发，与CopterSim对应）。
        Com='COM3'（Widnows下）或 Com='/dev/ttyUSB0'（Linux系统，也可能是ttyS0、ttyAMA0等），表示通过USB线（或者数传）连接飞控，使用默认57600的波特率。注意：波特率使用port口设置，默认port=0，会重映射为57600
        Com='Direct'，表示UDP直连模式（对应旧版接口的真机模式），这种模式下使用使用同一端口收发（端口号有port设置），例如Com='Direct'，port=15551，表示通过15551这一个端口来收发数据
        注意：COM模式和Direct模式下，ID只表示飞机的ID号，而不表示端口号
    port: UDP模式下默认情况下设为0，会自动根据IP填充，按平台规则，port=20100+CopterID*2-2。如果这里赋值大于0，则会强制使用port定义的端口。
          COM模式下，Port默认表示波特率self.baud=port。如果port=0，则会设置self.baud=57600
          Direct模式下，Port默认表示收发端口号（使用相同端口）
          redis模式下，Port对应服务器端口号self.redisPort = port。如果port=0，则self.redisPort=6379为平台默认值。

    接口示例：
    UDP模式
    PX4MavCtrler(1) # 默认IP
    PX4MavCtrler(1,'192.168.31.24') # 指定IP，用于远程控制

    串口模式
    PX4MavCtrler(1,'127.0.0.1','com1',57600) # 指定IP，用于远程控制

    """

    # @brief RflyRosStart的构造函数
    # @param 初始化参数为：飞机的ID号、飞机的ip地址、飞机与Pixhawk的连接模式、端口号、飞机模型
    # @return 无
    def __init__(self, ID=1, ip='127.0.0.1',Com='udp',port=0, simulinkDLL=False):
        ret = self.KillMavRos() #首先这个脚本不能用于多个飞控，因为这里面没有涉及到命名空间的使用。因此可以在启动mavros节点钱， 先kill 已有mavros节点
        if(not ret):
            print("mavros node not clean")
            sys.exit()
        self.isCom = False
        self.Com = Com
        self.baud = 115200
        self.isRealFly = 0
        self.ip = ip
        self.isRedis = False
        self.simulinkDLL = simulinkDLL
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
            if(len(strlist) >= 2): # 串口号:波特率 协议解析，为了兼容旧接口
                if strlist[1].isdigit():
                    self.baud = int(strlist[1])
            self.ComName = strlist[0] # 串口名字

        # 网络直连模式解析
        if Com[0:7]=='Direct:' or Com[0:7]=='direct:': # 如果UDP直连的真机模式
            strlist = Com.split(':')
            if(len(strlist) >= 2):
                if strlist[1].isdigit():
                    self.port = int(strlist[1])
                    self.isRealFly=1

        self.InitRosLoop()

    # @brief 建立ROS与mavros的连接
    # - @anchor InitRosLoop
    # @param 无
    # @return 无
    def InitRosLoop(self):
        
        if self.isCom:
            the_connection = mavutil.mavlink_connection(self.ComName,self.baud) # 先连接一下，获取SysID
            the_connection.recv_match(
                    type=['HEARTBEAT'],
                    blocking=True)
            self.tgtSys=the_connection.target_system
            the_connection.close()
            print(self.tgtSys)
            # 然后用mavros连接
            if is_use_ros1:
                cmdStr = 'roslaunch mavros px4.launch tgt_system:='+str(self.tgtSys)+' fcu_url:="serial://'+self.ComName+':'+str(self.baud)+'"'
            else:#
                cmdStr = 'ros2 launch mavros px4.launch.xml tgt_system:='+str(self.tgtSys)+' fcu_url:="serial://'+self.ComName+':'+str(self.baud)+'"'
        else:
            
            # 先连接一下，获取SysID
            the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:'+str(self.port+1))
            
            # the_connection.recv_match(
            #         type=['HEARTBEAT'],
            #         blocking=True)
            self.tgtSys=self.CopterID
            the_connection.close()
            print(self.tgtSys)
            # 然后用mavros连接
            if is_use_ros1:
                cmdStr = 'roslaunch mavros px4.launch tgt_system:='+str(self.tgtSys)+' fcu_url:="udp://:'+str(int(self.port)+1)+'@'+self.ip+':'+str(self.port)+'"'
            else:
                cmdStr = 'ros2 launch mavros px4.launch.xml tgt_system:='+str(self.tgtSys)+' fcu_url:="udp://:'+str(int(self.port)+1)+'@'+self.ip+':'+str(self.port)+'"'

        
        print(cmdStr)

        #使用管道方式打开ROS2 有bug
        #self.child = subprocess.Popen(cmdStr,
        #                 shell=True,
        #                 stdout=subprocess.PIPE)
        
        self.child = subprocess.Popen(cmdStr,
                         shell=True)
        time.sleep(10)
        # print(self.child.poll())


    def KillMavRos(self)->bool:
        # mavros_pid = -1
        # child = subprocess.Popen(['pgrep', '-f', "mavros"],stdout=subprocess.PIPE,shell=False)
        # response = child.communicate()[0]
        # ret = [int(pid) for pid in response.split()]
        # print("ret ", ret)
        # for i in range(len(ret)):
        #     pid = ret[i]
        #     print("mavros pid: ",pid)
        #     print("kill current mavros node or restart mavros node")
        #     subprocess.Popen("kill -9 {0}".format(str(pid)),shell=True)
        rec = time.time()
        while True:
            child = subprocess.Popen(['pgrep', '-f', "mavros_node"],stdout=subprocess.PIPE,shell=False)
            response = child.communicate()[0]
            ret = [int(pid) for pid in response.split()]
            print("ret: ",ret)
            if(len(ret) == 0):
                return True
            else:
                for i in range(len(ret)):
                    pid = ret[i]
                    print("mavros pid: ",pid)
                    print("kill current mavros node or restart mavros node")
                    sub = subprocess.Popen("kill -9 {0}".format(str(pid)),shell=True)
                    sub.communicate()
            time.sleep(3)
            if(time.time() - rec > 5):
                return False

    # @brief 结束ROS通信的打印函数
    # - @anchor EndRosLoop
    # @param 无
    # @return 无
    def EndRosLoop(self):
        #rosnode kill node_name
        #os.system('rosnode kill mavros')
        self.KillMavRos()

