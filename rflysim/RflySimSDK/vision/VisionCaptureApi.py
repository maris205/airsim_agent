#!/bin/python3
from email import header
import re
import socket
import threading
import time
import cv2
import numpy as np
import struct
import mmap
import json
import sys
import os
import math
import copy
import platform
import time
import psutil
import subprocess

## @file
#  这是一个与RflySim3D进行交互的模块。
#  @anchor VisionCaptureApi接口库文件
#  对应例程链接见

# 是否启用ROS图像转发功能
# IsEnable ROS image topic forwarding

##  @brief 用来控制是否启用ROS图像话题转发功能
isEnableRosTrans = False
##  @brief 判断ROS的版本
is_use_ros1 = True
##  @brief 判断是否在Linux环境下
isLinux = False
##  @brief 判断是否在WSL环境下
isWSL = False

if 'WSL_DISTRO_NAME' in os.environ:
    isWSL = True


##  @brief 根据ROS版本导入不同的Python库
if platform.system().lower() == "linux":
    isLinux = True
    try:
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

        if ros_version == "1":
            import rospy
        else:
            import rclpy
            from rclpy.node import Node
            from rclpy.clock import Clock
            from rclpy.duration import Duration
            from rclpy.qos import qos_profile_sensor_data
            is_use_ros1 = False

    except ImportError:
        print("Faild to load ROS labs")

## @class Queue
#  @brief Queue结构体类。类的文档字符串是pacth
class Queue:
    """pacth"""

    ## @brief Queue的构造函数 
    #  @param 初始化一个队列对象items
    def __init__(self):
        self.items = []

    ## @brief 将一个元素添加到队列的末尾
    # @param 初始化一个队列对象items
    def enqueue(self, item):
        self.items.insert(0, item)

    ## @brief 将列表的首位元素删除，并返回该列表。
    #  @return  返回列表。
    def dequeue(self):
        return self.items.pop()

    ## @brief 检查队列是否为空
    #  @return 如果items列表为空，返回True；否则返回False
    def is_empty(self):
        return self.items == []

    ## @brief 用于返回队列中的元素个数
    #  @return 返回队列中的元素个数
    def size(self):
        return len(self.items)


# 注意:本条消息会发送给指定远端电脑的端口20005
# struct RflyTimeStmp{
#     int checksum; //校验位，取123456789
#     int copterID; //当前飞机的ID号
#     long long SysStartTime; //开始仿真时的时间戳（单位毫秒，格林尼治标准起点）
#     long long SysCurrentTime;//当前时间戳（单位毫秒，格林尼治标准起点）
#     long long HeartCount; //心跳包的计数器
# } 2i3q

## @class RflyTimeStmp
#  @brief RflyTimeStmp结构体类，用于处理与CopterSim仿真相关的时间戳信息
class RflyTimeStmp:
    
    ## @brief RflyTimeStmp的构造函数
    # @param 初始化一个包含校验和、CopterID、SysStartTime、SysCurrentTime等等参数的值，默认为0
    def __init__(self):
        ## @var RflyTimeStmp.checksum
        #  @brief 这是校验和的值，初始化为1234567897
        self.checksum = 1234567897
        ## @var RflyTimeStmp.copterID
        #  @brief 用于识别直升机的标识符
        self.copterID = 0
        ## @var RflyTimeStmp.SysStartTime
        #  @brief 用于记录CopterSim开始仿真时，电脑的时间戳
        self.SysStartTime = 0  # CopterSim开始仿真时，电脑的时间戳（单位秒）
        ## @var RflyTimeStmp.SysCurrentTime
        #  @brief 用于记录CopterSim运行电脑当前的时间戳
        self.SysCurrentTime = 0  # CopterSim运行电脑当前的时间戳（单位秒）
        ## @var RflyTimeStmp.HeartCount
        #  @brief 用于记录心跳计数，初始化为0
        self.HeartCount = 0

        # Python端处理的时间戳。
        ## @var RflyTimeStmp.isCopterSimOnPC
        #  @brief 用于判断CopterSim和本Python脚本是否在一台电脑上运行，初始化为False
        self.isCopterSimOnPC = False
        # 注意：如果CopterSim和本Python脚本在一台电脑，SysCurrentTime和time.time()的数值应该相差很小（最多延迟10ms）
        # 以此差值来判断，CopterSim和本Python脚本是否在一台电脑上
        ## @var RflyTimeStmp.rosStartTimeStmp
        #  @brief CopterSim开始仿真时，Python脚本电脑ROS的时间戳，初始化为0
        self.rosStartTimeStmp = 0  # CopterSim开始仿真时，本Python脚本电脑ROS的时间戳
        # 注意：如果CopterSim和本Python脚本在一台电脑
        # 那么pyStartTimeStmp<--SysStartTime直接用CopterSim记录的起始仿真时间
        # 那么rosStartTimeStmp<--pyStartTimeStmp + ROSTime - time.time()，也就是说加上一个ROS时间相对本机的偏差量

        ## @var RflyTimeStmp.pyStartTimeStmp
        #  @brief Python脚本处理的时间戳，初始化为0
        self.pyStartTimeStmp = 0

        # 如果CopterSim和本Python脚本不在一台电脑
        # 那么pyStartTimeStmp<--time.time() - SysCurrentTime + SysStartTime - 10
        # 也就是根据当前的仿真时间（SysCurrentTime-SysStartTime）去逆推CopterSim开始仿真时时间，这里加了一个10毫秒的延迟量
        # 那么rosStartTimeStmp<--pyStartTimeStmp + ROSTime - time.time()，也就是说加上一个ROS时间相对本机的偏差量

    ## @brief RflyTimeStmp的构造函数
    # @param 初始化一个包含校验和、CopterID、SysStartTime、SysCurrentTime等等参数的值，默认为0
    def __init__(self, iv):
        ## @var RflyTimeStmp.checksum
        #  @brief 这是校验和的值，初始化为1234567897
        self.checksum = iv[0]
        ## @var PX4SILIntFloat.CopterID
        #  @brief 用于识别直升机的标识符。
        self.copterID = iv[1]
        ## @var RflyTimeStmp.SysStartTime
        #  @brief 用于记录CopterSim开始仿真时，电脑的时间戳
        self.SysStartTime = iv[2] / 1000.0
        ## @var RflyTimeStmp.SysCurrentTime
        #  @brief 用于记录CopterSim运行电脑当前的时间戳
        self.SysCurrentTime = iv[3] / 1000.0
        ## @var RflyTimeStmp.HeartCount
        #  @brief 用于记录心跳计数，初始化为0
        self.HeartCount = iv[4]

    ## @brief RflyTimeStmp的更新函数
    # @param 更新一个包含校验和、CopterID、SysStartTime、SysCurrentTime等等参数
    def Update(self, iv):
        ## @var RflyTimeStmp.checksum
        #  @brief 更新checksum为iv[0]
        self.checksum = iv[0]
        ## @var RflyTimeStmp.copterID
        #  @brief 更新checksum为iv[1]
        self.copterID = iv[1]
        ## @var RflyTimeStmp.SysStartTime
        #  @brief 更新checksum为iv[2] / 1000.0
        self.SysStartTime = iv[2] / 1000.0
        ## @var RflyTimeStmp.SysCurrentTime
        #  @brief 更新checksum为iv[3] / 1000.0
        self.SysCurrentTime = iv[3] / 1000.0
        ## @var RflyTimeStmp.HeartCount
        #  @brief 更新checksum为iv[4]
        self.HeartCount = iv[4]

## @class VisionSensorReq
#  @brief VisionSensorReq结构体类，用于初始化各种与摄像机和传感器相关的参数
class VisionSensorReq:
    """This is a class (C++ struct) that sent to UE4 to request and set camera parameters.
    # struct VisionSensorReq {
        uint16 checksum; //数据校验位，12345
        uint16 SeqID; //内存序号ID
        uint16 TypeID; //传感器类型ID
        uint16 TargetCopter; //绑定的目标飞机     //可改变
        uint16 TargetMountType; //绑定的类型    //可改变
        uint16 DataWidth;   //数据或图像宽度
        uint16 DataHeight; //数据或图像高度
        uint16 DataCheckFreq; //检查数据更新频率
        uint16 SendProtocol[8]; //传输类型（共享内存、UDP传输无压缩、UDP视频串流），IP地址，端口号，...
        float CameraFOV;  //相机视场角（仅限视觉类传感器）  //可改变
        float SensorPosXYZ[3]; // 传感器安装位置    //可改变
        float EularOrQuat; //选择欧拉角或四元数方式，大于0.5就是四元数
        float SensorAngEular[3]; //传感器安装角度   //可改变
        float SensorAngQuat[4]; //传感器安装四元数   //可改变
        float otherParams[16]; //预留的16位数据位
    # }16H28f
    """

    def __init__(self):
        self.checksum = 12345
        self.SeqID = 0
        self.TypeID = 1
        self.TargetCopter = 1
        self.TargetMountType = 0
        self.DataWidth = 0
        self.DataHeight = 0
        self.DataCheckFreq = 0
        self.SendProtocol = [0] * 8
        self.CameraFOV = 90
        self.EularOrQuat = 0
        self.SensorAngQuat = [0, 0, 0, 0]
        self.SensorPosXYZ = [0, 0, 0]
        self.SensorAngEular = [0, 0, 0]
        self.otherParams = [0] * 16

## @class VisionSensorReqNew
#  @brief VisionSensorReqNew结构体类，用于初始化各种与摄像机和传感器相关的参数
#  在SensorType = 9的情况下
class VisionSensorReqNew:       # SensorType == 9
    """This is a class (C++ struct) that sent to UE4 to request and set camera parameters.
    # struct VisionSensorReq {
        uint16 checksum; //数据校验位，12345
        uint16 SeqID; //内存序号ID
        uint32 bitmask; // 控制
        uint16 TypeID; //传感器类型ID
        uint16 TargetCopter; //绑定的目标飞机     //可改变
        uint16 TargetMountType; //绑定的类型    //可改变
        uint16 DataWidth;   //数据或图像宽度
        uint16 DataHeight; //数据或图像高度
        uint16 DataCheckFreq; //检查数据更新频率
        uint16 SendProtocol[8]; //传输类型（共享内存、UDP传输无压缩、UDP视频串流），IP地址，端口号，...
        float CameraFOV;  //相机视场角（仅限视觉类传感器）  //可改变
        float SensorPosXYZ[3]; // 传感器安装位置    //可改变
        float EularOrQuat; //选择欧拉角或四元数方式，大于0.5就是四元数
        float SensorAngEular[3]; //传感器安装角度   //可改变
        float SensorAngQuat[4]; //传感器安装四元数   //可改变
        float otherParams[16]; //预留的16位数据位
    # }2H1I14H28f
    """

    def __init__(self):
        self.checksum = 12345
        self.SeqID = 0
        self.bitmask = 0
        self.TypeID = 1
        self.TargetCopter = 1
        self.TargetMountType = 0
        self.DataWidth = 0
        self.DataHeight = 0
        self.DataCheckFreq = 0
        self.SendProtocol = [0] * 8
        self.CameraFOV = 90
        self.EularOrQuat = 0
        self.SensorAngQuat = [0, 0, 0, 0]
        self.SensorPosXYZ = [0, 0, 0]
        self.SensorAngEular = [0, 0, 0]
        self.otherParams = [0] * 16

## @class imuDataCopter
#  @brief imuDataCopter结构体类，用于从 CopterSim 接收 IMU 数据，并将这些数据发布到 ROS 话题上
class imuDataCopter:
    """This is a class (C++ struct) for IMU data receive from CopterSim
    # struct imuDataCopter{
    #     int checksum; //数据校验位1234567898
    #     int seq; //消息序号
    #     double timestmp;//时间戳
    #     float acc[3];
    #     float rate[3];
    # }   //2i1d6f
    """

    ## @brief imuDataCopter的构造函数
    # @param 初始化类属性，包括校验和、序号、时间戳、加速度和速率等
    def __init__(self, imu_name="/rflysim/imu", node=None):
        global isEnableRosTrans
        global is_use_ros1
        self.checksum = 1234567898
        self.seq = 0
        self.timestmp = 0
        self.acc = [0, 0, 0]
        self.rate = [0, 0, 0]
        self.imuStmp = 0  # 经过校正的IMU时间戳
        self.rflyStartStmp = 0  # CopterSim开始仿真的时间戳
        if isEnableRosTrans:
            self.time_record = -1
            self.isUseTimeAlign = True  # 是否使用与图像时间对其的方式发布数据
            if len(imu_name) == 0:
                imu_name = "/rflysim/imu"
            if is_use_ros1:
                self.ns = rospy.get_namespace()
                if len(self.ns) > 1:
                    imu_name = self.ns + "rflysim/imu"
                self.imu_pub = rospy.Publisher(imu_name, sensor.Imu, queue_size=1)
                self.rostime = rospy.Time.now()

            else:
                self.rostime = rclpy.time.Time()
                self.imu_pub = node.create_publisher(sensor.Imu, imu_name, 1)
            self.time_queue = Queue()
            self.newest_time_img = -1
            self.test_imu_time = 0
            self.test_sum = 0
            self.count = 0
            self.ros_imu = sensor.Imu()
            self.imu_frame_id = "imu"
            self.ros_imu.header.frame_id = self.imu_frame_id

    ## @brief 用于与图像时间对齐
    def AlignTime(self, img_time):  # 原子操作不需要用锁，用了反而较低下率
        self.newest_time_img = img_time
        # print("queue size: ",self.time_queue.size())
        # print("current <img:%f,imu:%f>"% (img_time,self.test_imu_time))
        # self.test_sum += abs(self.newest_time_img - self.test_imu_time)
        # self.count +=1
        # if(self.count == 10):
        #     print("====",self.test_sum/self.count)
        #     self.count = 0
        #     self.test_sum = 0

        pass

    ## @brief 用于将IMU数据发布到ROS话题。它根据当前使用的是ROS1还是ROS2，
    # 设置时间戳和IMU数据，然后发布这些数据
    # @param xxx_covariance是指xxx的协方差矩阵
    def Imu2ros(self, node=None):
        global is_use_ros1
        if isEnableRosTrans:
            # ros_imu = sensor.Imu()
            if is_use_ros1:
                self.ros_imu.header.stamp = rospy.Duration(self.imuStmp)
            else:
                rclpy_time = self.rostime + Duration(
                    seconds=self.imuStmp, nanoseconds=0
                )
                # self.ros_imu.header.stamp = rclpy_time.to_msg()
                seconds, nanoseconds = rclpy_time.seconds_nanoseconds()
                self.ros_imu.header.stamp.sec = int(seconds & 0x7FFFFFFF)
                self.ros_imu.header.stamp.nanosec = nanoseconds
            self.ros_imu.orientation.w = 0.0
            self.ros_imu.orientation.x = 0.0
            self.ros_imu.orientation.y = 0.0
            self.ros_imu.orientation.z = 0.0
            self.ros_imu.orientation_covariance[0] = -1.0
            self.ros_imu.orientation_covariance[1] = -1.0
            self.ros_imu.orientation_covariance[2] = -1.0
            self.ros_imu.orientation_covariance[3] = -1.0
            self.ros_imu.orientation_covariance[4] = -1.0
            self.ros_imu.orientation_covariance[5] = -1.0
            self.ros_imu.orientation_covariance[6] = -1.0
            self.ros_imu.orientation_covariance[7] = -1.0
            self.ros_imu.orientation_covariance[8] = -1.0
            self.ros_imu.linear_acceleration.x = self.acc[0]
            self.ros_imu.linear_acceleration.y = -self.acc[1]
            self.ros_imu.linear_acceleration.z = -self.acc[2]
            self.ros_imu.angular_velocity.x = self.rate[0]
            self.ros_imu.angular_velocity.y = -self.rate[1]
            self.ros_imu.angular_velocity.z = -self.rate[2]
            self.ros_imu.angular_velocity_covariance[0] = 0.001
            self.ros_imu.angular_velocity_covariance[4] = 0.001
            self.ros_imu.angular_velocity_covariance[8] = 0.001
            # self.ros_imu.header.stamp.secs = self.timestmp
            self.imu_pub.publish(self.ros_imu)

## @class DistanceSensor
#  @brief DistanceSensor结构体类，用于初始化测距传感器的参数
class DistanceSensor:
    ## @brief DistanceSensor的构造函数
    # @param 初始化类属性
    def __init__(self):
        ##  @var DistanceSensor.TimeStamp
        #   @brief 这是当前消息的时间戳，初始化为 0
        self.TimeStamp = 0
        ##  @var DistanceSensor.Distance
        #   @brief 这是距离传感器测量到的距离，初始化为 0
        self.Distance = 0
        ##  @var DistanceSensor.CopterID
        #   @brief 用于标识直升机的ID，初始化为 0
        self.CopterID = 0
        ##  @var DistanceSensor.RayStart
        #   @brief 这是射线起点的坐标，初始化为[0,0,0]
        self.RayStart = [0,0,0]
        ##  @var DistanceSensor.AngEular
        #   @brief 这是传感器的欧拉角（Euler Angles），初始化为[0,0,0]
        self.AngEular = [0,0,0]
        ##  @var DistanceSensor.ImpactPoint
        #   @brief 这是碰撞点的坐标，初始化为[0,0,0]
        self.ImpactPoint = [0,0,0]
        ##  @var DistanceSensor.BoxOri
        #   @brief 这是盒子的原点或参考点，初始化为[0,0,0]
        self.BoxOri = [0,0,0]

## @class SensorReqCopterSim
#  @brief SensorReqCopterSim结构体类，用于向UE4请求传感器数据的
class SensorReqCopterSim:
    """This is a class (C++ struct) that sent to UE4 to request sensor data.
    # struct SensorReqCopterSim{
    #     uint16_t checksum;
    #     uint16_t sensorType;
    #     uint16_t updateFreq;
    #     uint16_t port;
    #     uint8_t IP[4];
    #     float Params[6];
    # } //4H4B6f
    """

    ## @brief SensorReqCopterSim的构造函数
    # @param 初始化传感器的类型、数据更新频率、网络端口和目标IP地址，以及其他需要传递的参数
    def __init__(self):
        ##  @var SensorReqCopterSim.checksum
        #   @brief 这是校验是数据的值，初始化为12345
        self.checksum = 12345
        ##  @var SensorReqCopterSim.sensorType
        #   @brief 这是传感器的类型
        self.sensorType = 0
        ##  @var SensorReqCopterSim.updateFreq
        #   @brief 这是传感器数据更新的频率
        self.updateFreq = 100
        ##  @var SensorReqCopterSim.port
        #   @brief 这是指定数据传输的端口
        self.port = 9998
        ##  @var SensorReqCopterSim.IP
        #   @brief 这是指定数据传输的目标IP地址
        self.IP = [127, 0, 0, 1]
        ##  @var SensorReqCopterSim.Params
        #   @brief 这是额外的参数数组，可以用于传递传感器请求的其他配置参数
        self.Params = [0, 0, 0, 0, 0, 0]

## @class VisionCaptureApi
#  @brief VisionCaptureApi结构体类，用于从 UE4 请求图像数据，并可以选择使用 ROS 进行传感器数据的发布和接收
class VisionCaptureApi:
    """This is the API class for python to request image from UE4"""

    ## @brief VisionCaptureApi的析构函数
    # @brief 用于清理ROS资源。在对象销毁时调用。如果启用了ROS且使用的是ROS2，则销毁ROS节点并关闭rclpy
    def __del__(self):
        global isEnableRosTrans
        global is_use_ros1
        self.stopFlag=True
        if isEnableRosTrans and not is_use_ros1:
            self.ros_node.destroy_node()
            rclpy.shutdown()

    ## @brief VisionCaptureApi的构造函数
    # @param 初始化各种类属性和套接字，用于与UE4和ROS进行通信
    def __init__(self, ip="127.0.0.1"):
        global isEnableRosTrans
        global is_use_ros1
        self.stopFlag=False
        if isEnableRosTrans:
            if is_use_ros1:
                #启动之前先判断有没有启动roscore
                # roscore_process = subprocess.Popen(['pgrep', '-f', "roscore"],stdout=subprocess.PIPE,shell=False)
                # res_roscore = roscore_process.communicate()[0]
                # roscore_pids = [int(pid) for pid in res_roscore.split()]
                
                rosmaster_process = subprocess.Popen(['pgrep', '-f', "rosmaster"],stdout=subprocess.PIPE,shell=False)
                res_rosmaster = rosmaster_process.communicate()[0]
                rosmaster_pids = [int(pid) for pid in res_rosmaster.split()]
                rec = time.time()
                while len(rosmaster_pids) == 0:
                    if(len(rosmaster_pids) == 0):
                        roscore_sub = subprocess.Popen("roscore",shell=True)
                        # roscore_sub.communicate()
                        time.sleep(3)
                    rosmaster_process = subprocess.Popen(['pgrep', '-f', "rosmaster"],stdout=subprocess.PIPE,shell=False)
                    res_rosmaster = rosmaster_process.communicate()[0]
                    rosmaster_pids = [int(pid) for pid in res_rosmaster.split()]
                    time.sleep(1)

                    if time.time() - rec > 10: #5秒内rosmaster 还没起来直接退出程序
                        print("启动roscore 失败，退出程序")
                        sys.exit()
                rospy.init_node("RecvRFlySim3DData", anonymous=True)
            else:
                rclpy.init()
                self.ros_node = Node("RecvRFlySim3DData")
                # print("If you want use to ROS2, pelease set rosnode_id at initialize  class VisionCaptureApi")
                # sys.exit(-1)
            self.time_record = Any
            self.rostime = Any
            # 加入ROS节点的初始化工作
        ##  @var VisionCaptureApi.udp_socket
        # @brief 用于UDP通信的套接字
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ##  @var VisionCaptureApi.udp_imu
        # @brief 用于IMU数据的UDP套接字
        self.udp_imu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.udp_imu.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ##  @var VisionCaptureApi.hostIp
        # @brief 用于获取并存储本机电脑的IP地址
        self.hostIp = socket.gethostbyname(socket.gethostname())  # 获取本机电脑的IP
        ##  @var VisionCaptureApi.VisSensor
        # @brief 用于存储视觉传感器的列表
        self.VisSensor = []
        ##  @var VisionCaptureApi.VisSensorNew
        # @brief 用于存储新添加的视觉传感器的列表
        self.VisSensorNew = []          # SensorType == 9
        ##  @var VisionCaptureApi.Img
        # @brief 用于存储从视觉传感器获取的图像数据的列表
        self.Img = []
        ##  @var VisionCaptureApi.Img_lock
        # @brief 用于实现类似多线程的数据同步，防止多个线程同时访问图像数据
        self.Img_lock = []  # 为类似多线程数据同步,加上线程锁
        ##  @var VisionCaptureApi.ImgData
        # @brief 用于存储图像数据的实际内容
        self.ImgData = []
        ##  @var VisionCaptureApi.hasData
        # @brief 用于存储布尔值列表，表示是否有新数据
        self.hasData = []
        ##  @var VisionCaptureApi.timeStmp
        # @brief 用于存储时间戳的列表，用于记录每个传感器数据的时间
        self.timeStmp = []
        ##  @var VisionCaptureApi.imgStmp
        # @brief 用于存储图像数据的时间戳列表
        self.imgStmp = []
        ##  @var VisionCaptureApi.rflyStartStmp
        # @brief 用于存储 Rfly 仿真开始的时间戳列表
        self.rflyStartStmp = []
        ##  @var VisionCaptureApi.IpList
        # @brief 用于存储需要通信的目标 IP 地址列表
        self.IpList = []
        ##  @var VisionCaptureApi.portList
        # @brief 用于存储需要通信的目标端口号列表
        self.portList = []
        ##  @var VisionCaptureApi.hasReqUE4
        # @brief 用于布尔值，表示是否已经向 UE4 请求了数据
        self.hasReqUE4 = False
        ##  @var VisionCaptureApi.sleepCheck
        # @brief 用于设定数据请求的时间间隔，单位为秒
        self.sleepCheck = 0.005
        ##  @var VisionCaptureApi.ip
        # @brief 用于存储传入的 IP 地址
        self.ip = ip
        ##  @var VisionCaptureApi.isRemoteSend
        # @brief 布尔值，表示是否启用远程发送
        self.isRemoteSend = False
        ##  @var VisionCaptureApi.RemotSendIP
        # @brief 用于存储远程发送的目标 IP 地址
        self.RemotSendIP = ""
        ##  @var VisionCaptureApi.isUE4DirectUDP
        # @brief 布尔值，表示是否直接通过 UDP 与 UE4 进行通信
        self.isUE4DirectUDP = False
        ##  @var VisionCaptureApi.hasIMUData
        # @brief 用于布尔值，表示是否有新的 IMU 数据
        self.hasIMUData = False
        ##  @var VisionCaptureApi.RflyTimeVect
        # @brief 存储 Rfly 时间向量的列表，用于记录时间同步信息
        self.RflyTimeVect = []
        ##  @var VisionCaptureApi.RflyLocalIPVect
        # @brief 存储本电脑的所有本地IP地址，用于判断数据是否来着本机
        self.RflyLocalIPVect = self.get_all_ip()
        #print(self.RflyLocalIPVect)
        ##  @var VisionCaptureApi.isNewJson
        # @brief 布尔值，表示是否使用新的 JSON 数据格式
        self.isNewJson = False
        ##  @var VisionCaptureApi.tTimeStmpFlag
        # @brief 布尔值，表示时间戳标志位，用于时间同步的控制
        self.tTimeStmpFlag = False

        ##  @var VisionCaptureApi.startTime
        # @brief 用于记录类实例化时的起始时间，用于计算运行时间
        self.startTime = time.time()
        ##  @var VisionCaptureApi.isPrintTime
        # @brief 布尔值，表示是否打印时间信息
        self.isPrintTime = False
        ##  @var VisionCaptureApi.lastIMUTime
        # @brief 记录最后一次接收到的 IMU 数据的时间
        self.lastIMUTime = time.time()
        ##  @var VisionCaptureApi.sensors_num
        # @brief 存储传感器的数量
        self.sensors_num = 0
        
        if isEnableRosTrans:
            self.sensors_frame_id = ["map"]
            self.imu_frame_id = "imu"
            self.imu_topic_name = ""
            self.sensor_pub = {}
            self.sensor_data = {}
            self.cv_bridge = CvBridge()
            self.topic_name = {}
            try:
                file = open(r"tf_cfg.yaml")
                y = yaml.safe_load(file)
                self.imu_frame_id = y["imu_frame_id"]
                self.sensors_frame_id = y["sensors_frame_id"]
                self.sensors_num = y["sensors_num"]
                self.imu_topic_name = y["imu_topic_name"]
            except IOError:
                print("使用默认的全局坐标系下的frame_id:map")

        if isEnableRosTrans:
            if not is_use_ros1:
                self.imu = imuDataCopter(
                    imu_name=self.imu_topic_name, node=self.ros_node
                )
                self.imu.imu_frame_id = self.imu_frame_id
            else:
                self.imu = imuDataCopter(imu_name=self.imu_topic_name)
                self.imu.imu_frame_id = self.imu_frame_id
        else:
            self.imu = imuDataCopter()
            if isEnableRosTrans:
                self.imu.imu_frame_id = self.imu_frame_id
        # self.lock = threading.Lock() #应用到IMU和图像数据时间对齐处理

    ## @brief 添加一个新的 VisionSensorReq 结构体到 VisSensor 列表中
    def addVisSensor(self, vsr=VisionSensorReq()):
        """Add a new VisionSensorReq struct to the list"""
        if isinstance(vsr, VisionSensorReq):
            self.VisSensor = self.VisSensor + [copy.deepcopy(vsr)]
        else:
            raise Exception("Wrong data input to addVisSensor()")
        
    ## @brief 添加一个新的 VisionSensorReqNew 结构体到 VisSensorNew 列表中
    #  @brief 和前一个析构函数名称一样，这个会替代掉上一个
    def addVisSensor(self, vsr=VisionSensorReqNew()):       # SensorType == 9
        """Add a new VisionSensorReq struct to the list"""
        if isinstance(vsr, VisionSensorReqNew):
            self.VisSensorNew = self.VisSensorNew + [copy.deepcopy(vsr)]
        else:
            raise Exception("Wrong data input to addVisSensor()")

    ## @brief 发送一个 SensorReqCopterSim 类的 UDP 消息给 CopterSim 以请求传感器数据
    def sendReqToCopterSim(self, srcs=SensorReqCopterSim(), copterID=1,IP='127.0.0.1'):
        """send UDP message SensorReqCopterSim to CopterSim to request a sensor data
        the copterID specify the index of CopterSim to request
        """
        if type(srcs).__name__ != "SensorReqCopterSim":
            print("Error: input is not SensorReqCopterSim class")
            return
        u16Value = [srcs.checksum, srcs.sensorType, srcs.updateFreq, srcs.port]
        u8Value = srcs.IP
        fValue = srcs.Params
        buf = struct.pack("4H4B6f", *u16Value, *u8Value, *fValue)
        self.udp_socket.sendto(buf, (IP, 30100 + (copterID - 1) * 2))

    ## @brief 发送请求给 CopterSim 以获取 IMU 数据，并初始化一个线程监听 IMU 数据
    def sendImuReqCopterSim(self, copterID=1, IP="127.0.0.1", freq=200):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        IP is the IP of the PC to send request to
        port is the base port that CopterSim send to
        (Actual port for a vehicle = baseport + copterID -1)
        freq is the frequency of the send data
        This function will init a thread to listen IMU data
        """
        self.sendImuReqClient(copterID, IP, freq)
        self.sendImuReqServe(copterID)


    ## @brief 发送命令到 CopterSim 请求IMU数据
    def sendImuReqClient(self, copterID=1, IP="127.0.0.1", freq=200):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        IP is the IP of the PC to send request to
        port is the base port that CopterSim send to
        (Actual port for a vehicle = baseport + copterID -1)
        freq is the frequency of the send data
        """

        # if RemotSendIP has been set, the IMU rev IP will be RemotSendIP
        # else use local IP address 127.0.0.1

        BackIP='127.0.0.1'
        if self.RemotSendIP != "":
            BackIP=self.RemotSendIP

        srcs = SensorReqCopterSim()
        srcs.sensorType = 0  # IMU传感器数据
        srcs.updateFreq = freq

        # 返回地址默认为127的地址，CopterSim收到后会原IP返回
        # 如果self.RemotSendIP有设置过的话，CopterSim会将数据转发到指定端口
        cList = BackIP.split(".")
        if len(cList) == 4:
            srcs.IP[0] = int(cList[0])
            srcs.IP[1] = int(cList[1])
            srcs.IP[2] = int(cList[2])
            srcs.IP[3] = int(cList[3])
        srcs.port = 31000 + copterID - 1
        self.sendReqToCopterSim(srcs, copterID,IP)  # 发送消息请求IMU数据

    ## @brief 发送命令到 CopterSim 请求IMU数据，并初始化一个线程监听IMU数据
    def sendImuReqServe(self, copterID=1):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        port is the base port that CopterSim send to
        (Actual port for a vehicle = baseport + copterID -1)
        This function will init a thread to listen IMU data
        """
        port = 31000 + copterID - 1
        self.udp_imu.bind(("0.0.0.0", port))
        self.tIMU = threading.Thread(target=self.getIMUDataLoop, args=(copterID,))
        self.tIMU.start()

    ## @brief 接收来自CopterSim（模拟飞行器）的IMU（惯性测量单元）数据，
    # 
    #  并根据需要将其转换并发布到ROS（机器人操作系统）
    def getIMUDataLoop(self, copterID):
        global isEnableRosTrans
        global is_use_ros1

        for i in range(len(self.RflyTimeVect)):
            if self.RflyTimeVect[i].copterID == copterID:
                if isEnableRosTrans:
                    self.imu.rflyStartStmp = self.RflyTimeVect[i].rosStartTimeStmp
                else:
                    self.imu.rflyStartStmp = self.RflyTimeVect[i].pyStartTimeStmp

        # if self.tTimeStmpFlag

        print("Start lisening to IMU Msg")
        while True:
            if self.stopFlag:
                break
            try:
                buf, addr = self.udp_imu.recvfrom(65500)
                if len(buf) == 40:
                    # print(len(buf[0:12]))
                    IMUData = struct.unpack("2i1d6f", buf)
                    if IMUData[0] == 1234567898:
                        self.imu.checksum = IMUData[0]
                        self.imu.seq = IMUData[1]
                        self.imu.timestmp = IMUData[2]

                        if self.imu.rflyStartStmp < 0.01:  # 说明还没获取到过CopterSim时间戳
                            print("No CopterSim time, use image time to calculate.")
                            if isEnableRosTrans:
                                if is_use_ros1:
                                    ros_now_time = rospy.Time.now().to_sec()
                                else:
                                    now = self.ros_node.get_clock().now()
                                    ros_now_time = (
                                    now.to_msg().sec + now.to_msg().nanosec * 1e-9
                                    )
                                self.imu.rflyStartStmp = (
                                    ros_now_time - self.imu.timestmp - 0.005
                                )
                                # tStmp.rosStartTimeStmp = tStmp.pyStartTimeStmp + ros_now_time - time.time()
                            else:
                                self.imu.rflyStartStmp = (
                                    time.time() - self.imu.timestmp - 0.005
                                )

                        self.imu.imuStmp = self.imu.rflyStartStmp + self.imu.timestmp

                        if self.isPrintTime:
                            self.lastIMUTime = time.time()
                            print("IMU:", self.imu.timestmp)
                        self.imu.acc[0] = IMUData[3]
                        self.imu.acc[1] = IMUData[4]
                        self.imu.acc[2] = IMUData[5]
                        self.imu.rate[0] = IMUData[6]
                        self.imu.rate[1] = IMUData[7]
                        self.imu.rate[2] = IMUData[8]
                        if not self.hasIMUData:
                            self.hasIMUData = True
                            print("Got CopterSim IMU Msg!")
                        if isEnableRosTrans and self.hasIMUData:
                            if is_use_ros1:
                                self.imu.Imu2ros()
                            else:
                                self.imu.Imu2ros(self.ros_node)
                            # 将IMU消息，推送到ROS消息中

            except Exception as ex:
                print("Error to listen to IMU Msg!")
                print(ex)
                sys.exit(0)

    ## @brief 获取本机所有 IP 地址 
    #  - @anchor get_all_ip 
    #  @return 返回本地IP地址列表。
    # 
    def get_all_ip(self):
        ip_list = []
        try:
            dic = psutil.net_if_addrs()
            for adapter in dic:
                snicList = dic[adapter]
                for snic in snicList:
                    if snic.family.name != 'AF_INET':
                        continue         
                    ip = snic.address
                    if ip.startswith('169'):
                        continue
                    ip_list.append(ip)      
        except:
            ip_list.append('127.0.0.1')
        if '127.0.0.1' not in ip_list:
            ip_list.append('127.0.0.1')
        return ip_list

    ## @brief 判断IP地址是否为本地IP 
    #  - @anchor isIpLocal 
    #  @return 返回本地IP地址列表。
    # 
    def isIpLocal(self,IP):
        if IP in self.RflyLocalIPVect: # 为本机
            return True
        else:
            return False

    ## @brief 通过UDP套接字监听特定端口（20005端口），以获取CopterID的时间戳信息
    def StartTimeStmplisten(self):
        """Start to listen to 20005 port to get RflyTimeStmp of CopterID"""
        self.udp_time = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.udp_time.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_time.bind(("0.0.0.0", 20005))
        # 增加对组播端口的支持

        try:
            status = self.udp_time.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                socket.inet_aton("224.0.0.10") + socket.inet_aton("0.0.0.0"),
            )
        except:
            print('Failed to Init multicast!')
        self.tTimeStmp = threading.Thread(target=self.TimeStmploop, args=())
        self.tTimeStmpFlag = True
        self.tTimeStmp.start()

    ## @brief 获取指定飞机的时间戳指针，
    def getTimeStmp(self,CopterID=1):
        time.sleep(3)
        for i in range(len(self.RflyTimeVect)):
            if self.RflyTimeVect[i].copterID == CopterID:
                isTimeExist = True
                # 如果已经在列表中了，就不再接收
                # self.RflyTimeVect[i].Update(TimeData)
                return self.RflyTimeVect[i]
        return RflyTimeStmp()


    ## @brief 停止时间戳监听功能
    def endTimeStmplisten(self):
        self.tTimeStmpFlag = False
        time.sleep(0.5)
        self.tTimeStmp.join()
        time.sleep(0.5)

    ## @brief 用于监听来自特定UDP端口的时间戳消息，并根据接收到的消息更新时间戳信息
    def TimeStmploop(self):
        global isEnableRosTrans
        global is_use_ros1
        print("Start lisening to timeStmp Msg")
        self.udp_time.settimeout(3)
        while self.tTimeStmpFlag:
            try:
                buf, addr = self.udp_time.recvfrom(65500)
                # print(addr)
                if len(buf) == 32:
                    # print(len(buf[0:12]))
                    TimeData = struct.unpack("2i3q", buf)
                    if TimeData[0] == 123456789:
                        cpIDTmp = TimeData[1]
                        isTimeExist = False
                        
                        # 确认是否已经接收过消息
                        for tStmp in self.RflyTimeVect:
                            if tStmp.copterID == cpIDTmp:
                                isTimeExist = True
                                
                                if self.isIpLocal(addr[0]): # 如果是本机消息，则只接收本机消息
                                    tStmp.Update(TimeData)
                                    if not tStmp.isCopterSimOnPC: # 如果之前没收到过本地包，则重新接收
                                        self.RflyTimeVect.remove(tStmp) 
                                        isTimeExist=False
                                else:
                                    if not tStmp.isCopterSimOnPC: # 如果未收到本机消息，就使用局域网消息
                                        tStmp.Update(TimeData)
                                break
                        
                        if not isTimeExist:
                            tStmp = RflyTimeStmp(TimeData)
                            CurPyTime = time.time()
                            #print("Got time msg from CopterSim #", tStmp.copterID)
                            if (self.isIpLocal(addr[0])):
                                tStmp.isCopterSimOnPC = True  # 说明Python和CopterSim在一台电脑上
                                tStmp.pyStartTimeStmp = tStmp.SysStartTime
                                print("Got time msg from CopterSim #",tStmp.copterID,", running on this PC")
                            else:  # 说明本Python脚本和CopterSim不在一台电脑上
                                tStmp.isCopterSimOnPC = False
                                tStmp.pyStartTimeStmp = (
                                    CurPyTime
                                    - tStmp.SysCurrentTime
                                    + tStmp.SysStartTime
                                    - 0.01
                                )
                                print("Got time msg from CopterSim #",tStmp.copterID,", not on this PC")
                                #print(tStmp.SysCurrentTime)
                                #print(CurPyTime)

                            if isEnableRosTrans:
                                if is_use_ros1:
                                    ros_now_time = rospy.Time.now().to_sec()
                                else:
                                    now = self.ros_node.get_clock().now()
                                    ros_now_time = (
                                        now.to_msg().sec + now.to_msg().nanosec * 1e-9
                                    )
                                tStmp.rosStartTimeStmp = (
                                    tStmp.pyStartTimeStmp + ros_now_time - CurPyTime
                                )

                            self.RflyTimeVect = self.RflyTimeVect + [
                                copy.deepcopy(tStmp)
                            ]  # 扩充列表，增加一个元素

            except:
                print("No Time Msg!")
                # 跳出循环，不再继续监听
                break

    ## @brief 向指定IP地址和端口发送图像更新请求
    # - @anchor sendUpdateUEImage
    ## @param vs 视觉传感器请求对象,用于初始化各种与摄像机和传感器相关的参数
    ## @param windID（默认为 0） 视觉传感器ID
    ## @param IP（默认为空） 目标IP地址
    ## @return 空
    def sendUpdateUEImage(self, vs=VisionSensorReq(), windID=0, IP=""):
        if not isinstance(vs, VisionSensorReq):
            raise Exception("Wrong data input to addVisSensor()")
        
        # 新版机制，直接会回传IP
        # # 如果是Linux系统，则获取自己的IP地址
        # if isLinux and (vs.SendProtocol[1]==127 or vs.SendProtocol[1]==0):
        #     ip=''
        #     try:
        #         s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #         s.connect(('10.254.254.254', 1))
        #         ip = s.getsockname()[0]
        #     except:
        #         ip=''
        #     finally:
        #         s.close()
        #     if ip!='':
        #         cList = ip.split(".")
        #         if len(cList) == 4:
        #             vs.SendProtocol[1] = int(cList[0])
        #             vs.SendProtocol[2] = int(cList[1])
        #             vs.SendProtocol[3] = int(cList[2])
        #             vs.SendProtocol[4] = int(cList[3])
         
        intValue = [
            vs.checksum,
            vs.SeqID,
            vs.TypeID,
            vs.TargetCopter,
            vs.TargetMountType,
            vs.DataWidth,
            vs.DataHeight,
            vs.DataCheckFreq,
        ] + vs.SendProtocol
        if self.isNewJson:  # 使用新版协议发送
            floValue = (
                [vs.CameraFOV]
                + vs.SensorPosXYZ
                + [vs.EularOrQuat]
                + vs.SensorAngEular
                + vs.SensorAngQuat
                + vs.otherParams
            )
            buf = struct.pack("16H28f", *intValue, *floValue)
        else:  # 使用旧版协议发送
            floValue = (
                [vs.CameraFOV]
                + vs.SensorPosXYZ
                + vs.SensorAngEular
                + vs.otherParams[0:8]
            )
            buf = struct.pack("16H15f", *intValue, *floValue)
        if IP == "":  # 如果指定了coptersim 电脑上的IP 使用这个值
            IP = self.ip
        self.udp_socket.sendto(buf, (IP, 20010 + windID))
        if self.RemotSendIP != "" and self.RemotSendIP != "127.0.0.1":
            self.udp_socket.sendto(buf, (self.RemotSendIP, 20010 + windID))

    ## @brief 向指定IP地址和端口发送图像更新请求
    # - @anchor sendUpdateUEImage
    ## @param vs 视觉传感器请求对象,用于初始化各种与摄像机和传感器相关的参数
    ## @param windID（默认为 0） 视觉传感器ID
    ## @param IP（默认为空） 目标IP地址
    ## @return 空
    def sendUpdateUEImaged(self, vs=VisionSensorReqNew(), windID=0, IP=""):     # SensorType == 9
        if not isinstance(vs, VisionSensorReqNew):
            raise Exception("Wrong data input to addVisSensor()")
        
        # 如果是Linux系统，则获取自己的IP地址
        if isLinux and (vs.SendProtocol[1]==127 or vs.SendProtocol[1]==0):
            ip=''
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(('10.254.254.254', 1))
                ip = s.getsockname()[0]
            except:
                ip=''
            finally:
                s.close()
            if ip!='':
                cList = ip.split(".")
                if len(cList) == 4:
                    vs.SendProtocol[1] = int(cList[0])
                    vs.SendProtocol[2] = int(cList[1])
                    vs.SendProtocol[3] = int(cList[2])
                    vs.SendProtocol[4] = int(cList[3])
         
        intValue = [
            vs.checksum,
            vs.SeqID] 
        intValue1 = [vs.bitmask]
        intValue2 = [vs.TypeID,
            vs.TargetCopter,
            vs.TargetMountType,
            vs.DataWidth,
            vs.DataHeight,
            vs.DataCheckFreq,] + vs.SendProtocol
        floValue = (
            [vs.CameraFOV]
            + vs.SensorPosXYZ
            + [vs.EularOrQuat]
            + vs.SensorAngEular
            + vs.SensorAngQuat
            + vs.otherParams
        )
        buf = struct.pack("2H1I14H28f", *intValue, *intValue1, *intValue2, *floValue)

        if IP == "":  # 如果指定了coptersim 电脑上的IP 使用这个值
            IP = self.ip
        self.udp_socket.sendto(buf, (IP, 20010 + windID))
        if self.RemotSendIP != "" and self.RemotSendIP != "127.0.0.1":
            self.udp_socket.sendto(buf, (self.RemotSendIP, 20010 + windID))

    ## @brief 用于发送命令到UE4应用程序的一个或多个窗口。
    # 它将命令转换成合适的格式，并通过UDP协议发送到指定的IP地址和端口
    def sendUE4Cmd(self, cmd, windowID=-1):
        # 如果是str类型，则转换为bytes类型
        if isinstance(cmd, str):
            cmd = cmd.encode()

        # print(type(cmd))
        if len(cmd) <= 51:
            buf = struct.pack("i52s", 1234567890, cmd)
        elif len(cmd) <= 249:
            buf = struct.pack("i252s", 1234567890, cmd)
        else:
            print("Error: Cmd is too long")
            return
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

    ## @brief 用于将视觉传感器列表发送到 RflySim3D 以请求图像
    # - @anchor sendReqToUE4
    ## @param windID（默认为 0） 窗口号
    ## @param IP（默认为空） 目标IP地址
    ## @return 空
    def sendReqToUE4(self, windID=0, IP=""):
        """send VisSensor list to RflySim3D to request image
        windID specify the index of RflySim3D window to send image
        """
        if IP == "":
            IP = self.ip

        if len(self.VisSensor) <= 0:
            print("Error: No sensor is obtained.")
            return False

        # EmptyMem = np.zeros(66,dtype=np.int).tolist()
        # buf = struct.pack("66i",*EmptyMem)
        # self.mm0.seek(0)
        # self.mm0.write(buf)
        # self.mm0.seek(0)
        contSeq0 = False
        if self.isUE4DirectUDP or self.RemotSendIP != "":
            for i in range(len(self.VisSensor)):
                if (
                    self.isUE4DirectUDP and self.VisSensor[i].SendProtocol[0] == 0
                ):  # 如果之前设置的是共享内存方式，则强制转化为UDP直发
                    self.VisSensor[i].SendProtocol[0] = 1
                if self.RemotSendIP != "":
                    cList = self.RemotSendIP.split(".")
                    if len(cList) == 4:
                        self.VisSensor[i].SendProtocol[1] = int(cList[0])
                        self.VisSensor[i].SendProtocol[2] = int(cList[1])
                        self.VisSensor[i].SendProtocol[3] = int(cList[2])
                        self.VisSensor[i].SendProtocol[4] = int(cList[3])
                if self.VisSensor[i].SeqID == 0:
                    contSeq0 = True

        if contSeq0:
            self.sendUE4Cmd("RflyClearCapture", windID)

        for i in range(len(self.VisSensor)):
            # struct VisionSensorReq {
            # 	uint16 checksum; //数据校验位，12345
            # 	uint16 SeqID; //内存序号ID
            # 	uint16 TypeID; //传感器类型ID
            # 	uint16 TargetCopter; //绑定的目标飞机     //可改变
            # 	uint16 TargetMountType; //绑定的类型    //可改变
            # 	uint16 DataWidth;   //数据或图像宽度
            # 	uint16 DataHeight; //数据或图像高度
            # 	uint16 DataCheckFreq; //检查数据更新频率
            # 	uint16 SendProtocol[8]; //传输类型（共享内存、UDP传输无压缩、UDP视频串流），IP地址，端口号，...
            # 	float CameraFOV;  //相机视场角（仅限视觉类传感器）  //可改变
            #   float EularOrQuat; //选择欧拉角或四元数方式，大于0.5就是四元数
            #   float SensorAngEular[3]; //传感器安装角度   //可改变
            #   float SensorAngQuat[4]; //传感器安装四元数   //可改变
            #   float otherParams[16]; //预留的16位数据位
            # }16H28f
            vs = self.VisSensor[i]
            
            # # 如果是Linux系统，则获取自己的IP地址
            # if isLinux and (vs.SendProtocol[1]==127 or vs.SendProtocol[1]==0):
            #     ip=''
            #     try:
            #         s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            #         s.connect(('10.254.254.254', 1))
            #         ip = s.getsockname()[0]
            #     except:
            #         ip=''
            #     finally:
            #         s.close()
            #     if ip!='':
            #         cList = ip.split(".")
            #         if len(cList) == 4:
            #             vs.SendProtocol[1] = int(cList[0])
            #             vs.SendProtocol[2] = int(cList[1])
            #             vs.SendProtocol[3] = int(cList[2])
            #             vs.SendProtocol[4] = int(cList[3])
            
            intValue = [
                vs.checksum,
                vs.SeqID,
                vs.TypeID,
                vs.TargetCopter,
                vs.TargetMountType,
                vs.DataWidth,
                vs.DataHeight,
                vs.DataCheckFreq,
            ] + vs.SendProtocol
            if self.isNewJson:  # 使用新版协议发送
                floValue = (
                    [vs.CameraFOV]
                    + vs.SensorPosXYZ
                    + [vs.EularOrQuat]
                    + vs.SensorAngEular
                    + vs.SensorAngQuat
                    + vs.otherParams
                )
                buf = struct.pack("16H28f", *intValue, *floValue)
            else:  # 使用旧版协议发送
                floValue = (
                    [vs.CameraFOV]
                    + vs.SensorPosXYZ
                    + vs.SensorAngEular
                    + vs.otherParams[0:8]
                )
                buf = struct.pack("16H15f", *intValue, *floValue)
            self.udp_socket.sendto(buf, (IP, 20010 + windID))

        time.sleep(1)

        return True

        # if IP != "127.0.0.1" or isLinux:
        #     return True

        # # struct UE4CommMemData {
        # # 	int Checksum;//校验位，设置为1234567890
        # # 	int totalNum;//最大传感器数量
        # # 	int WidthHeigh[64];//分辨率宽高序列，包含最多32个传感器的
        # # }
        # if isLinux:  # Linux下共享内存代码
        #     # Linux mmap
        #     # SHARE_MEMORY_FILE_SIZE_BYTES = 66*4
        #     f = open("/dev/shm/UE4CommMemData", "r+b")
        #     fd = f.fileno()
        #     self.mm0 = mmap.mmap(fd, 66 * 4)
        # else:  # Windows下共享内存代码
        #     self.mm0 = mmap.mmap(0, 66 * 4, "UE4CommMemData")  # 公共区

        # Data = np.frombuffer(self.mm0, dtype=np.int32)
        # checksum = Data[0]
        # totalNum = Data[1]
        # ckCheck = False
        # # print(Data)
        # if checksum >= 1234567890 and checksum <= 1234568890:
        #     CamSeqIndex = checksum - 1234567890
        #     ckCheck = True
        #     for i in range(len(self.VisSensor)):
        #         isSucc = False
        #         vs = self.VisSensor[i]
        #         idx = vs.SeqID - CamSeqIndex * 32
        #         width = Data[2 + idx * 2]
        #         height = Data[2 + idx * 2 + 1]
        #         if width == vs.DataWidth and height == vs.DataHeight:
        #             if idx <= totalNum:
        #                 isSucc = True
        #         if not isSucc:
        #             ckCheck = False
        #             break
        # if not ckCheck:
        #     print("Error: Sensor req failed from UE4.")
        #     return False
        # print("Sensor req success from UE4.")
        # self.hasReqUE4 = True
        # return True

    ## @brief 定义了一个线程，用于通过 UDP 接收图像数据包，并根据特定协议解析和处理这些数据包
    # @param CheckSum 用于存储校验和
    # @param CheckSumSize 校验和的大小（字节数）
    # @param fhead_len 数据包头的长度
    # @param imgPackUnit 图像数据包的单位大小
    # @param Frameid 帧 ID，用于区分不同帧的数据包
    # @param seqList 存储数据包的序号
    # @param dataList 存储数据包的书局
    # @param timeList 存储数据包的时间戳
    # @param recPackNum 记录接收到的包数量
    # @param timeStmpStore 存储时间戳
    # @param no_fid_len 不包含帧 ID 的包头长度
    # @param fid_len 包含帧 ID 的包头长度
    # @param dd 临时存储解析后的数据
    # 
    # 整个函数处理的过程主要为一下部分：先是数据对齐，也就是会按照数据包的形式接收数据，需要对数据包进行校验，
    # 对不符合的数据丢弃，对乱序的数据正确排序等等，也就是尽可能恢复成可用的有效的数据包
    #
    # 其次，当数据包校验后，开始对齐数据的时间戳，主要分为两部分，一是图片类型的，二是点云类型的
    # 特别说明的是，点云类型处理中，还对数据进行了归一化，然后乘上配置系数，从而得到正确的数值
    # （需要理解的是，传感器获得的数据并不是对应仿真或者现实世界的，需要转化为需要的坐标系下的数据）
    #
    # 接着是测距传感器和深度转点云的传感器处理，都是从数据中提出相应的数值进行赋值，深度转点云后面还有上述点云归一化后的处理 
    # 
    # 再到后面就是给话题名赋值，每一个typeid对应的话题名称是什么，进行赋值 
    # 
    # 最后就是对点云数据进行赋值，设置了一个pointcloud2类型的msg，进行赋值。然后区分ros版本，把数据发布出去
    def img_udp_thrdNew(self, udpSok, idx, typeID):
        CheckSum = -1
        CheckSumSize = struct.calcsize("1i")
        fhead_len = 0
        imgPackUnit = 60000
        global isEnableRosTrans
        global is_use_ros1
        Frameid = -1
        seqList = []
        dataList = []
        timeList = []
        recPackNum = 0
        timeStmpStore = 0
        no_fid_len = struct.calcsize("4i1d")
        fid_len = struct.calcsize("6i1d")
        dd = None
        packSeq = -1
        while True:
            if self.stopFlag:
                break
            if isEnableRosTrans and ((is_use_ros1 and rospy.is_shutdown())):
                break
            try:
                buf, addr = udpSok.recvfrom(imgPackUnit + 2000)  # 加一些余量，确保包头数据考虑在内
            except socket.error:
                continue

            if CheckSum == -1:  # 第一帧初始化CheckSum
                CheckSum = struct.unpack("1i", buf[0:CheckSumSize])
                if CheckSum[0] == 1234567890:  # 不包含帧id的数据协议
                    fhead_len = no_fid_len
                    Frameid = 0
                if CheckSum[0] == 1234567893:  # 包含帧ID的数据协议
                    fhead_len = fid_len
                    Frameid = 0
            if len(buf) < fhead_len:  # 如果数据包还没包头长，数据错误
                print("img_udp_thrdNew len(buf)<fhead_size")
                continue
            if fhead_len == fid_len:
                dd = struct.unpack("6i1d", buf[0:fhead_len])  # 校验，包长度，包序号，总包数，帧ID，占位，时间戳
                IsReframe = False  # 判断是否重复接受到了帧
                if dd[-3] != Frameid:  # 更新帧ID
                    if dd[-3] < 0:
                        print(
                            "\033[31m frame id less than zero ! \033[0m"
                        )  # 发送端没做int溢出处理
                    if dd[2] != 0 and self.isPrintTime: #新的一帧，包的序号必须为0
                        print("\033[31m in new frame,the first pack_seq no equal zereo \033[0m")
                    Frameid = dd[-3]
                    packSeq = dd[2]
                elif dd[2] == packSeq:
                    # print("have same frame received")  # 同一帧多次被接受
                    continue

            if fhead_len == no_fid_len:
                dd = struct.unpack("4i1d", buf[0:fhead_len])  # 校验，包长度，包序号，总包数，时间戳
            if dd == None:
                print("\033[31m Protocol error\033[0m")  # 通信协议不匹配
                continue
            if dd[0] != CheckSum[0] or dd[1] != len(buf):  # 校验位不对或者长度不对
                print("\033[31m Wrong Data!\033[0m")
                continue
            packSeq = dd[2]  # 更新包序号
            if packSeq == 0:  # 如果是第一个包
                seqList = []  # 清空数据序号列表
                dataList = []  # 清空数据缓存列表
                seqList = seqList + [packSeq]  # 提取序号
                dataList = dataList + [buf[fhead_len:]]  # 提取包头剩余数据
                timeStmpStore = dd[-1]  # 最后一个是时间戳
                recPackNum = dd[3]  # 以包头定义的总包作为接收结束标志
            else:  # 如果不是包头，直接将其存入列表
                if recPackNum == 0:
                    continue
                # 如果时间戳不一致
                if not math.isclose(timeStmpStore, dd[-1], rel_tol=0.00001) and self.isPrintTime:
                    print("时间戳不一样")
                    continue  # 跳过这个包
                seqList = seqList + [packSeq]  # 提取序号
                dataList = dataList + [buf[fhead_len:]]  # 提取包头剩余数据
            # if typeID==2:
            # print(seqList,recPackNum,len(dataList))
            if len(seqList) == recPackNum:  # 如果收到的包达到总数了，开始处理图像
                recPackNum = 0
                # print('Start Img Cap')
                data_total = b""
                dataOk = True
                for i in range(len(seqList)):
                    if seqList.count(i) < 1:
                        dataOk = False  # 如果某序号不在包中，报错
                        print("\033[31m Failed to process img pack \033[0m")
                        break
                    idx0 = seqList.index(i)  # 按次序搜索包序号
                    data_total = data_total + dataList[idx0]
                # if typeID==2:
                #    print(len(data_total))
                if dataOk:  # 如果数据都没问题，开始处理图像
                    # if typeID==2:
                    #    print('Start img cap',self.VisSensor[idx].SendProtocol[0])
                    if (
                        self.VisSensor[idx].SendProtocol[0] == 1
                        or self.VisSensor[idx].SendProtocol[0] == 3
                    ):
                        if (
                            self.VisSensor[idx].TypeID == 1
                            or self.VisSensor[idx].TypeID == 2
                            or self.VisSensor[idx].TypeID == 3
                            or self.VisSensor[idx].TypeID == 4
                            or self.VisSensor[idx].TypeID == 9      # SensorType == 9
                            or self.VisSensor[idx].TypeID == 40
                            or self.VisSensor[idx].TypeID == 41
                        ):
                            nparr = np.frombuffer(data_total, np.uint8)
                            colorType = cv2.IMREAD_COLOR
                            if typeID == 2:
                                colorType = cv2.IMREAD_ANYDEPTH
                            elif typeID == 3 or typeID == 40:
                                colorType = cv2.IMREAD_GRAYSCALE
                            self.Img_lock[idx].acquire()
                            self.Img[idx] = cv2.imdecode(nparr, colorType)
                            self.Img_lock[idx].release()
                            if self.Img[idx] is None:
                                print("\033[31m Wrong Img decode! \033[0m")
                                self.hasData[idx] = False
                            else:
                                self.hasData[idx] = True
                                self.timeStmp[idx] = timeStmpStore

                                if (
                                    self.rflyStartStmp[idx] < 0.01
                                ):  # 说明还没获取到过CopterSim时间戳
                                    print(
                                        "No CopterSim time, use image time to calculate."
                                    )
                                    if isEnableRosTrans:
                                        if is_use_ros1:
                                            ros_now_time = rospy.Time.now().to_sec()
                                        else:
                                            now = self.ros_node.get_clock().now()
                                            ros_now_time = (
                                                now.to_msg().sec + now.to_msg().nanosec * 1e-9
                                            )
                                        self.rflyStartStmp[idx] = (
                                            ros_now_time - self.timeStmp[idx] - 0.01
                                        )
                                        # tStmp.rosStartTimeStmp = tStmp.pyStartTimeStmp + ros_now_time - time.time()
                                    else:
                                        self.rflyStartStmp[idx] = (
                                            time.time() - self.timeStmp[idx] - 0.01
                                        )

                                self.imgStmp[idx] = (
                                    self.rflyStartStmp[idx] + self.timeStmp[idx]
                                )

                                if self.isPrintTime:
                                    dTime = time.time() - self.lastIMUTime
                                    print(
                                        "Img",
                                        idx,
                                        ":",
                                        "{:.5f}".format(timeStmpStore),
                                        ", dTimeIMU: ",
                                        dTime,
                                    )
                                    print("frame_id: ", Frameid, "idx: ", idx)

                        if self.VisSensor[idx].SendProtocol[0] == 1 and (
                            self.VisSensor[idx].TypeID == 20
                            or self.VisSensor[idx].TypeID == 21
                            or self.VisSensor[idx].TypeID == 22
                            or self.VisSensor[idx].TypeID == 23
                        ):
                            # print('')
                            posAng = np.frombuffer(
                                data_total, dtype=np.float32, count=6
                            )  # pos ang  6*4
                            PointNum = np.frombuffer(
                                data_total, dtype=np.int32, count=1, offset=4 * 6
                            )  # num 4*1
                            PointNum = PointNum[0]
                            # print('PointNum: ',PointNum)
                            # print('posAng: ', posAng)
                            self.ImgData[idx] = posAng.tolist() + [PointNum]

                            L = np.frombuffer(
                                data_total,
                                dtype=np.int16,
                                count=PointNum
                                * 4,  # --Lidar PointNum * 3, Fix PointNum * 4
                                offset=4 * 7,
                            )  # cloud
                            # reshape array to 4 channel image array H X W X 4
                            self.Img_lock[idx].acquire()
                            self.Img[idx] = L.reshape(
                                PointNum, 4
                            )  # --Lidar L.reshap(PointNum, 3)
                            self.Img[idx] = (
                                self.Img[idx]
                                / 32767.0
                                * self.VisSensor[idx].otherParams[0]
                            )

                            self.Img_lock[idx].release()
                            self.hasData[idx] = True
                            self.timeStmp[idx] = timeStmpStore
                            if (self.rflyStartStmp[idx] < 0.01):
                                # 说明还没获取到过CopterSim时间戳
                                print(
                                    "No CopterSim time, use image time to calculate."
                                )
                                if isEnableRosTrans:
                                    if is_use_ros1:
                                        ros_now_time = rospy.Time.now().to_sec()
                                    else:
                                        now = self.ros_node.get_clock().now()
                                        ros_now_time = (
                                            now.to_msg().sec + now.to_msg().nanosec * 1e-9
                                        )
                                    self.rflyStartStmp[idx] = (
                                        ros_now_time - self.timeStmp[idx] - 0.01
                                    )
                                    # tStmp.rosStartTimeStmp = tStmp.pyStartTimeStmp + ros_now_time - time.time()
                                else:
                                    self.rflyStartStmp[idx] = (
                                        time.time() - self.timeStmp[idx] - 0.01
                                    )

                            self.imgStmp[idx] = (
                                self.rflyStartStmp[idx] + self.timeStmp[idx])

                            if self.isPrintTime:
                                dTime = time.time() - self.lastIMUTime
                                print(
                                        "Img",
                                        idx,
                                        ":",
                                        "{:.5f}".format(timeStmpStore),
                                        ", dTimeIMU: ",
                                        dTime,
                                    )
                                print("frame_id: ", Frameid, "idx: ", idx)
                        if self.VisSensor[idx].TypeID == 5:
                            
                            self.Img_lock[idx].acquire()
                            if not isinstance(self.Img[idx], DistanceSensor):
                                self.Img[idx] = DistanceSensor()
                                
                            self.Img[idx].TimeStamp =  timeStmpStore
                            self.Img[idx].Distance = np.frombuffer(
                                data_total, dtype=np.float32, count=1, offset=0
                            )
                            self.Img[idx].CopterID = np.frombuffer(
                                data_total, dtype=np.int32, count=1, offset=4
                            )
                            self.Img[idx].RayStart = np.frombuffer(
                                data_total, dtype=np.float32, count=3, offset=8
                            )
                            self.Img[idx].Ang = np.frombuffer(
                                data_total, dtype=np.float32, count=3, offset=20
                            )
                            self.Img[idx].ImpactPoint = np.frombuffer(
                                data_total, dtype=np.float32, count=3, offset=32
                            )
                            self.Img[idx].BoxOri = np.frombuffer(
                                data_total, dtype=np.float32, count=3, offset=44
                            )
                            self.Img_lock[idx].release()
                            self.hasData[idx] = True
                            self.timeStmp[idx] = timeStmpStore
                            
                        if self.VisSensor[idx].TypeID == 7:
                            posAng = np.frombuffer(
                                data_total, dtype=np.float32, count=6
                            )  # pos ang  6*4
                            PointNum = np.frombuffer(
                                data_total, dtype=np.int32, count=1, offset=4 * 6
                            )  # num 4*1
                            PointNum = PointNum[0]
                            self.ImgData[idx] = posAng.tolist() + [PointNum]
                            L = np.frombuffer(
                                data_total,
                                dtype=np.int16,
                                count=PointNum * 3,  # --Lidar PointNum * 3, Fix PointNum * 4
                                offset=4 * 7,
                            )  # cloud
                            # reshape array to 4 channel image array H X W X 4
                            self.Img_lock[idx].acquire()
                            self.Img[idx] = L.reshape(PointNum, 3)  # --Lidar L.reshap(PointNum, 3)
                            self.Img[idx] = (
                                self.Img[idx]
                                / 32767.0
                                * self.VisSensor[idx].otherParams[0]
                            )
                            
                            self.Img_lock[idx].release()
                            self.hasData[idx] = True
                            self.timeStmp[idx] = timeStmpStore

                        if self.VisSensor[idx].TypeID == 30:
                            DataNum = np.frombuffer(
                                data_total, dtype=np.int32, count=1
                            )[0] # num
                            tempList = []
                            for i in range(DataNum):
                                CopterID = np.frombuffer(
                                data_total, dtype=np.int32, count=1, offset=4 + i*struct.calcsize("i5f")
                                )[0]
                                Credibility = np.frombuffer(
                                data_total, dtype=np.float32, count=1, offset=4 + 4 + i*struct.calcsize("i5f")
                                )[0]
                                Pos4f = np.frombuffer(
                                data_total, dtype=np.float32, count=4, offset=4 + 4 + 4 + i*struct.calcsize("i5f")
                                )
                                element = (CopterID, Credibility,*Pos4f)
                                tempList.append(element)
                            self.Img_lock[idx].acquire()
                            self.Img[idx]=tempList
                            self.Img_lock[idx].release()
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                                for j in range(len(self.Img[idx])):
                                    CopterID = self.Img[idx][j][0]
                                    Credibility=self.Img[idx][j][1]
                                    MinX=self.Img[idx][j][2]
                                    MinY=self.Img[idx][j][3]
                                    MaxX=self.Img[idx][j][4]
                                    MaxY=self.Img[idx][j][5]
                                    print(f"CopterID={CopterID},Credibility={Credibility},MinX={MinX},MinY={MinY},MaxX={MaxX},MaxY={MaxY}")

                        if self.VisSensor[idx].TypeID == 31:
                            DataNum = np.frombuffer(
                                data_total, dtype=np.int32, count=1
                            )[0] # num
                            tempList = []
                            #print(f"DataNum={DataNum}")
                            for i in range(DataNum):
                                CopterID = np.frombuffer(
                                data_total, dtype=np.int32, count=1, offset= 4 + i*struct.calcsize("i3f")
                                )
                                Pos3f = np.frombuffer(
                                data_total, dtype=np.float32, count=3, offset= 4 + 4 + i*struct.calcsize("i3f")
                                )
                                element = (*CopterID,*Pos3f)
                                tempList.append(element)
                            self.Img_lock[idx].acquire()
                            self.Img[idx]=tempList
                            self.Img_lock[idx].release()
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            for j in range(len(self.Img[idx])):
                                CopterID = self.Img[i][j][0]
                                PosX=self.Img[idx][j][1]
                                PosY=self.Img[idx][j][2]
                                PosZ=self.Img[idx][j][3]
                                print(f"CopterID={CopterID},PosX={PosX},PosY={PosY},PosZ={PosZ}")
                                
                    elif self.VisSensor[idx].SendProtocol[0] == 2:
                        dtyp = np.uint8
                        dim = 3
                        if typeID == 1 or typeID == 4 or typeID == 9 or typeID == 41:       # SensorType == 9
                            dtyp = np.uint8
                            dim = 3
                        elif typeID == 2:
                            dtyp = np.uint16
                            dim = 1
                        elif typeID == 3 or typeID == 40:
                            dtyp = np.uint8
                            dim = 1
                        DataWidth = self.VisSensor[idx].DataWidth
                        DataHeight = self.VisSensor[idx].DataHeight
                        L = np.frombuffer(data_total, dtype=dtyp)
                        # colorType=cv2.IMREAD_COLOR
                        # if typeID==2 or typeID==3:
                        #     colorType=cv2.IMREAD_GRAYSCALE
                        # self.Img[idx] = cv2.imdecode(nparr, colorType)
                        self.Img_lock[idx].acquire()
                        self.Img[idx] = L.reshape(DataHeight, DataWidth, dim)
                        self.Img_lock[idx].release()
                        self.hasData[idx] = True
                        self.timeStmp[idx] = timeStmpStore
                        if self.isPrintTime:
                            dTime = time.time() - self.lastIMUTime
                            print("Img", idx, ":", timeStmpStore, ", dTimeIMU: ", dTime)

                    if isEnableRosTrans and self.hasData[idx]:  # 如果需要发布ros消息
                        if self.VisSensor[idx].TypeID >= 1:  # 目前任务所有取图操作都同时进行
                            self.imu.AlignTime(timeStmpStore)  # 发送时间戳到imu发布线程
                        seq_id = str(self.VisSensor[idx].SeqID)

                        if self.time_record[idx] < 0.0000001:
                            self.time_record[idx] = timeStmpStore
                            if is_use_ros1:
                                self.rostime[idx] = rospy.Time.now()
                            else:
                                self.rostime[idx] = self.ros_node.get_clock().now()
                            continue

                        type_id = self.VisSensor[idx].TypeID
                        if (
                            type_id == 1
                            or type_id == 2
                            or type_id == 3
                            or type_id == 4
                            or type_id == 9
                            or type_id == 40
                            or type_id == 8
                            or type_id == 41
                        ):      # SensorType == 9
                            encoding_ = "bgr8"
                            # type = sensor.Image
                            if not seq_id in self.sensor_data.keys():
                                self.sensor_data[seq_id] = sensor.Image()
                                frame_id = "map"
                                if len(self.VisSensor) == self.sensors_num:
                                    frame_id = self.sensors_frame_id[idx]
                                self.sensor_data[seq_id].header.frame_id = frame_id
                            if is_use_ros1:
                                self.sensor_data[seq_id].header.stamp = rospy.Duration(
                                    self.imgStmp[idx]
                                )
                            else:
                                rclpy_time = self.rostime[idx] + Duration(
                                    seconds=self.imgStmp[idx],
                                    nanoseconds=0,
                                )
                                #self.sensor_data[
                                #    seq_id
                                #].header.stamp = rclpy_time.to_msg()
                                seconds, nanoseconds = rclpy_time.seconds_nanoseconds()
                                self.sensor_data[
                                   seq_id
                                ].header.stamp.sec =  int(seconds & 0x7FFFFFFF)
                                self.sensor_data[
                                   seq_id
                                ].header.stamp.nanosec =  nanoseconds
                                self.sensor_data[seq_id].header.frame_id = frame_id
                            # self.sensor_data[seq_id].header.seq = Frameid #ROS2 haven't this val
                            byte_num = 1
                            if type_id == 1:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor" + seq_id + "/img_rgb"
                                    )
                                # msg.encoding = "bgr8"
                                byte_num = 3
                            elif type_id == 2:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor" + seq_id + "/img_depth"
                                    )
                                encoding_ = "mono16"
                                byte_num = 2
                            elif type_id == 3:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor" + seq_id + "/img_gray"
                                    )
                                encoding_ = "mono8"
                            elif type_id == 9:      # SensorType == 9
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor" + seq_id + "/img_cine"
                                    )
                                byte_num = 3
                            elif type_id == 40:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor"
                                        + seq_id
                                        + "/img_Infrared_Gray"
                                    )
                                encoding_ = "mono8"
                            elif type_id == 4:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor" + seq_id + "/img_Segmentation"
                                    )
                                byte_num = 3
                            elif type_id == 8: #鱼眼相机
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = ("/rflysim/sensor" + seq_id + "/fisheye")
                                    byte_num = 3
                            elif type_id == 41:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor" + seq_id + "/img_Infrared"
                                    )
                                byte_num = 3

                            if is_use_ros1:
                                # 其实ros1中的图像也可以使用cv_bridge去转换,但是在ubuntu18.04之前的ros1默认不版本不支持python3,
                                # 强行使用python3的cv_bridge去接口去转换,ROS C++中使用话题接收时存在兼容问题.综合考虑这里使用最原始的方式去赋值.
                                self.sensor_data[seq_id].height = self.Img[idx].shape[0]
                                self.sensor_data[seq_id].width = self.Img[idx].shape[1]
                                self.sensor_data[seq_id].encoding = encoding_
                                self.sensor_data[seq_id].data = self.Img[idx].tostring()
                                self.sensor_data[seq_id].step = (
                                    self.sensor_data[seq_id].width * byte_num
                                )
                            else:
                                # 这里使用cv2_to_imgmsg实际是调用的C++接口,处理速度上比上面那种方式快,更能节省CPU资源
                                # cv2.imshow("rosdata",self.Img[idx])
                                # cv2.waitKey(1)
                                self.sensor_data[seq_id] = self.cv_bridge.cv2_to_imgmsg(self.Img[idx], encoding= encoding_)
                                self.sensor_data[seq_id].header.frame_id = frame_id

                        if type_id == 20 or type_id == 21 or type_id == 22 or type_id == 23:
                            if type_id == 20 and not seq_id in self.topic_name.keys():
                                self.topic_name[seq_id] = (
                                    "/rflysim/sensor" + seq_id + "/vehicle_lidar"
                                )
                            if type_id == 21 and not seq_id in self.topic_name.keys():
                                self.topic_name[seq_id] = (
                                    "/rflysim/sensor" + seq_id + "/global_lidar"
                                )
                            if type_id == 22 and not seq_id in self.topic_name.keys():
                                self.topic_name[seq_id] = (
                                    "/rflysim/sensor" + seq_id + "/livox_lidar"
                                )
                            if type_id == 23 and not seq_id in self.topic_name.keys():
                                self.topic_name[seq_id] = (
                                    "/rflysim/sensor" + seq_id + "/mid360_lidar"
                                )
                            if not seq_id in self.sensor_data.keys():
                                # type = sensor.PointCloud2
                                msg = sensor.PointCloud2()
                                msg.height = 1
                                msg.width = self.Img[idx].shape[0]
                                msg.fields = [
                                    sensor.PointField(
                                        name = "x", offset = 0, datatype = sensor.PointField.FLOAT32, count = 1
                                    ),
                                    sensor.PointField(
                                        name = "y", offset = 4, datatype = sensor.PointField.FLOAT32, count = 1
                                    ),
                                    sensor.PointField(
                                        name = "z", offset = 8, datatype = sensor.PointField.FLOAT32, count = 1
                                    ),
                                    sensor.PointField(
                                        name = "seg", offset = 12, datatype = sensor.PointField.FLOAT32, count = 1
                                    )
                                ]
                                msg.is_bigendian = False
                                msg.point_step = 16
                                msg.is_dense = False
                                self.sensor_data[seq_id] = msg
                                frame_id = "map"
                                if len(self.VisSensor) == self.sensors_num:
                                    frame_id = self.sensors_frame_id[idx]
                                self.sensor_data[seq_id].header.frame_id = frame_id
                            # self.sensor_data[seq_id].header.seq = Frameid
                            self.sensor_data[seq_id].width = self.Img[idx].shape[0]
                            self.sensor_data[seq_id].row_step = (
                                self.sensor_data[seq_id].point_step
                                * self.Img[idx].shape[0]
                            )

                            self.sensor_data[seq_id].data = np.asarray(
                                self.Img[idx], np.float32
                            ).tostring()
                        if type_id == 7:
                            if type_id == 7 and not seq_id in self.topic_name.keys():
                                self.topic_name[seq_id] = (
                                    "/rflysim/sensor" + seq_id + "/Depth_Cloud"
                                )

                            if not seq_id in self.sensor_data.keys():
                                # type = sensor.PointCloud2
                                msg = sensor.PointCloud2()
                                msg.height = 1
                                msg.width = self.Img[idx].shape[0]
                                msg.fields = [
                                    sensor.PointField(
                                        name = "x", offset = 0, datatype = sensor.PointField.FLOAT32, count = 1
                                    ),
                                    sensor.PointField(
                                        name = "y", offset = 4, datatype = sensor.PointField.FLOAT32, count = 1
                                    ),
                                    sensor.PointField(
                                        name = "z", offset = 8, datatype = sensor.PointField.FLOAT32, count = 1
                                    )
                                ]
                                print("pointCloud Num: ", len(msg.fields))
                                msg.is_bigendian = False
                                msg.point_step = 12
                                msg.is_dense = False
                                self.sensor_data[seq_id] = msg
                                frame_id = "map"
                                if len(self.VisSensor) == self.sensors_num:
                                    frame_id = self.sensors_frame_id[idx]
                                self.sensor_data[seq_id].header.frame_id = frame_id
                            self.sensor_data[seq_id].header.seq = Frameid
                            self.sensor_data[seq_id].width = self.Img[idx].shape[0]
                            self.sensor_data[seq_id].row_step = (
                                self.sensor_data[seq_id].point_step
                                * self.Img[idx].shape[0]
                            )

                            self.sensor_data[seq_id].data = np.asarray(
                                self.Img[idx], np.float32
                            ).tostring()


                        if is_use_ros1:
                            if not self.topic_name[seq_id] in self.sensor_pub.keys():
                                self.sensor_pub[
                                    self.topic_name[seq_id]
                                ] = rospy.Publisher(
                                    self.topic_name[seq_id],
                                    type(self.sensor_data[seq_id]),
                                    queue_size=10,
                                )
                            self.sensor_data[seq_id].header.stamp = rospy.Duration(
                                self.imgStmp[idx]
                            )
                            self.sensor_pub[self.topic_name[seq_id]].publish(
                                self.sensor_data[seq_id]
                            )
                        else:
                            if not self.topic_name[seq_id] in self.sensor_pub.keys():
                                self.sensor_pub[
                                    self.topic_name[seq_id]
                                ] = self.ros_node.create_publisher(
                                    type(self.sensor_data[seq_id]),
                                    self.topic_name[seq_id],
                                    qos_profile=qos_profile_sensor_data,
                                )
                            rclpy_time = self.rostime[idx] + Duration(
                                seconds=(timeStmpStore - self.time_record[idx]),
                                nanoseconds=0,
                            )
                            # self.sensor_data[seq_id].header.stamp = rclpy_time.to_msg()
                            seconds, nanoseconds = rclpy_time.seconds_nanoseconds()
                            self.sensor_data[seq_id].header.stamp.sec = int(seconds & 0x7FFFFFFF)
                            self.sensor_data[seq_id].header.stamp.nanosec = nanoseconds
                            self.sensor_pub[self.topic_name[seq_id]].publish(
                                self.sensor_data[seq_id]
                            )
        udpSok.close()

    ## @brief 从共享内存读取数据并进行处理，以不同的传感器类型来解析数据
    # @param mmList 一个存储内存映射对象 (mmap) 的列表，用于管理多个传感器的共享内存映射
    # @param dim 表示数据的维度
    # @param dimSize 表示每个维度的数据大小
    # @param otherSize 表示除主要数据外的额外数据大小（以字节为单位）
    # 
    # 与上面的img_udp_thrdNew析构函数主要区别就在与，这个析构函数是mm作为数据流，img_udp_thrdNew是data_total 
    # 
    # 值得注意的是，mm的获取通过共享内存，通过mm = mmap.mmap()函数实现
    # 
    # 而data_total数据来源于buf, addr = udpSok.recvfrom()得到的buf（也就是通过UDP传输）  
    def img_mem_thrd(self, idxList):
        global isEnableRosTrans
        global is_use_ros1
        mmList = []
        for i in range(len(idxList)):
            idx = idxList[i]
            SeqID = self.VisSensor[idx].SeqID
            DataWidth = self.VisSensor[idx].DataWidth
            DataHeight = self.VisSensor[idx].DataHeight
            typeID = self.VisSensor[idx].TypeID
            dim = 3
            dimSize = 1
            otherSize = 0
            if typeID == 1 or typeID == 4 or typeID == 9 or typeID == 8 or typeID == 41:       # SensorType == 9
                dim = 3
                dimSize = 1
            elif typeID == 2:
                dim = 1
                dimSize = 2
            elif typeID == 3 or typeID == 40:
                dim = 1
                dimSize = 1
            elif typeID == 20 or typeID == 21 or typeID == 22:
                # dim = 3
                dim = 4  # --Lidar  dim = 3
                dimSize = 2
                otherSize = 4 * 7
            elif typeID == 5:  # 1 + 8 + 14 * 4
                DataWidth = 0
                DataHeight = 0
                dim = 0
                dimSize = 0
                otherSize = 14 * 4
            elif typeID == 7:
                dim = 3
                dimSize = 2
                otherSize = 4 * 7
            elif typeID == 23:
                DataWidth = 64
                DataHeight = 272
                dim = 4  # --Lidar  dim = 3
                dimSize = 2
                otherSize = 4 * 7
            elif typeID == 30:
                DataWidth = 0
                DataHeight = 0
                dim = 0
                dimSize = 0
                otherSize = 4 + self.VisSensor[idx].otherParams[1] * struct.calcsize("i5f") #int数量+最大数量*每个目标的大小
                #otherParams[0]是最大探测距离、otherParams[1]是最大捕获数量
            elif typeID == 31:
                DataWidth = 0
                DataHeight = 0
                dim = 0
                dimSize = 0
                otherSize = 4 + self.VisSensor[idx].otherParams[1] * struct.calcsize("i3f") #int数量+最大数量*每个目标的大小
                #otherParams[0]是最大探测距离、otherParams[1]是最大捕获数量

            if isLinux:
                # Linux
                dataLen = DataWidth * DataHeight * dim * dimSize + 1 + 8 + otherSize
                f = open("/dev/shm/" + "RflySim3DImg_" + str(SeqID), "r+b")
                fd = f.fileno()
                mm = mmap.mmap(fd, dataLen)
                # mm = mmap_file.read(SHARE_MEMORY_FILE_SIZE_BYTES)
            else:
                mm = mmap.mmap(
                    0,
                    DataWidth * DataHeight * dim * dimSize + 1 + 8 + otherSize,
                    "RflySim3DImg_" + str(SeqID),
                )
            mmList = mmList + [mm]
        # cv2.IMWRITE_PAM_FORMAT_GRAYSCALE
        while True:
            if self.stopFlag:
                break
            if isEnableRosTrans and ((is_use_ros1 and rospy.is_shutdown())):
                break
            for kk in range(len(idxList)):
                mm = mmList[kk]
                idx = idxList[kk]
                DataWidth = self.VisSensor[idx].DataWidth
                DataHeight = self.VisSensor[idx].DataHeight
                typeID = self.VisSensor[idx].TypeID
                dtyp = np.uint8
                dim = 3
                if typeID == 1 or typeID == 4 or typeID == 9 or typeID == 8 or typeID == 41:       # SensorType == 9
                    dtyp = np.uint8
                    dim = 3
                elif typeID == 2:
                    dtyp = np.uint16
                    dim = 1
                elif typeID == 3 or typeID == 40:
                    dtyp = np.uint8
                    dim = 1
                elif typeID == 20 or typeID == 21 or typeID == 22 or typeID == 23:
                    dtyp = np.int16
                    dim = 4  # --Lidar   dim = 3
                elif typeID == 5:
                    dtyp = np.float32
                    dim = 0
                elif typeID == 7:
                    dim = 3
                for ii in range(3):  # 尝试读取三次内存区域
                    flag = np.frombuffer(mm, dtype=np.uint8, count=1)
                    # print(flag[0])
                    if flag[0] == 2:  # 图像已写入完成
                        # print(flag[0])
                        mm.seek(0)
                        mm.write_byte(3)  # 进入读取状态

                        # 开始读取图片
                        # L=np.frombuffer(mm,dtype = np.uint8)
                        # struct.unpack('d',L[1:9]) #获得时间戳
                        self.timeStmp[idx] = np.frombuffer(
                            mm, dtype=np.float64, count=1, offset=1
                        )

                        if self.rflyStartStmp[idx] < 0.01:  # 说明还没获取到过CopterSim时间戳
                            print("No CopterSim time, use image time to calculate.")
                            if isEnableRosTrans:
                                if is_use_ros1:
                                    ros_now_time = rospy.Time.now().to_sec()
                                else:
                                    now = self.ros_node.get_clock().now()
                                    ros_now_time = (
                                        now.to_msg().sec + now.to_msg().nanosec * 1e-9
                                    )
                                self.rflyStartStmp[idx] = (
                                    ros_now_time - self.timeStmp[idx] - 0.01
                                )
                                # tStmp.rosStartTimeStmp = tStmp.pyStartTimeStmp + ros_now_time - time.time()
                            else:
                                self.rflyStartStmp[idx] = (
                                    time.time() - self.timeStmp[idx] - 0.01
                                )

                        self.imgStmp[idx] = self.rflyStartStmp[idx] + self.timeStmp[idx]

                        if self.isPrintTime:
                            dTime = time.time() - self.lastIMUTime
                            print(
                                "Img",
                                idx,
                                ":",
                                self.timeStmp[idx],
                                ", dTimeIMU: ",
                                dTime,
                            )

                        mm.seek(0)
                        mm.write_byte(4)  # 进入读取完成状态
                        if (
                            typeID == 1
                            or typeID == 2
                            or typeID == 3
                            or typeID == 4
                            or typeID == 9
                            or typeID == 8
                            or typeID == 40
                            or typeID == 41
                        ):      # SensorType == 9
                            L = np.frombuffer(mm, dtype=dtyp, offset=9)
                            # reshape array to 4 channel image array H X W X 4
                            self.Img_lock[idx].acquire()
                            self.Img[idx] = L.reshape(DataHeight, DataWidth, dim)
                            self.Img_lock[idx].release()
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            if self.isRemoteSend:
                                self.sendImgUDPNew(idx)

                        elif typeID == 20 or typeID == 21 or typeID == 22:
                            posAng = np.frombuffer(
                                mm, dtype=np.float32, count=6, offset=9
                            )  # pos ang
                            PointNum = np.frombuffer(
                                mm, dtype=np.int32, count=1, offset=9 + 4 * 6
                            )  # num
                            PointNum = PointNum[0]
                            # print('PointNum: ',PointNum)
                            # print('posAng: ', posAng)
                            self.ImgData[idx] = posAng.tolist() + [PointNum]

                            L = np.frombuffer(
                                mm, dtype=dtyp, count=PointNum * dim, offset=9 + 4 * 7
                            )  # cloud
                            # reshape array to 4 channel image array H X W X 4
                            self.Img_lock[idx].acquire()
                            self.Img[idx] = L.reshape(PointNum, dim)
                            self.Img[idx] = (
                                self.Img[idx]
                                / 32767.0
                                * self.VisSensor[idx].otherParams[0]
                            )
                            self.Img_lock[idx].release()
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            if self.isRemoteSend:
                                # pos ang num cloud
                                L = np.frombuffer(
                                    mm,
                                    dtype=np.uint8,
                                    count=PointNum * dim * 2 + 4 * 7,
                                    offset=9,
                                )
                                self.sendImgBuffer(idx, L.tostring())
                        elif typeID == 5:
                            
                            self.Img_lock[idx].acquire()
                            if not isinstance(self.Img[idx], DistanceSensor):
                                self.Img[idx] = DistanceSensor()
                            flag = np.frombuffer(mm, dtype=np.uint8, count=1, offset=0)
                            self.Img[idx].TimeStamp = np.frombuffer(
                                mm, dtype=np.float64, count=1, offset=1
                            )
                            self.Img[idx].Distance = np.frombuffer(
                                mm, dtype=np.float32, count=1, offset=9
                            )
                            self.Img[idx].CopterID = np.frombuffer(
                                mm, dtype=np.int32, count=1, offset=13
                            )
                            self.Img[idx].RayStart = np.frombuffer(
                                mm, dtype=np.float32, count=3, offset=17
                            )
                            self.Img[idx].Ang = np.frombuffer(
                                mm, dtype=np.float32, count=3, offset=29
                            )
                            self.Img[idx].ImpactPoint = np.frombuffer(
                                mm, dtype=np.float32, count=3, offset=41
                            )
                            self.Img[idx].BoxOri = np.frombuffer(
                                mm, dtype=np.float32, count=3, offset=53
                            )
                            self.Img_lock[idx].release()
                            self.hasData[idx] = True
                            self.timeStmp[idx] = timeStmpStore
                            
                        elif typeID == 7:
                            posAng = np.frombuffer(
                                mm, dtype=np.float32, count=6, offset=9
                            )  # pos ang
                            PointNum = np.frombuffer(
                                mm, dtype=np.int32, count=1, offset=9 + 4 * 6
                            )  # num
                            PointNum = PointNum[0]
                            #print('PointNum: ',PointNum)
                            #print(len(mm))
                            self.ImgData[idx] = posAng.tolist() + [PointNum]

                            L = np.frombuffer(
                                mm, dtype=np.int16, count=PointNum * dim, offset=9 + 4 * 7
                            )  # cloud
                            # reshape array to 4 channel image array H X W X 4
                            self.Img_lock[idx].acquire()
                            self.Img[idx] = L.reshape(PointNum, dim)
                            self.Img[idx] = (
                                self.Img[idx]
                                / 32767.0
                                * self.VisSensor[idx].otherParams[0]
                            )
                            self.Img_lock[idx].release()
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            if self.isRemoteSend:
                                # pos ang num cloud
                                L = np.frombuffer(
                                    mm,
                                    dtype=np.uint8,
                                    count=PointNum * dim * 2 + 4 * 7,
                                    offset=9,
                                )
                                self.sendImgBuffer(idx, L.tostring())
                        elif typeID == 30:
                            DataNum = np.frombuffer(
                                mm, dtype=np.int32, count=1, offset=9
                            )[0] # num
                            tempList = []
                            for i in range(DataNum):
                                CopterID = np.frombuffer(
                                mm, dtype=np.int32, count=1, offset=9 + 4 + i*struct.calcsize("i5f")
                                )[0]
                                Credibility = np.frombuffer(
                                mm, dtype=np.float32, count=1, offset=9 + 4 + 4 + i*struct.calcsize("i5f")
                                )[0]
                                Pos4f = np.frombuffer(
                                mm, dtype=np.float32, count=4, offset=9 + 4 + 4 + 4 + i*struct.calcsize("i5f")
                                )
                                element = (CopterID, Credibility,*Pos4f)
                                tempList.append(element)
                            self.Img_lock[idx].acquire()
                            self.Img[idx]=tempList
                            self.Img_lock[idx].release()
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            if self.isRemoteSend:
                                pass
                                #暂不支持
                        elif typeID == 31:
                            DataNum = np.frombuffer(
                                mm, dtype=np.int32, count=1, offset=9
                            )[0] # num
                            tempList = []
                            print(f"DataNum={DataNum}")
                            for i in range(DataNum):
                                CopterID = np.frombuffer(
                                mm, dtype=np.int32, count=1, offset=9 + 4 + i*struct.calcsize("i3f")
                                )
                                Pos3f = np.frombuffer(
                                mm, dtype=np.float32, count=3, offset=9 + 4 + 4 + i*struct.calcsize("i3f")
                                )
                                element = (*CopterID,*Pos3f)
                                tempList.append(element)
                            self.Img_lock[idx].acquire()
                            self.Img[idx]=tempList
                            self.Img_lock[idx].release()
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            if self.isRemoteSend:
                                pass
                                #暂不支持
                        # 读取到图片后就退出for循环
                        # print("readImg"+str(idx))
                        # Linux ROS话题发布
                        if isEnableRosTrans and self.hasData[idx]:  # 如果需要发布ros消息
                            if self.VisSensor[idx].TypeID >= 1:  # 目前任务所有取图操作都同时进行
                                self.imu.AlignTime(self.timeStmp[idx])  # 发送时间戳到imu发布线程
                            topic_name = "/rflysim/sensor" + str(
                                self.VisSensor[idx].SeqID
                            )
                            frame_id = "map"  # 为了方便可视化，使用默认frame_id，在算法里使用时，需要根据实际情况修改
                            # 为了方便可视化，使用默认frame_id，在算法里使用时，需要根据实际情况修改
                            if len(self.VisSensor) == self.sensors_num:
                                frame_id = self.sensors_frame_id[idx]
                            header = std_msg.Header()
                            # header.seq = Frameid
                            header.frame_id = frame_id
                            if is_use_ros1:
                                header.stamp = rospy.Duration(self.imgStmp[idx])
                            else:
                                rclpy_time = Duration(
                                    seconds=(self.imgStmp[idx]),
                                    nanoseconds=0,
                                )
                                seconds, nanoseconds = rclpy_time.seconds_nanoseconds()
                                header.stamp.sec = int(seconds &0x7FFFFFFF)
                                header.stamp.nanosec  = nanoseconds

                            type_id = self.VisSensor[idx].TypeID

                            # print('Img',idx,':',header.stamp.to_sec())

                            type = Any
                            msg = Any
                            if (
                                type_id == 1
                                or type_id == 2
                                or type_id == 3
                                or type_id == 4
                                or type_id == 9
                                or type_id == 40
                                or type_id == 41
                            ):      # SensorType == 9
                                encoding_ = "bgr8"
                                type = sensor.Image
                                msg = sensor.Image()
                                byte_num = 1
                                msg.header = header
                                if type_id == 1:
                                    topic_name += "/img_rgb"
                                    # msg.encoding = "bgr8"
                                    byte_num = 3
                                elif type_id == 2:
                                    encoding_ = "mono16"
                                    topic_name += "/img_depth"
                                    byte_num = 2
                                elif type_id == 3:
                                    encoding_ = "mono8"
                                    topic_name += "/img_gray"
                                elif type_id == 9:      # SensorType == 9
                                    topic_name += "/img_cine"
                                    byte_num = 3
                                elif type_id == 40:
                                    encoding_ = "mono8"
                                    topic_name += "/img_Infrared_Gray"
                                elif type_id == 4:
                                    topic_name += "/img_Segmentation"
                                    byte_num = 3
                                else:
                                    topic_name += "/img_Infrared"
                                    byte_num = 3
                                msg.height = self.Img[idx].shape[0]
                                msg.width = self.Img[idx].shape[1]
                                msg.encoding = encoding_
                                msg.data = self.Img[idx].tostring()
                                msg.step = msg.width * byte_num
                                # print(encoding_)
                            if type_id == 20 or type_id == 21 or type_id == 22:
                                type = sensor.PointCloud2
                                msg = sensor.PointCloud2()
                                msg.header = header
                                if type_id == 20:
                                    topic_name += "/vehicle_lidar"  # Topic /rflysim/senso3/vehicle_lidar
                                if type_id == 21:
                                    topic_name += "/global_lidar"
                                if type_id == 22:
                                    topic_name += "/livox_lidar"
                                msg.height = 1
                                msg.width = self.Img[idx].shape[0]
                                msg.fields = [
                                    sensor.PointField(
                                        "x", 0, sensor.PointField.FLOAT32, 1
                                    ),
                                    sensor.PointField(
                                        "y", 4, sensor.PointField.FLOAT32, 1
                                    ),
                                    sensor.PointField(
                                        "z", 8, sensor.PointField.FLOAT32, 1
                                    ),
                                    sensor.PointField(  # --Lidar
                                        "w", 12, sensor.PointField.FLOAT32, 1
                                    ),
                                ]
                                msg.is_bigendian = False
                                msg.point_step = 16  # --Lidar
                                msg.row_step = msg.point_step * self.Img[idx].shape[0]
                                msg.is_dense = False
                                msg.data = np.asarray(
                                    self.Img[idx], np.float32
                                ).tostring()
                            if type_id == 7:
                                type = sensor.PointCloud2
                                msg = sensor.PointCloud2()
                                msg.header = header
                                if type_id == 7:
                                    topic_name += "/Depth_Cloud"  # Topic /rflysim/senso3/vehicle_lidar
                                msg.height = 1
                                msg.width = self.Img[idx].shape[0]
                                msg.fields = [
                                    sensor.PointField(
                                        "x", 0, sensor.PointField.FLOAT32, 1
                                    ),
                                    sensor.PointField(
                                        "y", 4, sensor.PointField.FLOAT32, 1
                                    ),
                                    sensor.PointField(
                                        "z", 8, sensor.PointField.FLOAT32, 1
                                    )
                                ]
                                msg.is_bigendian = False
                                msg.point_step = 12  # --Lidar
                                msg.row_step = msg.point_step * self.Img[idx].shape[0]
                                msg.is_dense = False
                                msg.data = np.asarray(
                                    self.Img[idx], np.float32
                                ).tostring()
                            if is_use_ros1:
                                if not topic_name in self.sensor_pub.keys():
                                    self.sensor_pub[topic_name] = rospy.Publisher(
                                        topic_name, type, queue_size=10
                                    )
                                self.sensor_pub[topic_name].publish(msg)
                            else:
                                if not topic_name in self.sensor_pub.keys():
                                    self.sensor_pub[
                                        topic_name
                                    ] = self.ros_node.create_publisher(
                                        type, topic_name, 10
                                    )
                                self.sensor_pub[topic_name].publish(msg)

                        break
            time.sleep(0.001)

    ## @brief 用于启动图像捕获的循环过程，先是一些初始化，然后根据SendProtocol[0]来判断是内存共享还是UDP传输，分别进行相应的处理函数
    # @brief memList 存储需要从内存接收图像的传感器索引列表
    # @brief udpList 存储需要从UDP接收图像的传感器索引列表 
    # @param isRemoteSend（默认为False） 是否将图像从共享内存转发到UDP端口
    # - False表示将将图像从共享内存转发到UDP端口
    # - True表示将将图像从共享内存转发到UDP端口
    def startImgCap(self, isRemoteSend=False):
        """start loop to receive image from UE4,
        isRemoteSend=true will forward image from memory to UDP port
        """
        self.isRemoteSend = isRemoteSend
        global isEnableRosTrans
        memList = []
        udpList = []
        if isEnableRosTrans:
            self.time_record = np.zeros(len(self.VisSensor))
            if is_use_ros1:
                self.rostime = np.ndarray(len(self.time_record), dtype=rospy.Time)
            else:
                self.rostime = np.ndarray(len(self.time_record), dtype=rclpy.time.Time)

        for i in range(len(self.VisSensor)):
            self.Img = self.Img + [0]
            self.Img_lock = self.Img_lock + [
                threading.Lock()
            ]  # 每个传感器都是一个独立的线程，应时使用独立的锁
            self.ImgData = self.ImgData + [0]
            self.hasData = self.hasData + [False]
            self.timeStmp = self.timeStmp + [0]
            self.imgStmp = self.imgStmp + [0]

            TarCopt = self.VisSensor[i].TargetCopter
            starTime = 0
            for j in range(len(self.RflyTimeVect)):
                if self.RflyTimeVect[j].copterID == TarCopt:
                    if isEnableRosTrans:
                        starTime = self.RflyTimeVect[j].rosStartTimeStmp
                    else:
                        starTime = self.RflyTimeVect[j].pyStartTimeStmp
                    print("Got start time for SeqID #", self.VisSensor[i].SeqID)
            self.rflyStartStmp = self.rflyStartStmp + [starTime]
            IP = (
                str(self.VisSensor[i].SendProtocol[1])
                + "."
                + str(self.VisSensor[i].SendProtocol[2])
                + "."
                + str(self.VisSensor[i].SendProtocol[3])
                + "."
                + str(self.VisSensor[i].SendProtocol[4])
            )
            if IP == "0.0.0.0":
                IP = "127.0.0.1"
            if self.RemotSendIP != "":
                IP = self.RemotSendIP
            self.IpList = self.IpList + [IP]
            self.portList = self.portList + [self.VisSensor[i].SendProtocol[5]]
            if self.VisSensor[i].SendProtocol[0] == 0:
                memList = memList + [i]
            else:
                udpList = udpList + [i]

        if len(memList) > 0:
            self.t_menRec = threading.Thread(target=self.img_mem_thrd, args=(memList,))
            self.t_menRec.start()

        if len(udpList) > 0:
            # print('Enter UDP capture')
            for i in range(len(udpList)):
                udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 60000 * 100)
                udp.bind(("0.0.0.0", self.portList[udpList[i]]))
                typeID = self.VisSensor[udpList[i]].TypeID
                t_udpRec = threading.Thread(
                    target=self.img_udp_thrdNew,
                    args=(
                        udp,
                        udpList[i],
                        typeID,
                    ),
                )
                t_udpRec.start()

    ## @brief 用于图像数据的UDP发送
    # - @anchor sendImgUDPNew
    # @param idx 图像的索引
    # @return 空
    def sendImgUDPNew(self, idx):
        img_encode = cv2.imencode(".png", self.Img[idx])[1]
        data_encode = np.array(img_encode)
        data = data_encode.tostring()
        self.sendImgBuffer(idx, data)

    ## @brief 用于将图像数据分割为固定大小的数据包，并添加头部信息后发送
    # -@anchor sendImgBuffer
    # @param idx 图像的索引
    # @param data 图像数据
    # - imgPackUnit 定义了每个数据包的最大大小
    # - imgLen 表示整个图像数据的长度，即数据流的总字节数
    # - imgpackNum 表示将图像数据分割成的数据包数量
    # - CheckSum 用作校验和，确保数据在传输过程中的完整性
    # - timeStmpSend 表示发送图像数据时的时间戳
    # @return 空
    def sendImgBuffer(self, idx, data):
        imgPackUnit = 60000
        imgLen = len(data)
        imgpackNum = imgLen // imgPackUnit + 1
        IP = self.IpList[idx]
        if self.RemotSendIP != "":
            IP = self.RemotSendIP

        CheckSum = 1234567890
        timeStmpSend = self.timeStmp[idx]

        # 循环发送图片码流
        for i in range(imgpackNum):
            dataSend = []
            if imgPackUnit * (i + 1) > len(data):  # 末尾包数据提取
                dataSend = data[imgPackUnit * i :]
            else:  # 前面数据包直接提取60000数据
                dataSend = data[imgPackUnit * i : imgPackUnit * (i + 1)]
            PackLen = 4 * 4 + 8 * 1 + len(dataSend)  # fhead的4i1d长度，加上图片数据长度
            fhead = struct.pack(
                "4i1d", CheckSum, PackLen, i, imgpackNum, timeStmpSend
            )  # 校验，包长度，包序号，总包数，时间戳
            dataSend = fhead + dataSend  # 包头加上图像数据
            self.udp_socket.sendto(dataSend, (IP, self.portList[idx]))  # 发送出去

    ## @brief 用来从配置文件 Config.json 中加载相机配置信息，并根据配置信息创建相机列表
    # @ChangeMode（默认为-1） 转化模式
    # - -1表示默认不切换模式
    # - >=0表示将SendProtocol[0]设置为转化模式
    # @param jsonPath（默认为空） 配置文件的路径
    # 先选中json文件，然后读取文件内容，检查内容是否符合格式（是否为int类型等等），最后赋值
    def jsonLoad(self, ChangeMode=-1, jsonPath=""):
        """load config.json file to create camera list for image capture,
        if ChangeMode>=0, then the SendProtocol[0] will be set to ChangeMode to change the transfer style
        """
        # print(sys.path[0])
        if os.path.isabs(jsonPath):
            print("Json use absolute path mode")
        else:
            print("Json use relative path mode")
            if len(jsonPath) == 0:
                jsonPath = os.path.dirname(os.path.realpath(sys.argv[0])) + "/Config.json"
            else:
                jsonPath = os.path.dirname(os.path.realpath(sys.argv[0])) + "/" + jsonPath

        print("jsonPath=", jsonPath)

        if not os.path.exists(jsonPath):
            print("The json file does not exist!")
            return False
        self.isNewJson = False
        
        # 加载Json并删除注释语句
        lines = []
        with open(jsonPath, "r", encoding="utf-8") as f:
            for row in f.readlines(): # 第二步：读取文件内容 
                if row.strip().startswith("//") or row.strip().startswith("#"):   # 第三步：对每一行进行过滤 
                    continue
                row = re.sub('//.*', '', row)       # 去掉//打头的字符串
                row = re.sub('#.*', '', row)       # 去掉#打头的字符串
                lines.append(row)                   # 第四步：将过滤后的行添加到列表中.
        #print(lines)
        jsData = json.loads("\n".join(lines))
        if len(jsData["VisionSensors"]) <= 0:
            print("No sensor data is found!")
            return False
        
        CurID=-1
        for i in range(len(jsData["VisionSensors"])):
            visSenStructTemp = VisionSensorReqNew()     # SensorType == 9
            visSenStruct = VisionSensorReq()
            if isinstance(jsData["VisionSensors"][i]["SeqID"], int):
                visSenStruct.SeqID = jsData["VisionSensors"][i]["SeqID"]
                
                if visSenStruct.SeqID==0: # 如果自动确定SeqID的方式
                    if CurID>=0: # 如果前面已有序号
                        visSenStruct.SeqID = CurID+1 # 自增
                CurID = visSenStruct.SeqID
                
            else:
                print("Json data format is wrong!")
                continue

            if isinstance(jsData["VisionSensors"][i]["TypeID"], int):
                visSenStruct.TypeID = jsData["VisionSensors"][i]["TypeID"]
            else:
                print("Json data format is wrong!")
                continue

            if isinstance(jsData["VisionSensors"][i]["TargetCopter"], int):
                visSenStruct.TargetCopter = jsData["VisionSensors"][i][
                    "TargetCopter"
                ]
            else:
                print("Json data format is wrong!")
                continue

            if isinstance(jsData["VisionSensors"][i]["TargetMountType"], int):
                visSenStruct.TargetMountType = jsData["VisionSensors"][i][
                    "TargetMountType"
                ]
            else:
                print("Json data format is wrong!")
                continue

            if isinstance(jsData["VisionSensors"][i]["DataWidth"], int):
                visSenStruct.DataWidth = jsData["VisionSensors"][i]["DataWidth"]
            else:
                print("Json data format is wrong!")
                continue

            if isinstance(jsData["VisionSensors"][i]["DataHeight"], int):
                visSenStruct.DataHeight = jsData["VisionSensors"][i]["DataHeight"]
            else:
                print("Json data format is wrong!")
                continue

            if isinstance(jsData["VisionSensors"][i]["DataCheckFreq"], int):
                visSenStruct.DataCheckFreq = jsData["VisionSensors"][i][
                    "DataCheckFreq"
                ]
            else:
                print("Json data format is wrong!")
                continue

            if isinstance(
                jsData["VisionSensors"][i]["CameraFOV"], float
            ) or isinstance(jsData["VisionSensors"][i]["CameraFOV"], int):
                visSenStruct.CameraFOV = jsData["VisionSensors"][i]["CameraFOV"]
            else:
                print("Json data format is wrong!")
                continue

            if len(jsData["VisionSensors"][i]["SendProtocol"]) == 8:
                visSenStruct.SendProtocol = jsData["VisionSensors"][i][
                    "SendProtocol"
                ]
                if ChangeMode != -1:
                    # 如果是远程接收模式，那么读图这里需要配置为UDP接收
                    visSenStruct.SendProtocol[0] = ChangeMode
                
                #print('JsonData',visSenStruct.SendProtocol[0],isLinux,ChangeMode)
                
                # 如果是共享内存模式，且在Linux系统下，则强制切换为UDP模式
                if visSenStruct.SendProtocol[0]==0 and isLinux and ChangeMode==-1:
                    visSenStruct.SendProtocol[0]=1 # 切换为UDP模式
                    
                if visSenStruct.SendProtocol[5]==0: # 自动确定端口方式
                    visSenStruct.SendProtocol[5] = 9999+CurID # 使用默认自增端口的方式
                
            else:
                print("Json data format is wrong!")
                continue

            if len(jsData["VisionSensors"][i]["SensorPosXYZ"]) == 3:
                visSenStruct.SensorPosXYZ = jsData["VisionSensors"][i][
                    "SensorPosXYZ"
                ]
            else:
                print("Json data format is wrong!")
                continue

            isNewProt = False

            if "EularOrQuat" in jsData["VisionSensors"][i]:
                isNewProt = True
                visSenStruct.EularOrQuat = jsData["VisionSensors"][i]["EularOrQuat"]
            else:
                visSenStruct.EularOrQuat = 0

            if len(jsData["VisionSensors"][i]["SensorAngEular"]) == 3:
                visSenStruct.SensorAngEular = jsData["VisionSensors"][i][
                    "SensorAngEular"
                ]
            else:
                print("Json data format is wrong!")
                continue

            if isNewProt:
                if len(jsData["VisionSensors"][i]["SensorAngQuat"]) == 4:
                    visSenStruct.SensorAngQuat = jsData["VisionSensors"][i][
                        "SensorAngQuat"
                    ]
                else:
                    print("Json data format is wrong!")
                    continue

            if isNewProt:  # 新协议使用16维的otherParams
                if len(jsData["VisionSensors"][i]["otherParams"]) == 16:
                    visSenStruct.otherParams = jsData["VisionSensors"][i][
                        "otherParams"
                    ]
                else:
                    print("Json data format is wrong!")
                    continue
            else:
                if len(jsData["VisionSensors"][i]["otherParams"]) == 8:
                    visSenStruct.otherParams = (
                        jsData["VisionSensors"][i]["otherParams"] + [0] * 8
                    )  # 扩展到16维
                else:
                    print("Json data format is wrong!")
                    continue
            self.VisSensor = self.VisSensor + [visSenStruct]
            visSenStructTemp.bitmask = 0        # SensorType == 9
            visSenStructTemp.CameraFOV = visSenStruct.CameraFOV
            visSenStructTemp.checksum = visSenStruct.checksum
            visSenStructTemp.DataCheckFreq = visSenStruct.DataCheckFreq
            visSenStructTemp.DataHeight = visSenStruct.DataHeight
            visSenStructTemp.DataWidth = visSenStruct.DataWidth
            visSenStructTemp.EularOrQuat = visSenStruct.EularOrQuat
            visSenStructTemp.otherParams = visSenStruct.otherParams
            visSenStructTemp.SendProtocol = visSenStruct.SendProtocol
            visSenStructTemp.SensorAngEular = visSenStruct.SensorAngEular
            visSenStructTemp.SensorAngQuat  = visSenStruct.SensorAngQuat
            visSenStructTemp.SensorPosXYZ = visSenStruct.SensorPosXYZ
            visSenStructTemp.SeqID = visSenStruct.SeqID
            visSenStructTemp.TargetCopter = visSenStruct.TargetCopter
            visSenStructTemp.TargetMountType = visSenStruct.TargetMountType
            visSenStructTemp.TypeID = visSenStruct.TypeID
            self.VisSensorNew = self.VisSensorNew + [visSenStructTemp]

            if ~self.isNewJson and isNewProt:
                self.isNewJson = True

        if (len(self.VisSensor)) <= 0:
            print("No sensor is obtained.")
            return False
        print("Got", len(self.VisSensor), "vision sensors from json")

        if len(self.RflyTimeVect) == 0 and ~self.tTimeStmpFlag:
            # print('Start listening CopterSim time Data')
            self.StartTimeStmplisten()
            time.sleep(2)
            self.endTimeStmplisten()
        if len(self.RflyTimeVect) > 0:
            print("Got CopterSim time Data for img")
        else:
            print("No CopterSim time Data for img")

        return True

    
    def stopRun(self):
        self.stopFlag=True
        if self.t_menRec:
            self.t_menRec.join()