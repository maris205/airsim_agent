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
import cv2
import numpy as np
## @file
#  
#  @anchor NetSimAPIV4接口库文件

# UAVSendData
# double uavTimeStmp 仿真时间戳（单位秒）
# double timeDelay 这个消息包的延迟，
# 注意：当前电脑时间戳-收到包的时间戳等于延迟，因此发包和发包电脑需要是同一台
# 注意： 如果收发包不是同一台点，那么需要两台电脑做时钟同步
# float uavAngEular[3] 欧拉角 弧度
# float uavVelNED[3] 速度 米
# double uavPosGPSHome[3] GPS 维度（度）、经度（度）、高度（米）
# double uavPosNED[3] 本地位置 米 （相对起飞点）
# double uavGlobalPos[3] 全局位置 （相对与所有飞机的地图中心）
# d6f9d = 长度104
class UAVSendData:
    # 无人机数据类，从mav中拿到无人机的数据，并转发到网络，给别的飞机
    # 包含飞机的位置、速度、姿态角等
    def __init__(self):
        self.hasUpdate=False
        self.timeDelay=0
        self.CopterID=0
        self.uavTimeStmp=0
        self.uavAngEular=[0,0,0]
        self.uavVelNED=[0,0,0]
        self.uavPosGPSHome=[0,0,0]
        self.uavPosNED=[0,0,0]
        self.uavGlobalPos=[0,0,0]

# PX4 MAVLink listen and control API and RflySim3D control API
class NetSimAPI:

    # 无人机通信接口API，MavOrCopterID可以指定mav=PX4MavCtrlV4或CopterID号
    # 如果传入mav，则默认启用无人机数据转发模式，会将UAVSendData转发到指定端口（由enNetForward或enUavForward指定）
    
    # 本接口有四种用法
    # 一、单点通信模拟。
    # 用enNetForward或enUavForward指定数据包发到指定的飞机（端口）列表，然后StartNetRec指定接收自己飞机（端口）数据
    # 例如, 1号飞机，指定将数据转发给2 3 4 5，并指定接收发往1号飞机数据
    #       ......
    #       5号飞机，指定将数据发往1 2 3 4，并指定接收5号飞机数据
    # 通过上述配置，可以指定组建一种指定飞机ID的通信方式（对应单播）。
    
    # 二、广播(组播)模拟
    # 用enNetForward或enUavForward指定数据包发往默认端口60000（或0号飞机），然后StartNetRec绑定并接收60000（或0号飞机）数据
    # 通过这种方式，相当于构建了一个广播（组播）网络，每个飞机都能收到其他飞机的数据，通过数据包内的CopterID，可以识别出数据来自哪个飞机
    
    # 三、广播模拟+飞机筛选
    # 在“二、广播模拟”的基础上，
    # 1. 可以通过netResetSendList或netAddUavSendList来维护一个飞机列表，使得数据包只会被指定的飞机响应
    # 2. 可以通过StartReqUavData函数，来请求别的飞机，将自己加入飞机列表，以便能收到指定飞机的数据
    # 通过上述列表控制功能，可以实现网络数据的筛选
    
    # 四、连接网络仿真器
    # 在“一、单点通信模拟”的基础上，还可以通过enNetForward或enUavForward将数据包统一发给网络仿真器的端口（例如，65000）
    # 其次，每个飞机订阅自己的数据接口（例如，60000+CopterID）
    # 网络仿真器，对于每个飞机本应该收到的数据，会经过网络仿真模拟，加入延迟、丢包等环节，再将数据包转发到各个飞机
    # 例如，1号飞机本身要将数据发给3号飞机，但是它们之间被高墙遮挡，网络仿真器将会把1号飞机发给3号飞机的数据包拦截
    # 或者，3号飞机被对方网络干扰器锁定，则网络仿真器将不会转发任何数据包给3号飞机
    
    # 本接口能够转发和接受的数据类型主要有两种：
    # 1. 来自mav的底层飞控数据，包括位置点等（需要将mav传入NetSimAPI构造函数），数据包结构见UAVSendData
    # 2. 使用netForwardBuf直接将某个buf包转发出去，对方需要用同样的方法进行解包
    # 注意：后一种方法，需要在外层python中，写一个死循环，并调用bufEvent.wait()来接收buf包来的信号
    #       然后，从bufData中，获取buf包的值，并进行数据解析。
    #       注意，还可以从bufHead=[checkSum,CopterID,sendMode,StartIdx,SendMask,TimeUnix]中获取数据包源头信息

    def __init__(self, MavOrCopterID=1):
        # mav如果赋值为PX4MavCtrlV4的实例，则self.SendState为True，会自动转发无人机的位置、速度等状态信息
        # mav如果赋值为CopterID，则不会转发飞机自身信息，而是需要用netForwardBuf来发送buf到其他飞机
        
        self.mav = MavOrCopterID
        
        if isinstance(MavOrCopterID,int):
            self.CopterID = MavOrCopterID # 直接指定飞机序号
            self.SendState=False # 不转发本无人机数据，只转发buf
        else:
            self.CopterID = self.mav.CopterID # 取上层mav中的CopterID
            self.SendState = True # 转发本机内层mav的无人机数据

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 

        # 网络仿真时，数据转发专用端口
        self.ForwardIP='127.0.0.1'
        self.ForwardPort=60000 # 默认发给0号节点，也就是假设是地面中心
        self.enForward=False
        
        # 网络仿真时，数据转发到飞机的标志位
        self.netSendIDList=[] # 发送飞机总的列表
        self.netSendStarList=[] # 64个飞机为一组，起始飞机的序号
        self.netSendMaskList=[] # 64个飞机的发送标志为，1表示发送，0表示不发送
        self.netSendMode=0 # 网络通信模式

        # 网络仿真时，请求消息的飞机ID 
        self.netReqIDList=[]    # 请求飞机总的列表
        self.netReqStarList=[]  # 64个飞机为一组，起始飞机的序号
        self.netReqMaskList=[]  # 64个飞机的发送标志为，1表示发送，0表示不发送

        # 无人机数据向量与字典，如果飞机已在列表中，会更新，如果不在则自动扩充列表
        self.UavData = [] # 无人机数据结构体向量，收到的所有飞机的数据会更新存在本列表
        # 注：self.UavData[0], self.UavData[1]，表示最先收到的飞机的数据包，不一定和CopterID匹配
        self.UavDict = {} # 无人机数据结构体字典，收到的所有飞机的数据会更新存在本字典中
        # self.UavDict['1'] 表示CopterID为1的飞机的数据指针，便于快速获取指定飞机数据
        
        # 用于网络通信的事件信号
        self.bufEvent = threading.Event() # 线程事件，用于通知外层Python程序有数据更新了
        self.bufData=[] # 存储当前的数据buf
        self.bufHead=[] # 存储当前的包头信息
        # bufHead=[checkSum,CopterID,sendMode,StartIdx,SendMask,TimeUnix]
        #checkSum=12345678 # int32型，校验位
        #CopterID, int32型，当前飞机的ID号
        #sendMode,  int32型，发送模式，有组播，也有单拨
        #StartIdx,  int32型，发送列表的起始飞机序号
        #SendMask,  uinit64型，从序号开始的64个飞机是否发送
        #TimeUnix, double型，发送前获取到的电脑时钟        
        
    # enNetForward的简化版，只需要指令期望发送的飞机列表，会自动用224.0.0.10 + 60000系列端口转发
    # uavList=[0 1 2 3等]，表示想要将数据或buf转发给哪些飞机
    # uavList=0对应60000端口，是默认的所有飞机共同的通信端口（也就是能收发所有飞机的数据）
    # Interval表示每隔数据点，向外发一次包。如果Interval=0表示，来包就转发；
    # 如果Interval=5，表示每5个数据向外发一次。这个接口可以降低发包速率，调节通信数量。
    def enUavForward(self,CopterIDList=[0],Interval=0): # 默认只发给0号飞机（对应60000端口，发给所有飞机）
        if not isinstance(CopterIDList,list): # 如果是一维标量，就转为数组
            CopterIDList = [CopterIDList]
        PortList=[]
        for i in range(len(CopterIDList)):
            PortList = PortList + [CopterIDList[i]+60000] # 自动填充默认端口规则
        self.enNetForward(PortList,'224.0.0.10',Interval)
        
    # 启用网络转发，将本机收到的消息，转发给targetIP上的PortList系列端口
    # 默认发给0号节点，也就是假设是地面中心，以及组播IP 224.0.0.10
    # Interval表示每隔数据点，向外发一次包。如果Interval=0表示，来包就转发；
    # 如果Interval=5，表示每5个数据向外发一次。这个接口可以降低发包速率，调节通信数量。
    def enNetForward(self,PortList=[60000],targetIP='224.0.0.10',Interval=0):
        if self.enForward:
            return
        self.ForwardIP=targetIP
        if not isinstance(PortList,list):# 如果是一维标量，就转为数组
            PortList=[PortList]
        self.ForwardPort=PortList
        self.enForward=True
        self.Interval=Interval
        if Interval<0:
            self.Interval=0
        self.packCount=0
        self.packTarget=0
        self.t1 = threading.Thread(target=self.getMavEvent, args=())
        self.t1.start()

    
    # 中止通信转发的函数，在结束仿真时调用
    def endNetForward(self):
        self.enForward=False
        self.t1.join()

    # 清空发送飞机的列表
    def netResetSendList(self): # 清空待发送列表
        self.netSendIDList=[]
        self.netSendStarList=[]
        self.netSendMaskList=[]
        #self.netSendMode=sendMode

    # 添加发送飞机列表，可多次调用，最后维持一个总表
    def netAddUavSendList(self,uavList=[]):

        for i in range(len(uavList)): # 遍历待发送飞机列表
            CurID = uavList[i]
            isIdExist=False
            for j in range(len(self.netSendIDList)): # 判断是否已经在列表中
                if CurID== self.netSendIDList[j]:
                    isIdExist=True
                    break
            if not isIdExist: # 不在列表中就加入列表
                self.netSendIDList = self.netSendIDList + [CurID]

        # 开始更新目标发送列表

        self.netSendIDList.sort() # 从小到大重新排序
        
        netGrp=[]
        self.netSendStarList=[]
        self.netSendMaskList=[]        
        for i in range(len(self.netSendIDList)):
            curID = self.netSendIDList[i]
            if len(netGrp)==0:
                netGrp=[curID]
            else:
                if curID-netGrp[0]<64: # 如果还没有越界，就累加
                    netGrp=netGrp+[curID]
            
            # 到达
            if curID-netGrp[0]>=64 or i==len(self.netSendIDList)-1:
                # 开始将上一组数据，写到列表中
                startIdx = netGrp[0]
                mask = 0x0000000000000000 # 64位的mask
                for j in range(len(netGrp)):
                    dist = netGrp[j]-startIdx
                    mask = mask | 1<<dist
                self.netSendStarList = self.netSendStarList + [startIdx]
                self.netSendMaskList = self.netSendMaskList + [mask]

                # 如果超过64位了，则直接从新开启一组
                if curID-netGrp[0]>=64 and i<len(self.netSendIDList)-1:
                    netGrp=[curID]

                # 如果超过64位且刚好达到末尾，则直接计算矩阵
                if curID-netGrp[0]>=64 and i==len(self.netSendIDList)-1:
                    startIdx = curID
                    mask = 1
                    self.netSendStarList = self.netSendStarList + [startIdx]
                    self.netSendMaskList = self.netSendMaskList + [mask]

    # 清空请求发数据给本飞机的列表
    def netResetReqList(self): # 清空待发送列表
        self.netReqIDList=[]
        self.netReqStarList=[]
        self.netReqMaskList=[]

    # 扩充请求发数据给本飞机的列表，可多次调用，最终维护一个总表
    def netAddUavReqList(self,uavList=[]):

        for i in range(len(uavList)): # 遍历待发送飞机列表
            CurID = uavList[i]
            isIdExist=False
            for j in range(len(self.netReqIDList)): # 判断是否已经在列表中
                if CurID== self.netReqIDList[j]:
                    isIdExist=True
                    break
            if not isIdExist: # 不在列表中就加入列表
                self.netReqIDList = self.netReqIDList + [CurID]

        # 开始更新目标发送列表
        self.netReqIDList.sort() # 从小到大重新排序
        
        netGrp=[]
        self.netReqStarList=[]
        self.netReqMaskList=[]        
        for i in range(len(self.netReqIDList)):
            curID = self.netReqIDList[i]
            if len(netGrp)==0:
                netGrp=[curID]
            else:
                if curID-netGrp[0]<64: # 如果还没有越界，就累加
                    netGrp=netGrp+[curID]
            
            # 到达
            if curID-netGrp[0]>=64 or i==len(self.netReqIDList)-1:
                # 开始将上一组数据，写到列表中
                startIdx = netGrp[0]
                mask = 0x0000000000000000 # 64位的mask
                for j in range(len(netGrp)):
                    dist = netGrp[j]-startIdx
                    mask = mask | 1<<dist
                self.netReqStarList = self.netReqStarList + [startIdx]
                self.netReqMaskList = self.netReqMaskList + [mask]

                # 如果超过64位了，则直接从新开启一组
                if curID-netGrp[0]>=64 and i<len(self.netReqIDList)-1:
                    netGrp=[curID]

                # 如果超过64位且刚好达到末尾，则直接计算矩阵
                if curID-netGrp[0]>=64 and i==len(self.netReqIDList)-1:
                    startIdx = curID
                    mask = 1
                    self.netReqStarList = self.netReqStarList + [startIdx]
                    self.netReqMaskList = self.netReqMaskList + [mask]

    # 开始以1Hz频率广播数据给请求飞机，后者收到消息后，将会将自己数据传过来
    def StartReqUavData(self,uavList=[]):
        self.netAddUavReqList(uavList)
        self.stopFlagReq=False
        self.tReq = threading.Thread(target=self.sendReqUavLoop, args=())
        self.tReq.start()

    # 结束请求飞机数据循环
    def EndReqUavData(self):
        self.stopFlagReq=True
        self.tReq.join()

    # 发送本飞机数据的循环线程
    def sendReqUavLoop(self):
        while True:
            if self.stopFlagReq:
                break
            
            checkSum=12345678 # int32型，校验位
            CopterID = self.CopterID # int32型，当前飞机的ID号
            sendMode = 12345 # int32型，发送模式，有组播，也有单拨 # 这里12345表示是一个请求消息包

            ReqStarList = self.netReqStarList
            ReqMaskList = self.netReqMaskList
            # 如果没有指定发送列表，则填0发出
            if len(ReqStarList)==0:
                ReqStarList=[0]
                ReqMaskList=[0]

            for j in range (len(ReqStarList)):
    
                StartIdx = ReqStarList[j] # int32型，发送列表的起始飞机序号
                SendMask = ReqMaskList[j] # uinit64型，从序号开始的64个飞机是否发送
                TimeUnix = time.time_ns()/1e9 # 使用time_ns能获取比time更高的精度
                #checkSum=12345678 # int32型，校验位
                #CopterID, int32型，当前飞机的ID号
                #sendMode,  int32型，发送模式，有组播，也有单拨
                #StartIdx,  int32型，发送列表的起始飞机序号
                #SendMask,  uinit64型，从序号开始的64个飞机是否发送
                #TimeUnix, double型，发送前获取到的电脑时钟
                # 编码规则 iiiiQd
                # 注意：sendMode、StartIdx、和SendMask是组网仿真程序需要使用的，接收端用不着可忽略
                # 注意：TimeUnix主要用于统计当前的通信延迟
                bufID = struct.pack('iiiiQd',checkSum,CopterID,sendMode,StartIdx,SendMask,TimeUnix) # 封装校验位和飞机ID
                myBuf = bufID #这里只发一个表头

                for i in range(len(self.ForwardPort)): # 向端口列表转发数据
                    self.udp_socket.sendto(myBuf,(self.ForwardIP,self.ForwardPort[i])) # 将数据转发给网络仿真器
                    #print('Send')

            # 每休息一秒发送一次请求
            time.sleep(1)

    # 发出一个buf包，给到指定飞机或端口
    def netForwardBuf(self,buf):
        # 将数据发送到ForwardIP和ForwardPort（端口或列表）
        if self.enForward: #如果开启网络转发
            checkSum=12345678 # int32型，校验位
            CopterID = self.CopterID # int32型，当前飞机的ID号
            sendMode = self.netSendMode # int32型，发送模式，有组播，也有单拨

            SendStarList = self.netSendStarList
            SendMaskList = self.netSendMaskList
            # 如果没有指定发送列表，则填0发出
            if len(SendStarList)==0:
                SendStarList=[0]
                SendMaskList=[0]

            for j in range (len(SendStarList)):
    
                StartIdx = SendStarList[j] # int32型，发送列表的起始飞机序号
                SendMask = SendMaskList[j] # uinit64型，从序号开始的64个飞机是否发送
                TimeUnix = time.time_ns()/1e9 # 使用time_ns能获取比time更高的精度
                #checkSum=12345678 # int32型，校验位
                #CopterID, int32型，当前飞机的ID号
                #sendMode,  int32型，发送模式，有组播，也有单拨
                #StartIdx,  int32型，发送列表的起始飞机序号
                #SendMask,  uinit64型，从序号开始的64个飞机是否发送
                #TimeUnix, double型，发送前获取到的电脑时钟
                # 编码规则 iiiiQd
                # 注意：sendMode、StartIdx、和SendMask是组网仿真程序需要使用的，接收端用不着可忽略
                # 注意：TimeUnix主要用于统计当前的通信延迟
                bufID = struct.pack('iiiiQd',checkSum,CopterID,sendMode,StartIdx,SendMask,TimeUnix) # 封装校验位和飞机ID
                myBuf = bufID+buf #包头封装一个飞机ID号

                for i in range(len(self.ForwardPort)): # 向端口列表转发数据
                    self.udp_socket.sendto(myBuf,(self.ForwardIP,self.ForwardPort[i])) # 将数据转发给网络仿真器
                    #print('Send')

    # 监听底层mav的数据更新事件，并将数据转发出去
    def getMavEvent(self):

        while True:
            # 如果不启用数据转发模式，则直接退出
            if not self.enForward:
                break
            
            # 如果需要转发本机数据，也直接退出
            if not self.SendState:
                break
            
            # 如果mav收到了消息
            self.mav.netEvent.wait()
            #print(time.time())
                       
            self.packCount=self.packCount+1
            # 到达期望次数才发包，降低发送频率
            if self.packCount>self.packTarget:
                # double uavTimeStmp 时间戳
                # float uavAngEular[3] 欧拉角 弧度
                # float uavVelNED[3] 速度 米
                # double uavPosGPSHome[3] GPS 维度（度）、经度（度）、高度（米）
                # double uavPosNED[3] 本地位置 米 （相对起飞点）
                # double uavGlobalPos[3] 全局位置 （相对与所有飞机的地图中心）
                # d6f9d = 长度104
                if self.enForward: #如果开启网络转发
                    # 提取mav的必要数据，向外转发
                    buf=struct.pack('d6f9d',self.mav.uavTimeStmp,*self.mav.uavAngEular,*self.mav.uavVelNED,*self.mav.uavPosGPSHome,*self.mav.uavPosNED,*self.mav.uavGlobalPos)
                    self.netForwardBuf(buf)
                    #print('Send Data')
                self.packTarget = self.packCount+self.Interval
            
            self.mav.netEvent.clear()
    
    # 开始监听发往自己飞机的数据，端口根据CopterID自动设置
    def StartNetRecOwn(self):
        self.StartNetRec(60000+self.CopterID)
    
    # 开启接收发往指定飞机的数据
    # 默认接收1号飞机端口60001，和组播IP 224.0.0.10
    def StartNetRec(self,MultiPort=60000,MultiIP='224.0.0.10'):
        # 设置组播监听IP和地址，并开启监听
        ANY = '0.0.0.0'
        self.udp_socketNet = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建UDP实例
        self.udp_socketNet.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # 设置广播支持
        self.udp_socketNet.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) # 设置端口复用
        self.udp_socketNet.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 60000 * 100) # 设置数据缓存
        self.udp_socketNet.bind((ANY, MultiPort)) # 绑定端口
        try:
            # 加入组播，注需要联网才支持
            status = self.udp_socketNet.setsockopt(socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                socket.inet_aton(MultiIP) + socket.inet_aton(ANY))
        except:
            print('Failed to Init multicast!')
            
        self.stopFlagNet=False
        self.t1Net = threading.Thread(target=self.getMavMsgNet, args=())
        self.t1Net.start()
        
    # 退出网络数据监听线程循环
    def endNetLoop(self):

        self.stopFlagNet=True
        self.t1Net.join()
        self.udp_socketNet.close()

    # 监听网络发往本机数据的线程循环函数
    def getMavMsgNet(self):


        while True:
            if self.stopFlagNet:
                break
            
            # try:
            buf,addr = self.udp_socketNet.recvfrom(65500)
            #print('Data Rec')
            # 如果数据包太小，说明不正确，直接跳过
            if len(buf)<32:
                continue

            if len(buf)==32:# 只有包头，说明是请求消息
                checkSum,CopterID,sendMode,StartIdx,SendMask,TimeUnix=struct.unpack('iiiiQd',buf[0:32]) # 获取校验位和通知
                if sendMode==12345: # 如果是请求模式
                    hasMatch=False
                    for i in range(64):
                        if SendMask & 1<<i: # 如果这一位使能了
                            TarCopID = StartIdx + i # 提取使能的飞机ID（回传列表）
                            if TarCopID==self.CopterID: # 如果本飞机的ID在回传列表中
                                self.netAddUavSendList([CopterID]) # 将目标飞机的ID记录在发送列表中，这样就能将消息发给他了
                                break

                continue
            
            #checkSum=12345678 # int32型，校验位
            #CopterID, int32型，当前飞机的ID号
            #sendMode,  int32型，发送模式，有组播，也有单拨
            #StartIdx,  int32型，发送列表的起始飞机序号
            #SendMask,  uinit64型，从序号开始的64个飞机是否发送
            #TimeUnix, double型，发送前获取到的电脑时钟
            # 编码规则 iiiiQd
            # 注意：sendMode、StartIdx、和SendMask是组网仿真程序需要使用的，接收端用不着可忽略
            # 注意：TimeUnix主要用于统计当前的通信延迟
            checkSum,CopterID,sendMode,StartIdx,SendMask,TimeUnix=struct.unpack('iiiiQd',buf[0:32]) # 获取校验位和通知
            timeDelay = time.time_ns()/1e9 - TimeUnix # 当前时间戳减去发包时间戳，为传输延迟
            if checkSum==12345678: # 如果校验通过,符合预设的密码（这里是为了排除无干的消息）
                
                bufHead=[checkSum,CopterID,sendMode,StartIdx,SendMask,TimeUnix]
                
                buf = buf[32:]
                # UAVSendData
                # double uavTimeStmp 仿真时间戳，单位秒
                # double timeDelay 这个消息包的延迟，
                # 注意：当前电脑时间戳-收到包的时间戳等于延迟，因此发包和发包电脑需要是同一台
                # 注意： 如果收发包不是同一台点，那么需要两台电脑做时钟同步
                # float uavAngEular[3] 欧拉角 弧度
                # float uavVelNED[3] 速度 米
                # double uavPosGPSHome[3] GPS 维度（度）、经度（度）、高度（米）
                # double uavPosNED[3] 本地位置 米 （相对起飞点）
                # double uavGlobalPos[3] 全局位置 （相对与所有飞机的地图中心）
                # d6f9d = 长度104
                
                if len(buf)==104:
                    UIV=struct.unpack('d6f9d',buf)
                    uavTimeStmp=UIV[0]
                    uavAngEular=UIV[1:4]
                    uavVelNED=UIV[4:7]
                    uavPosGPSHome=UIV[7:10]
                    uavPosNED=UIV[10:13]
                    uavGlobalPos=UIV[13:16]
                    
                    isCopterExist=False
                    for i in range(len(self.UavData)): #遍历数据列表，飞机ID有没有出现过
                        if self.UavData[i].CopterID == CopterID: #如果出现过，就直接更新数据
                            idx=i
                            isCopterExist=True
                            self.UavData[idx].hasUpdate=True
                            self.UavData[idx].timeDelay = timeDelay
                            self.UavData[idx].uavTimeStmp=uavTimeStmp
                            self.UavData[idx].uavAngEular=uavAngEular
                            self.UavData[idx].uavVelNED = uavVelNED
                            self.UavData[idx].uavPosGPSHome = uavPosGPSHome
                            self.UavData[idx].uavPosNED = uavPosNED
                            self.UavData[idx].uavGlobalPos = uavGlobalPos
                            
                            #self.UavDict[str(idx)] = UavData
                            break
                    if not isCopterExist:#如果没有出现过，就创建一个结构体
                        vsr=UAVSendData()
                        vsr.hasUpdate=True
                        vsr.CopterID = CopterID
                        vsr.uavTimeStmp=uavTimeStmp
                        vsr.timeDelay = timeDelay
                        vsr.uavAngEular=uavAngEular
                        vsr.uavVelNED = uavVelNED
                        vsr.uavPosGPSHome = uavPosGPSHome
                        vsr.uavPosNED = uavPosNED
                        vsr.uavGlobalPos = uavGlobalPos  
                        vsrNew=copy.deepcopy(vsr)
                        self.UavData = self.UavData +  [vsrNew] #扩充列表，增加一个元素
                        self.UavDict[str(CopterID)] = vsrNew
        
                # 如果数据包被读取，则重新赋值
                if not self.bufEvent.isSet():
                    # 赋值数据包，并
                    self.bufData=buf
                    self.bufEvent.set()
                    self.bufHead = bufHead
                    # except:
                    #     self.stopFlagNet=True
                    #     break
    
    # 从所有飞机状态数据库中，读取指定飞机的数据指针
    def getUavData(self,CopterID):
        for uav in self.UavData:
            if uav.CopterID == CopterID:
                return uav
        return UAVSendData()
            
