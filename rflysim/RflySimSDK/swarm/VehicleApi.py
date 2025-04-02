import socket
import threading
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import struct
from math import *
import math
import sys
import copy
import os
import cv2
import numpy as np
## @file
#  
#  @anchor VehicleApi接口库文件


#import win_precise_time as time


'''
self.CurFlag == 0:  Offboard模式
self.CurFlag == 10: 起飞滑跑阶段
self.CurFlag == 11: 起飞爬升阶段
self.CurFlag == 12: 起飞平飞阶段
self.CurFlag == 121: 起飞盘旋阶段


ctrlMode  < 0:  设置成<0(小于0),表示是空命令，模块不会向外发布消息。
ctrlMode == 2:  sendPosNED命令模式
ctrlMode == 10: 设置飞机的速度和和盘旋半径
ctrlMode == 11: Mavlink的本地起飞命令
ctrlMode == 12: Mavlink的全球起飞命令
ctrlMode == 13: SendVelYawAlt速度航线和高度命令模式

'''
## @class Vehicle
#  @brief 本类用于创建并控制飞行器对象（Vehicle）。包含对飞行器类型、位置、姿态、速度、飞行模式以及地图场景等的初始化和更新逻辑。
class Vehicle:
    ## @brief 构造函数，初始化飞行器的ID、类型、地图场景、更新频率以及其他控制参数。
    #  - @anchor __init__
    #  @param CopterID(int): 飞机ID，默认为1。
    #  @param Vehicletype(int): 飞机类型编号，默认为3。
    #  @param mapName(str): 地图名称，默认为'Grasslands'。
    #  @param updatefreq(int): 往RflySim3D发送消息的频率，同时也是模型更新频率，默认为100。当updatefreq=0时，需外部调用step函数。
    #  @param isBroadCast(bool): 是否将飞机三维数据广播到局域网其他电脑，默认为False。    
    def __init__(self,CopterID = 1,Vehicletype = 3,mapName='Grasslands', updatefreq = 100, isBroadCast=False):
        # updatefreq 是往RflySim3D发送消息的频率，同时也是模型的默认更新频率
        # 如果updatefreq=0，则禁用本模块的step函数的while循环，需要从外部调用step
        # isBroadCast 表示飞机的三维数据是否广播到局域网其他电脑
        
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)        

        self.CopterID = CopterID # 飞机ID
        self.Vehicletype = Vehicletype # 飞机样式
        self.updatefreq = updatefreq # 更新频率
        self.isBroadCast = isBroadCast # 是否广播，也就是飞机运动数据，是否发送到其他电脑
        
        self.map = UEMapServe(mapName)
        strCmd = 'RflyChangeMapbyName '+mapName
        self.sendUE4Cmd(strCmd.encode())
        
        self.intStateX= 0 # 初始位置x
        self.intStateY= 0 # 初始位置Y
        self.intAlt = 0 # 初始高度
        self.intStateYaw = 0 # 初始偏航角
        
        self.isInPointMode=False

        self.uavAngEular = [0, 0, 0]  # Estimated Eular angles from PX4
        self.trueAngEular = [0, 0, 0] # True simulated Eular angles from CopterSim's DLL model
        self.uavAngRate = [0, 0, 0]  # Estimated angular rate from PX4
        self.trueAngRate = [0, 0, 0] # True simulated angular rate from CopterSim's DLL model
        self.uavPosNED = [0, 0, 0] # Estimated Local Pos (related to takeoff position) from PX4 in NED frame
        self.truePosNED = [0, 0, 0] # True simulated position (related to UE4 map center) from CopterSim's DLL model
        self.uavVelNED = [0, 0, 0] # Estimated local velocity from PX4 in NED frame
        self.trueVelNED = [0, 0, 0] # True simulated speed from CopterSim's DLL model  in NED frame
        self.uavPosGPS = [0, 0, 0,0, 0, 0,0,0,0] # Estimated GPS position from PX4 in NED frame,lat lon alt relative_alt vx vy vz hdg
        self.uavGlobalPos = [0, 0, 0] # Estimated global position from PX4 that transferred to UE4 map

        self.offMode=0 # Offboard的具体模式
        self.EnList = [0,1,0,0,0,1]
        self.type_mask = self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yaw=0
        self.yawrate = 0

        self.B = [5, 10, 5, 12, 1]

        # 期望控制信号
        self.velOff=list(self.vel)
        self.yawOff=self.yaw
        self.yawRateOff=self.yawrate
        self.MotorRPMS=[0,0,0,0,0,0,0,0]  
        self.velair=0 

        self.MaxSpeed=20
        
        self.FwSpeedTrim = 10 # 固定翼巡航速度，发航点时，以此速度前进
        self.FwLoiterRad = 10 # 固定翼盘旋半径
        
        self.isArmed = False
        
        
        self.CurFlag = 0 # 当前模式
        self.lastFlag = 0 # 上一模式
        
        # Flag = 0 -> Offboard模式，需要输入的参数如下
        # 需要的参数较多，在上边
        
        # Flag = 1 -> 固定翼Offboard模式（速度、高度和偏航），需要输入的参数如下
        
        
        # Flag = 10 -> 滑跑模式，需要输入的参数如下
        self.TkOffYaw = 0 # 滑跑的偏航方向
        self.TkOffAccSpeed = 1 # 滑跑的加速度
        self.TkOffMaxSpeed = 10 # 滑跑的目标速度（达到目标速度后起飞）
        self.TkOffNextFlag = 11 # 达到目标速度后，切换到的模式，这里11模式表示固定斜率爬升
        
        # Flag = 11 -> 爬升模式，需要输入的参数如下
        self.ClimbSlope = 10/180.0*math.pi # 爬升的斜率
        self.ClimbStartSpeed = 10 # 爬升的加速度（速度增加）
        self.ClimbAccSpeed = 1 # 爬升的加速度（速度增加）
        self.ClimbMaxSpeed = 20 # 爬升的目标速度（达到目标速度后起飞）
        self.ClimbMaxAlt = -50 # 爬升的最大高度
        self.ClimbNextFlag = 12 # 到达爬升高度后的下一个模式，12是平飞模式
        
        # Flag = 12 -> 平飞模式，需要输入的参数如下
        self.ForwardSpeed  = 20 # 平飞速度
        self.ForwardPosXYZ = [180,0,-50] # 下一个航路点的位置
        self.ForwardNextFlag = 13 # 平飞模式的下一个模式，13是盘旋模式
        
        # Flag = 13 -> 盘旋模式，需要输入的参数如下
        self.CircleSpeed = 20 # 盘旋的速度
        self.CircleRadius = 20 # 盘旋的半径
        self.CirclePosXYZ = [200,0,-50] # 盘旋的中心坐标
        self.CircleDir = 1 # 盘旋的方向，这里1是顺时针
        
        self.VehiclePitch = 0
        self.VehicleYaw = 0
        
        self.GPSOrigin=[40.1540302,116.2593683,50]
        self.geo = EarthModel()
        self.lastTimeStep = -1
        
        self.lastCtrl=[0,0,0,0]

    ## @brief 对输入的偏航角进行角度约束，将偏航角限制在[-pi, pi]区间内。
    #  - @anchor yawSat
    #  @param yaw(float): 待约束的偏航角度（单位：弧度）
    #  @return 返回约束后的偏航角度，范围在[-pi, pi]之间    
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

    ## @brief 对偏航角进行幅度限制，将偏航角限制在[-pi, pi]附近的范围内。
    #  - @anchor yawSat1
    #  @param yaw(float): 待处理的偏航角度（弧度值）
    #  @return 返回处理后的偏航角度    
    def yawSat1(self,yaw):
        """ satuate the yaw angle from -pi to pi
        """
        if yaw>3:
            yaw = yaw-math.pi*2
            yaw=self.yawSat(yaw)
        elif yaw <-3:
            yaw = yaw+math.pi*2
            yaw=self.yawSat(yaw)
        return yaw

    ## @brief 对滚转角进行幅度限制，将滚转角限制在[-pi/6, pi/6]范围内。
    #  - @anchor RollSat
    #  @param Roll(float): 待处理的滚转角度（弧度值）
    #  @return 返回处理后的滚转角度
    def RollSat(self,Roll):
        """ satuate the yaw angle from -pi to pi
        """
        if Roll>math.pi/6:
            Roll = Roll-math.pi/6
            Roll=self.RollSat(Roll)
        elif Roll <-math.pi/6:
            Roll = Roll+math.pi/6
            Roll=self.RollSat(Roll)
        return Roll
    
    ## @brief 对输入的PWM值进行饱和限制，限制在[-thres, thres]范围内。
    #  - @anchor sat
    #  @param inPwm(float): 输入的PWM值
    #  @param thres(float): 饱和限制阈值，默认值为1
    #  @return 返回限制后的PWM值    
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

    ## @brief 初始化简化模型（质点模型）的运行环境，包括起始状态、目标IP地址和GPS原点设定。
    #  启动模型更新循环与Mavlink数据监听线程。
    #  - @anchor initSimpleModel
    #  @param intState(list): 初始状态 [PosX(m), PosY(m), Yaw(deg)]
    #  @param targetIP(str): 发送数据的目标IP地址
    #  @param GPSOrigin(list): GPS原点坐标 [lat, lon, alt]    
    def initSimpleModel(self,intState=[0,0,0],targetIP = '127.0.0.1',GPSOrigin=[40.1540302,116.2593683,50]):
        """ Init and start the point mass model for UAV control
        intAlt (unit m) is the init height of the vehicle on the UE4 map, which can be obtained from the CopterSim or UE4
        intState contains the PosX (m), PosY (m) and Yaw (degree) of the vehicle, which denotes the initial state of the vehicle
        it is the same as the value on CopterSim UI.
        targetIP: is the target IP address (Simulink rec PC) to send vehicle info
        """
        
        
        self.intStateX=intState[0] # Init PosX of CopterSim
        self.intStateY=intState[1] # Init PosY of CopterSim
        self.intStateYaw=intState[2] # Init Yaw angle of CopterSim
        
        self.GPSOrigin = GPSOrigin
        
        self.intAlt=self.map.getTerrainAltData(self.intStateX,self.intStateY) # Init altitude from CopterSim ground height of current map
        print(self.intAlt)
        
        # 开启模型更新循环
        if self.updatefreq>=1: # 如果启用Vehicle内部While循环更新
            self.isInPointMode=True
            self.t3 = threading.Thread(target=self.SimpleModelLoop, args=())
            self.t3.start()
            self.lastTimeStep = time.time()
        
        # Mavlink 收发接口
        self.udpCtrlRec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udpCtrlRec.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.RecIPPort = ('0.0.0.0', 20100+(self.CopterID-1)*2) # 接收接口20100系列
        self.udpCtrlRec.bind(self.RecIPPort)  # 绑定监听端口
        self.SendIpPort = (targetIP,20100+(self.CopterID-1)*2+1) # 填充发送接口
        
        # 开启数据监听循环
        self.t4Flag=True
        self.t4=threading.Thread(target=self.RecMavLoop, args=())
        #print("Listening。。。")
        self.t4.start()
        
        
    ## @brief 接收并处理来自Mavlink的控制数据循环线程。
    #  - @anchor RecMavLoop
    #  当t4Flag被设为False时退出循环。
    #  根据接收到的控制模式(ctrlMode)和数据(controls)，执行相应的动作，如改变飞行模式、速度、高度、航向等。        
    def RecMavLoop(self):
        
        isInTakeoffMode=False
        
        while True:
            if not self.t4Flag: # 如果t4Flag显示停止，则退出循环
                break
            
            buf,addr = self.udpCtrlRec.recvfrom(65500)
            ListenDate=struct.unpack('iiffff',buf[0:24])
            #print("mat",self.ListenDate)
            # struct inOffboardShortData{
            #     int checksum; # 校验位，需要确认等于1234567890
            #     int ctrlMode;
            #     float controls[4];
            # }
            checksum = ListenDate[0]
            if checksum == 1234567890: # 数据校验通过
                ctrlMode = ListenDate[1] # 控制模式标志位
                controls = ListenDate[2:6] # 2到5号数位
                
                if ctrlMode<0:#这里如果设置成<0（小于0），表示是空命令，模块不会向外发布消息。
                    continue
                
                if ctrlMode == 2:#这里如果设置成2，对应了sendPosNED命令
                    self.SendPosNED(controls[0],controls[1],controls[2],controls[3])
                    # TODO: 这里应该去修改SendPosNED函数，实现发送航路点自动飞过去的功能。
                    # 其实可以改造下，直接用self.CurFlag == 12的平飞模式代替
                
                if ctrlMode == 9:#这里是设置解锁标志位
                    isArm=controls[0]
                    self.SendMavArm(isArm)
                
                if ctrlMode == 10:#这里如果设置成10，表示设置飞机的速度和和盘旋半径
                    #controls[0]表示速度（仅适用于固定翼发航点的模式）
                    #controls[1]表示设置盘旋半径（仅适用于固定翼发航点的模式）
                    self.SendCruiseSpeed(controls[0]) # 设置固定翼巡航速度（飞往航路点的速度）
                    self.SendCruiseRadius(controls[1]) # 设置固定翼的盘旋半径
                
                if ctrlMode == 11: # 表示是Mavlink的本地起飞命令
                    #controls[0]~controls[2]表示期望的本地起飞坐标。（若飞机未解锁，会自动解锁）
                    if not isInTakeoffMode: # 如果不在起飞模式，则响应起飞命令
                        self.isArmed=True # 自动解锁
                        self.sendMavTakeOff(controls[0],controls[1],controls[2]) # 发送起飞命令
                        isInTakeoffMode=True
                
                if ctrlMode == 12: # 这里如果设置成12，表示是Mavlink的全球起飞命令
                    #controls[0]~controls[2]表示期望的GPS全局起飞坐标(经纬高)。（若飞机未解锁，会自动解锁）
                    if not isInTakeoffMode:
                        self.isArmed=True # 自动解锁
                        self.sendMavTakeOffGPS(controls[0],controls[1],controls[2]) # 发送起飞命令
                        isInTakeoffMode=True
                
                if ctrlMode == 13: #这里如果设置成13，表示速度、航线和高度指令
                    #controls[0]~controls[2]表示期望的速度、高度和航向。
                    #（若飞机未解锁，会自动解锁）
                    #（若飞机未进入Offboard模式，会自动进入Offboard模式）
                    self.isArmed=True # 自动解锁
                    self.SendVelYawAlt(controls[0],controls[1],controls[2])
                    #self.CurFlag=14 速度偏航高度控制模式
                    # TODO：SendVelYawAlt内部函数需要重写
                    
        
    ## @brief 停止质点模型的运行。
    #  - @anchor EndSimpleModel
    #  停止内部循环更新模型以及Mavlink数据监听循环，并回收线程资源。        
    def EndSimpleModel(self):
        """ End the point mass model
        """
        self.isInPointMode=False
        # 停止模型更新循环
        if self.updatefreq>=1: # 如果启用Vehicle内部While循环更新
            time.sleep(0.5)
            self.t3.join()
        
        # 停止监听循环
        self.t4Flag=False
        time.sleep(0.5)
        self.t4.join()
        self.lastTimeStep=-1
        
    ## @brief 固定翼滑跑（Runway TakeOff）相关预留函数（当前未实现）。        
    #  - @anchor enFixedWRWTO
    def enFixedWRWTO(self):     
        pass    

    ## @brief 根据EnList生成POSITION_TARGET_LOCAL_NED消息的type_mask位图。
    #  - @anchor TypeMask
    #  参见：https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
    #  @param EnList(list): 含有bool的列表，用于决定启用或禁用指定参数（Pos、Vel、Acc、Force、Yaw、Yawrate）。
    #  @return 返回生成的type_mask整数值。
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


    ## @brief 点质量模型的主循环函数，以固定频率更新模型状态、发送UAV状态和输出数据到RflySim3D。
    #  - @anchor SimpleModelLoop
    #  当isInPointMode为False时退出循环。
    #  内部使用time.sleep维持固定更新频率，并调用ModelStep函数更新状态和输出。
    def SimpleModelLoop(self):
        """ This is the dead loop for point mass model
        """
        # Offboard message sending loop, 100Hz
        self.startTime3 = time.time()
        self.startTime= time.time()
        self.lastTime3 = self.startTime3

        while True:
            if not self.isInPointMode:
                break
            self.startTime3 = self.startTime3 + 1.0/self.updatefreq
            currTime = time.time()
            sleepTime = self.startTime3 - currTime
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.startTime3 = currTime
            
            # 将目前的信息传入，开始更新模型
            self.ModelStep(currTime)
            
        #print("Point Mode Stoped.")

    ## @brief 模型单步更新函数。
    #  - @anchor ModelStep
    #  根据当前时间与上一次更新的时间差dt对模型状态进行推进和计算。
    #  包含对输入处理(ProssInput)、步进(Step)以及状态发送(SendUavState、SendOutput)。
    #  @param CurTime(float): 当前时间戳，若为负数则自动获取当前时间。    
    def ModelStep(self,CurTime=-1):
        if CurTime<0:
            CurTime = time.time()
        if self.lastTimeStep <0:
            self.lastTimeStep=time.time()
        dt = CurTime - self.lastTimeStep
        self.lastTimeStep = CurTime
        if dt<=0:
            return
        
        # 下面开始根据步长间隔，更新模型
        if self.isArmed: # 如果解锁了，才更新控制输入
            self.ProssInput(dt) # 这个函数里面，处理用户的输入命令
            
        self.Step(dt) # 这个函数里面，用龙格库塔法之类，向前推进一步
        self.SendUavState(CurTime) # 这个函数里面，将数据发送给20101端口给Simulink，模拟PX4内部估计状态
        self.SendOutput(CurTime) # 这个函数里面，将数据发送给RflySim3D，模拟飞机的三维真值数据     
    
    ## @brief 发送UAV的状态数据给Simulink，用于模拟PX4内部估计状态。
    #  - @anchor SendUavState
    #  数据结构参考 outHILStateShort:
    #    int checksum; //校验位1234567890
    #    int32_t gpsHome[3]; // Home GPS位置(lat,long以度*1e7，alt以m*1e3向上为正)
    #    float AngEular[3];  // 估计的欧拉角，单位：弧度
    #    float localPos[3];  // 估计的本地位置，NED，单位：m
    #    float localVel[3];  // 估计的本地速度，NED，单位：m/s
    #  打包后发送给设定的IP和端口（SendIpPort），用于与Simulink通信。
    #  @param CurTime(float): 当前时间戳，用于标记发送数据的时间（本函数中未使用）。            
    def SendUavState(self,CurTime):
        # struct outHILStateShort{
        #     int checksum; //校验位1234567890
        #     int32_t gpsHome[3];     //Home GPS position, lat&long: deg*1e7, alt: m*1e3 and up is positive
        #     float AngEular[3];    //Estimated Euler angle, unit: rad
        #     float localPos[3];    //Estimated locoal position, NED, unit: m
        #     float localVel[3];    //Estimated locoal velocity, NED, unit: m/s
        # } 4i9f
        checkSum=1234567890
        lat = int(self.GPSOrigin[0]*1e7)
        lon = int(self.GPSOrigin[1]*1e7)
        alt = int(self.GPSOrigin[2]*1e3)
        gpsHome = [lat,lon,alt] # 转化为经纬高的int型
        AngEular = self.uavAngEular
        localPos = self.uavPosNED
        localVel = self.uavVelNED
        checkSum=1234567890
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("4i9f",checkSum,*gpsHome,*AngEular,*localPos,*localVel)
        self.udp_socket.sendto(buf, self.SendIpPort) # 注意，这里设置的是20101系列端口，IP地址由用户输入

    ## @brief 根据当前模式和控制指令对输入进行处理的函数。
    #  - @anchor ProssInput
    #  包括Offboard模式、起飞模式（滑跑和爬升阶段）以及平飞和盘旋模式。
    #  不同的模式根据状态和目标点计算期望速度、偏航角和俯仰角，并存储在相应的成员变量中，为后续的模型更新提供参考值。
    #  @param dt(float): 本次调用与上次调用之间的时间间隔，单位：秒        
    def ProssInput(self,dt):
        # 这是一个输入处理函数
        
        if self.CurFlag == 0: # 如果是Offboard模式
        
            # 下面分别是期望的速度、偏航和偏航角速度
            self.velOff=list(self.vel)
            self.yawOff=self.yaw
            self.yawRateOff=self.yawrate
            
            # 根据预设的飞行模式，来计算期望的速度
            # 如果是速度模式，那么直接将速度传递给模型
            # 如果是位置模式，还要看目前离期望位置有多远，再计算期望速度
            # 另外，还需要考虑是在机体坐标系还是地球坐标系
            
            # Calculate desired speed according to target Position
            if self.EnList[1]!=0: # if speed mode
                self.velOff=list(self.vel)
                self.yawRateOff=self.yawrate
            elif self.EnList[0]!=0: # if position mode
                targetPosE=list(self.pos)
                if self.coordinate_frame == mavlink2.MAV_FRAME_BODY_NED:
                    targetPosE[0] = self.pos[0]*math.cos(self.uavAngEular[2])+self.pos[1]*math.sin(self.uavAngEular[2])
                    targetPosE[1] = self.pos[0]*math.sin(self.uavAngEular[2])+self.pos[1]*math.cos(self.uavAngEular[2])
                self.velOff[0]=self.sat((targetPosE[0]-self.uavPosNED[0])*0.5,self.MaxSpeed)
                self.velOff[1]=self.sat((targetPosE[1]-self.uavPosNED[1])*0.5,self.MaxSpeed)
                self.velOff[2]=self.sat((targetPosE[2]-self.uavPosNED[2])*0.5,self.MaxSpeed)
                self.yawRateOff=self.sat((self.yawOff-self.uavAngEular[2])*2,math.pi/4)
            else:
                self.velOff=[0,0,0]
                self.yawOff=0
                self.yawRateOff=0
                
        if self.CurFlag == 10: # 如果是起飞模式的滑跑阶段
            
            if self.lastFlag != self.CurFlag: # 初次进入本模式，需要配置状态和参数
                self.uavVelNED=[0,0,0]
                self.VehiclePitch = 0 # 固定翼的俯仰角
                self.VehicleYaw = self.TkOffYaw
                #self.uavAngEular[2] = self.TkOffYaw
                print('起飞模式的滑跑阶段')
                self.lastFlag=self.CurFlag
            
            LastSpeed = math.sqrt(self.velOff[0]*self.velOff[0] + self.velOff[1]*self.velOff[1]) # 当前速度
            curSpeed = LastSpeed + self.TkOffAccSpeed*dt # 做加速运动
            if curSpeed >= self.TkOffMaxSpeed:
                curSpeed = self.TkOffMaxSpeed
            
            self.yawOff = self.TkOffYaw # 滑跑的偏航角
            self.velOff[0] = curSpeed * math.cos(self.yawOff)
            self.velOff[1] = curSpeed * math.sin(self.yawOff)
            self.velOff[2]=0
            
            self.VehiclePitch = 0
            if curSpeed >= self.TkOffMaxSpeed: # 如果达到预设的起飞速度，就切换到滑跑起飞模式
                self.CurFlag = 11
            
        if self.CurFlag == 11: # 如果是起飞爬升阶段
            
            if self.lastFlag != self.CurFlag: # 初次进入本模式，需要配置状态和参数
                self.VehiclePitch = self.ClimbSlope+0.1
                self.VehicleYaw = self.TkOffYaw
                print('起飞模式的爬升阶段')
                self.lastFlag=self.CurFlag
                
            LastSpeed = math.sqrt(self.velOff[0]*self.velOff[0] + self.velOff[1]*self.velOff[1]+ self.velOff[2]*self.velOff[2]) # 当前速度
            curSpeed = LastSpeed + self.ClimbAccSpeed*dt # 做加速运动
            if curSpeed > self.ClimbMaxSpeed:
                curSpeed = self.ClimbMaxSpeed

            self.yawOff = self.TkOffYaw # 滑跑的偏航角
            
            self.velOff[0] = curSpeed * math.cos(self.yawOff)*math.cos(self.ClimbSlope)
            self.velOff[1] = curSpeed * math.sin(self.yawOff)*math.cos(self.ClimbSlope)  
            self.velOff[2] = -curSpeed *math.sin(self.ClimbSlope)           
            
            fxVel = math.sqrt(self.uavVelNED[0]*self.uavVelNED[0]+self.uavVelNED[1]*self.uavVelNED[1])
            self.VehiclePitch = self.fixedPitchFromVel(fxVel)
            if -self.uavPosNED[2] > -self.ClimbMaxAlt: # 达到起飞高度
                self.CurFlag = 12 # 切换到下一个模式
        
        
        if self.CurFlag == 12: # 如果平飞模式
            
            if self.lastFlag != self.CurFlag: # 初次进入本模式，需要配置状态和参数
                print('起飞模式的平飞阶段')
                self.lastFlag=self.CurFlag
                
                #self.VehiclePitch = self.fixedPitchToVel(self.ForwardSpeed)  
            targetPosE = self.ForwardPosXYZ
            YErr = self.ForwardPosXYZ[1] - self.uavPosNED[1]
            XErr = self.ForwardPosXYZ[0] - self.uavPosNED[0]
            zErr = self.ForwardPosXYZ[2] - self.uavPosNED[2]
            targetyaw = math.atan2(YErr,XErr)
            targetSlop = -math.atan2(zErr,math.sqrt(YErr*YErr+XErr*XErr))
            self.velOff[0] = self.ForwardSpeed * math.cos(targetyaw)*math.cos(targetSlop)
            self.velOff[1] = self.ForwardSpeed * math.sin(targetyaw)*math.cos(targetSlop)  
            self.velOff[2] = -self.ForwardSpeed *math.sin(targetSlop)               
            self.VehicleYaw = targetyaw
            dist = math.sqrt(YErr*YErr+XErr*XErr+zErr*zErr)
            if dist <= self.CircleRadius:
                self.CurFlag=121
            fxVel = math.sqrt(self.uavVelNED[0]*self.uavVelNED[0]+self.uavVelNED[1]*self.uavVelNED[1])
            self.VehiclePitch = self.fixedPitchFromVel(fxVel)
                
        if self.CurFlag == 121: # 如果盘旋模式
            if self.lastFlag != self.CurFlag:  # 初次进入本模式，需要配置状态和参数
                print('起飞模式的盘旋模式')
                self.lastFlag = self.CurFlag

            W = self.CircleSpeed*dt
            self.CircleStep = self.CircleStep+W*dt
            angle = self.yawSat(self.CircleStep)
            if self.CircleDir == 1:  # 顺时针盘旋
                self.velOff[0] = self.CircleRadius*math.sin(angle)#self.TranTimingfun(self.velOff[0],self.CircleRadius*math.sin(angle),dt)
                self.velOff[1] = self.CircleRadius*math.cos(angle)#self.TranTimingfun(self.velOff[1],self.CircleRadius*math.cos(angle),dt)
                
            else:  # 逆时针盘旋
                self.velOff[0] = -self.CircleRadius*math.sin(-angle)
                self.velOff[1] = -self.CircleRadius*math.cos(-angle)

            
            fxVel = math.sqrt(self.uavVelNED[0]*self.uavVelNED[0]+self.uavVelNED[1]*self.uavVelNED[1])
            self.VehiclePitch = self.fixedPitchFromVel(fxVel)


    ## @brief 根据当前速度vel来确定飞行器的俯仰角。
    #  - @anchor fixedPitchFromVel 
    #  当vel > 20 m/s时，设定俯仰角为2度；
    #  当5 < vel < 20 m/s时，俯仰角在2度到12度之间线性变化；
    #  当vel <= 5 m/s时，设定俯仰角为12度。
    #  @param vel(float): 当前飞行速度，单位：m/s
    #  @return pitch(float): 对应的俯仰角（弧度）
    def fixedPitchFromVel(self,vel):
        pitch=0
        if vel >20:
            pitch=2/180.0*math.pi
        
        if vel>5 and vel < 20:
            pitch = (2+(20-vel)/15.0*10)/180.0*math.pi # 2度到12度，5m/s到20m/s
            
        if vel <=5:
            pitch = 12/180.0*math.pi
        return pitch
   
    ## @brief 使用龙格-库塔法以及内部定义的动力学方程，进行状态更新（位置、速度、高度、航向等）。
    #  - @anchor Step
    #  根据当前模式（CurFlag）和飞行器状态，求解飞行器在dt时间间隔内的状态演进。
    #  包含高度控制、空速控制、航向控制以及水平面位置积分等过程。
    #  @param dt(float): 时间步长，单位为秒。                    
    def Step(self,dt):  
        if self.lastFlag!=self.CurFlag:
            self.uavVelAir=math.sqrt(self.uavVelNED[0]*self.uavVelNED[0]+self.uavVelNED[1]*self.uavVelNED[1])
            self.lastFlag = self.CurFlag
            
        velE=list(self.velOff)

        # 高度控制函数g(u)：输入为控制变量u，输出为相同值
        def g(u):
            return u
        
        # f(t, Hc, Hcd, hr, u)为高度控制微分方程，利用参数B中b3和b4对高度误差(hr与Hc)、高度变化率误差(u与Hcd)进行反馈控制。
        # Hc: 当前高度期望值, Hcd: 高度期望变化率, hr: 当前真实高度, u: 实际控制输入(如当前高度或变化率)    
        def f(t, Hc, Hcd, hr, u):
            b3 = self.B[2]
            b4 = self.B[3]
            return b3 * (Hcd - u) + b4 * (Hc - hr)
        # Vr(t, Vc, vr): 空速控制函数，根据空速误差(Vc - vr)进行控制
        # Vc: 期望速度, vr: 当前真实速度
        def Vr(t, Vc, vr):
            bv = self.B[4]
            return bv * (Vc - vr)

        # 航向角控制函数
        def q(d):
            return d
        
        # Yrd(vr, Lar): 计算惯性坐标系下东向速度变化率，用vr和Lar（航向角）来计算Y方向速度分量变化
        def Yawr(ti, LAc, LAcd, LAr, d):
            b1 = self.B[0]
            b2 = self.B[1]
            return b1 * (LAcd - d) + b2 * (LAc - LAr)

        # 飞机的惯性空间北向速度:Xrd；
        # 飞机惯性的空间东向速度:Yrd。
        def Xrd(vr, Lar):
            # 速度的积分：Xr=-vr**2*sin(Lar)/2
            return vr * math.cos(Lar)

        def Yrd(vr, Lar):
            # 速度的积分：Yr=vr**2*cos(Lar)/2
            return vr * math.sin(Lar)

        # 四阶龙格库塔法求解二阶微分方程
        def Sec_RK4(dt, g, f, x1, x2, x3, x4):
            ti = time.time()
            m1 = g(x4)
            k1 = f(ti, x1, x2, x3, x4)

            m2 = g(x4 + k1 * dt / 2)
            k2 = f(ti+dt/2, x1 + k1 * dt / 2, x2 + k1 *dt / 2, x3 + k1 * dt / 2, x4 + m1 * dt / 2)

            m3 = g(x4 + k2 * dt / 2)
            k3 = f(ti+dt/2, x1 + k2 * dt / 2, x2 + k2 *dt / 2, x3 + k2 * dt / 2, x4 + m2 * dt / 2)

            m4 = g(x4 + k3 * dt / 2)
            k4 = f(ti+dt/2, x1 + k3 * dt / 2, x2 + k3 *dt / 2, x3 + k3 * dt / 2, x4 + m3 * dt / 2)

            F = (dt / 6) * (m1 + 2 * (m2 + m3) + m4)
            Fd = (dt / 6) * (k1 + 2 * (k2 + k3) + k4)

            return F, Fd
        # 四阶龙格库塔法求解一阶微分方程

        def Fir_RK4(dt, f, x1, x2):
            ti = time.time()
            k1 = f(ti, x1, x2)
            k2 = f(ti+dt/2, x1 + k1 * dt / 2, x2 + k1 * dt / 2)
            k3 = f(ti+dt/2, x1 + k2 * dt / 2, x2 + k2 * dt / 2)
            k4 = f(ti+dt/2, x1 + k3 * dt / 2, x2 + k3 * dt / 2)
            F = (dt / 6) * (k1 + 2 * (k2 + k3) + k4)
            return F
        
        if self.CurFlag==2:  #位置模型
            self.yawf = self.yaw
            self.yawRatef = self.yawrate
            self.velEf = self.vel
            self.targetPosEf = self.pos
            self.VelAirf=self.velair

            XErr = self.targetPosEf[0] - self.uavPosNED[0]
            YErr = self.targetPosEf[1] - self.uavPosNED[1]
            ZErr = self.targetPosEf[2] - self.uavPosNED[2]

            targetyaw = math.atan2(YErr, XErr)
            targetSlop = -math.atan2(ZErr, math.sqrt(YErr*YErr+XErr*XErr))
            vy1 = self.MaxSpeed * math.sin(targetyaw)*math.cos(targetSlop)
            vx1 = self.MaxSpeed * math.cos(targetyaw)*math.cos(targetSlop)

            self.VelAirf=sqrt(vx1*vx1+vy1*vy1)

                        
            #targetyaw1=acos(vx1/self.VelAirf)
            # if vx1>0:
            #     targetyaw1=asin(vy1/self.VelAirf)
            # if vx1<0 and vy1<0:
            #     targetyaw1=acos(-vx1/self.VelAirf)-math.pi
            
            targetyaw1=atan2(vy1,vx1)
            # if targetyaw1<-3:
            #     targetyaw1=atan2(vy1,vx1)-math.pi

            
            
            h1, hd1 = Sec_RK4(dt, g, f, self.targetPosEf[2], self.velEf[2], self.uavPosNED[2],self.uavVelNED[2])
            VRk1 = Fir_RK4(dt, Vr, self.VelAirf, self.uavVelAir)
            yaw, yawd = Sec_RK4(dt, q, Yawr, targetyaw1, self.yawRatef, self.uavAngEular[2],self.uavAngRate[2])
            dist = math.sqrt(YErr*YErr+XErr*XErr+ZErr*ZErr)
            if dist <= 0.5:
                vx1=0
                vy1=0
                hd1=0                
            fxVel = math.sqrt(self.uavVelNED[0]*self.uavVelNED[0]+self.uavVelNED[1]*self.uavVelNED[1])
            self.VehiclePitch = self.fixedPitchFromVel(fxVel)


            self.uavAngRate[0] = 0
            self.uavAngRate[1] = 0
            self.uavAngRate[2] = yawd  # self.sat(yawd,math.pi)


            self.uavAngEular[0] = 0
            self.uavAngEular[1] = self.uavAngEular[1]*0.97 + self.VehiclePitch*0.03
            if targetyaw1<-3 :
                self.uavAngEular[2] = -targetyaw1 
            else:
                self.uavAngEular[2] = self.uavAngEular[2]+self.uavAngRate[2]*dt*8
            
            #self.uavAngEular[2] = self.uavAngEular[2]*0.97+targetyaw*0.03
            

            self.uavVelNED[0] = self.sat(self.uavVelNED[0]*0.97+vx1*0.03, 15)
            self.uavVelNED[1] = self.sat(self.uavVelNED[1]*0.97+vy1*0.03, 15)
            self.uavVelNED[2] = self.sat(self.uavVelNED[2]*0.97+hd1*0.03, 20)

            self.uavVelAir = self.uavVelAir+VRk1

        elif self.CurFlag == 13:     # 速度高度偏航模型 
            print("进入RK41")
            self.yawf = self.vel[0]
            self.VelAirf=self.vel[1]
            self.yawRatef = self.yawrate
            self.velEf = [0,0,0]
            self.targetPosEf = self.pos
            

            yaw, yawd = Sec_RK4(dt, q, Yawr, self.yawf, self.yawRatef, self.uavAngEular[2],self.uavAngRate[2])
            h1, hd1 = Sec_RK4(dt, g, f, self.targetPosEf[2], self.velEf[2], self.uavPosNED[2],self.uavVelNED[2])
            VRk1 = Fir_RK4(dt, Vr, self.VelAirf, self.uavVelAir)
            vx1 = Xrd(self.uavVelAir, self.uavAngEular[2])
            vy1 = Yrd(self.uavVelAir, self.uavAngEular[2])


            rolld=math.atan2(vy1,-hd1)
            pitchd=math.atan2(-hd1,vx1)
            if self.uavAngRate[0]<0.005 and self.uavAngRate[0]>-0.005:
                rolld=0
            self.uavAngRate[0] = rolld
            self.uavAngRate[1] = pitchd
            self.uavAngRate[2] = yawd

            self.uavAngEular[0] = self.uavAngEular[0]+rolld*dt                   
            self.uavAngEular[1] = 0
            self.uavAngEular[2] = self.uavAngEular[2]+yaw

            
            self.uavVelNED[0] = vx1
            self.uavVelNED[1] = vy1
            self.uavVelNED[2] = hd1

            #self.uavVelNED[0] = self.sat(self.uavVelNED[0]*0.97+vx1*0.03, self.MaxSpeed)
            #self.uavVelNED[1] = self.sat(self.uavVelNED[1]*0.97+vy1*0.03, self.MaxSpeed)
            self.uavVelNED[2] = self.sat(self.uavVelNED[2]*0.97+hd1*0.03, self.MaxSpeed)

            self.uavVelAir = self.uavVelAir+VRk1
        
        else:
            self.uavVelNED[0] = velE[0]
            self.uavVelNED[1] = velE[1]
            self.uavVelNED[2] = velE[2]

            self.uavVelNED[0] = self.sat(self.uavVelNED[0]*0.97+velE[0]*0.03, 15)
            self.uavVelNED[1] = self.sat(self.uavVelNED[1]*0.97+velE[1]*0.03, 15)
            self.uavVelNED[2] = self.sat(self.uavVelNED[2]*0.97+velE[2]*0.03, 20)

            self.uavAngEular[1] = self.uavAngEular[1]*0.97+self.VehiclePitch*0.03
            self.uavAngEular[2] = math.atan2(self.velOff[1], self.velOff[0])  #self.uavAngEular[2]*0.97+ self.VehicleYaw*0.03
            
            
            if self.CurFlag == 121 and self.CircleDir == 1:  # 顺时针盘旋
                self.uavAngEular[0] = -abs(self.yawSat(math.atan2(self.velOff[1], -self.velOff[2]))*0.2)
            elif self.CurFlag == 121 and self.CircleDir == 2: # 逆时针盘旋
                self.uavAngEular[0] = abs(self.yawSat(math.atan2(self.velOff[1], -self.velOff[2]))*0.2)
            else:
                self.uavAngEular[0] = 0

        #self.uavAngRate[2] = self.sat(self.uavAngRate[2]*0.97+self.yawRateOff*0.03, math.pi)

        self.uavPosNED[0] = self.uavPosNED[0]+self.uavVelNED[0]*dt
        self.uavPosNED[1] = self.uavPosNED[1]+self.uavVelNED[1]*dt
        self.uavPosNED[2] = self.uavPosNED[2]+self.uavVelNED[2]*dt
        
        
        TrueX=self.intStateX+self.uavPosNED[0]
        TrueY=self.intStateY+self.uavPosNED[1]
        TrueAlt=self.intAlt+self.uavPosNED[2]
        TerAlt = self.map.getTerrainAltData(TrueX,TrueY)
        if TrueAlt>TerAlt and self.velOff[2]>0:
            self.uavVelNED=list([0,0,0])
            self.uavAngRate[2]=0

    
        if abs(self.uavVelNED[0])>0:
            self.MotorRPMS = [1000, 0, 0, 0, 0, 0, 0, 0]
        else:
            self.MotorRPMS = [0, 0, 0, 0, 0, 0, 0, 0]
    
    ## @brief 将当前飞行器的真实状态（位置、姿态、速度）发送给RflySim3D以更新显示。
    #  - @anchor SendOutput
    #  其中的truePosNED和trueAngEular等值均由当前状态（uavPosNED、uavAngEular等）和初始偏置（intStateX、intStateY、intAlt、intStateYaw）计算得出。
    #  通过sendUE4PosNew函数将飞行器的最新状态发送给RflySim3D进行渲染。
    #  @param CurTime(float): 当前时间戳，用于计算飞行时间并随同状态一起发送。
    def SendOutput(self,CurTime):
        # 这里更新细细发送到RflySim3D

        self.trueVelNED=list(self.uavVelNED)
        self.trueAngRat=list(self.uavAngRate)
        self.truePosNED[0]=self.intStateX+self.uavPosNED[0]
        self.truePosNED[1]=self.intStateY+self.uavPosNED[1]
        self.truePosNED[2]=self.intAlt+self.uavPosNED[2]
        self.trueAngEular[0]=self.uavAngEular[0]
        self.trueAngEular[1]=self.uavAngEular[1]
        #self.trueAngEular[2]=self.yawSat(self.uavAngEular[2]+self.intStateYaw)
        self.trueAngEular[2]=self.uavAngEular[2]+self.intStateYaw
        self.uavGlobalPos=list(self.truePosNED)
        self.uavTimeStmp=CurTime
        self.trueTimeStmp=CurTime
        
        # Send vehicle to UE4
        #sendUE4PosNew(self,copterID,vehicleType,PosE,AngEuler,Vel,PWMs,runnedTime
        runnedTime = CurTime-self.startTime
        self.sendUE4PosNew(self.CopterID,self.Vehicletype,self.truePosNED,self.trueAngEular,self.trueVelNED,self.MotorRPMS,runnedTime)

    ## @brief 发送目标位置与偏航指令给PX4（NED坐标系）。
    #  设置CurFlag=2表示进入某种自定义模式(例如Offboard位置控制)，offMode=0表示SET_POSITION_TARGET_LOCAL_NED消息格式。
    #  启用位置和偏航控制相关的使能位，禁用速度、加速度等其他通道。
    #  @param x(float): 目标位置X方向（北向），单位：米
    #  @param y(float): 目标位置Y方向（东向），单位：米
    #  @param z(float): 目标位置Z方向（下为正），当飞行器高于地面时z<0
    #  @param yaw(float): 目标偏航角（弧度）        
    def SendPosNED(self,x=0,y=0,z=0,yaw=0):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.CurFlag=2
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [1,0,0,0,1,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = yaw

    ## @brief 发送目标位置指令给PX4（NED坐标系，不控制偏航）。
    #  设置CurFlag=0表示Offboard模式，offMode=0表示SET_POSITION_TARGET_LOCAL_NED消息格式。
    #  仅启用位置控制，使能位中不启用yaw控制通道。
    #  @param x(float): 目标位置X方向（北向），单位：米
    #  @param y(float): 目标位置Y方向（东向），单位：米
    #  @param z(float): 目标位置Z方向（下为正），当飞行器高于地面时z<0
    def SendPosNEDNoYaw(self,x=0,y=0,z=0):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame without yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.CurFlag=0
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [1,0,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0

    # send target position in body FRD frame
    def SendPosFRD(self,x=0,y=0,z=0,yaw=0):
        """ Send vehicle targe position (m) to PX4 in the body forward-rightward-downward (FRD) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """
        self.CurFlag=0
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [1,0,0,0,1,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = yaw

    # send target position in body FRD frame
    def SendPosFRDNoYaw(self,x=0,y=0,z=0):
        """ Send vehicle targe position (m) to PX4 in the body forward-rightward-downward (FRD) frame without yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """

        self.CurFlag=0
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [1,0,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0

    # send velocity control signal in earth north-east-down (NED) frame to Pixhawk
    def SendVelNED(self,vx=0,vy=0,vz=0,yawrate=0):
        """ Send targe vehicle speed (m/s) to PX4 in the earth north-east-down (NED) frame with yawrate (rad/s)
        when the vehicle fly upward, the vz < 0
        """

        self.CurFlag=0
        self.EnList = [0,1,0,0,0,1]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate

    # send velocity control signal in earth north-east-down (NED) frame to Pixhawk
    def SendVelNEDNoYaw(self,vx,vy,vz):
        """ Send targe vehicle speed (m/s) to PX4 in the earth north-east-down (NED) frame without yaw control
        when the vehicle fly upward, the vz < 0
        """

        self.CurFlag=0
        self.EnList = [0,1,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = 0

    # send velocity control signal in body front-right-down (FRD) frame
    def SendVelFRD(self,vx=0,vy=0,vz=0,yawrate=0):
        """ Send vehicle targe speed (m/s) to PX4 in the body forward-rightward-downward (FRD) frame with yawrate control (rad/s)
        when the vehicle fly upward, the vz < 0
        """
        self.CurFlag=0
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [0,1,0,0,0,1]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate

    # send velocity control signal in body front-right-down (FRD) frame
    def SendVelNoYaw(self,vx,vy,vz):
        """ Send vehicle targe speed (m/s) to PX4 in the body forward-rightward-downward (FRD) frame without yawrate control (rad)
        when the vehicle fly upward, the vz < 0
        """
        # 缩小倍数
        P=1
        self.CurFlag=0
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.EnList = [0,1,0,0,0,0]
        self.type_mask=self.TypeMask(self.EnList)
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = 0
    
    def sendBuf(self,buf,windowID=-1):
        if windowID<0:
            if not self.isBroadCast: 
                for i in range(6):
                    self.udp_socket.sendto(buf, ('127.0.0.1', 20010+i))
            else:
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009)) #multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            
            if not self.isBroadCast:
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
            else:
                self.udp_socket.sendto(buf, ('255.255.255.255', 20010+windowID)) #ensure this PC can reciver message under specify IP mode
                        
    def sendUE4PosNew(self,copterID=1,vehicleType=3,PosE=[0,0,0],AngEuler=[0,0,0],VelE=[0,0,0],PWMs=[0]*8,runnedTime=-1,windowID=-1):
        """ send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
            # //输出到模拟器的数据
            # struct SOut2SimulatorSimpleTime {
            #     int checkSum; //1234567890
            #     int copterID;  //Vehicle ID
            #     int vehicleType;  //Vehicle type
            #     float PWMs[8];
            #     float VelE[3];
            #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            #     double PosE[3];   //NED vehicle position in earth frame (m)
            #     double runnedTime; //Current Time stamp (s)
            # };
            #struct.pack 3i14f4d
        """
 
        checkSum=1234567890

        buf = struct.pack("3i14f4d",checkSum,copterID,vehicleType,*PWMs,*VelE,*AngEuler,*PosE,runnedTime)
        self.sendBuf(buf,windowID)
                
    def SendCruiseSpeed(self,Speed=10): 
        """ Send command to change the Cruise speed (m/s) of the aircraft
        """
        #def SendCruiseSpeed(self,Speed,Type=1,Throttle=-1,Relative=0): 
        #  type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed); min:0 max:3 increment:1
        #  Speed (-1 indicates no change); min: -1; Unit: m/s
        #  Throttle (-1 indicates no change); min: -1; Unit: %
        #  Relative	0: absolute, 1: relative; min:0 max:1 increment:1
        #self.SendMavCmdLong(mavlink2.MAV_CMD_DO_CHANGE_SPEED, Type,Speed,Throttle,Relative,0,0,0)
        #self.sendMavSetParam('NAV_LOITER_RAD'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        #self.sendMavSetParam('FW_AIRSPD_TRIM'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        if Speed<5:
            Speed=5 # 固定翼的巡航速度应该不会低于5m/s
        
        self.FwSpeedTrim = Speed

    def SendCopterSpeed(self,Speed=5): 
        """ send command to set the maximum speed of the multicopter
        """
        # 最小3，最大20，默认5
        #self.sendMavSetParam('MPC_XY_VEL_MAX'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        self.MaxSpeed=Speed

    def SendGroundSpeed(self,Speed=15): 
        """ Send command to change the ground speed (m/s) of the aircraft
        """
        #self.sendMavSetParam('GND_SPEED_TRIM'.encode(), Speed, mavlink2.MAV_PARAM_TYPE_REAL32)
        self.GNDSpeed=Speed

    def SendCruiseRadius(self,rad=20): 
        """ Send command to change the Cruise Radius (m) of the aircraft
        """
        #self.sendMavSetParam('NAV_LOITER_RAD'.encode(), rad, mavlink2.MAV_PARAM_TYPE_REAL32)
        if rad<5:
            rad=5 #固定翼的盘旋半径，应该不会小于5m
        self.CircleRadius = rad

    def SendMavArm(self, isArm=0):
        self.isArmed = isArm

    def sendUE4Cmd(self,cmd,windowID=-1):
        """send command to control the display style of RflySim3D
            The available command are list as follows, the command string is as b'RflyShowTextTime txt time'
            RflyShowTextTime(String txt, float time)\\ let UE4 show txt with time second
            RflyShowText(String txt)\\  let UE4 show txt 5 second
            RflyChangeMapbyID(int id)\\ Change the map to ID (int number)
            RflyChangeMapbyName(String txt)\\ Change to map with name txt
            RflyChangeViewKeyCmd(String key, int num) \\ the same as press key + num on UE4
            RflyCameraPosAngAdd(float x, float y, float z,float roll,float pitch,float yaw) \\ move the camera with x-y-z(m) and roll-pitch-yaw(degree) related to current pos
            RflyCameraPosAng(float x, float y, float z, float roll, float pitch, float yaw) \\ set the camera with x-y-z(m) and roll-pitch-yaw(degree) related to UE origin
            RflyCameraFovDegrees(float degrees) \\ change the cameras fov (degree)
            RflyChange3DModel(int CopterID, int veTypes=0) \\ change the vehicle 3D model to ID
            RflyChangeVehicleSize(int CopterID, float size=0) \\change vhielce's size
            RflyMoveVehiclePosAng(int CopterID, int isFitGround, float x, float y, float z, float roll, float pitch, float yaw) \\ move the vehicle's  x-y-z(m) and roll-pitch-yaw(degree) related to current pos
            RflySetVehiclePosAng(int CopterID, int isFitGround, float x, float y, float z, float roll, float pitch, float yaw) \\ set the vehilce's x-y-z(m) and roll-pitch-yaw(degree) related to UE origin
            RflyScanTerrainH(float xLeftBottom(m), float yLeftBottom(m), float xRightTop(m), float yRightTop(m), float scanHeight(m), float scanInterval(m)) \\ send command to let UE4 scan the map to generate png and txt files
            RflyCesiumOriPos(double lat, double lon, double Alt) \\ change the lat, lon, Alt (degrees) of the Cesium map origin
            RflyClearCapture \\ clear the image capture unit
            struct Ue4CMD0{
                int checksum;
                char data[52];
            } i52s
            struct Ue4CMD{
                int checksum;
                char data[252];
            } i252s            
            
        """
        #print(len(cmd))
        if len(cmd)<=51:
            buf = struct.pack("i52s",1234567890,cmd)
        elif len(cmd)<=249:
            buf = struct.pack("i252s",1234567890,cmd)
        else:
            print('Error: Cmd is too long')
            return
        self.sendBuf(buf,windowID)

    # 发送速度、偏航和高度信号的接口
    # send target position in earth NED frame
    def SendVelYawAlt(self,vel=10,alt=-100,yaw=6.28):
        """ Send vehicle targe position (m) to PX4 in the earth north-east-down (NED) frame with yaw control (rad)
        when the vehicle fly above the ground, then z < 0
        """

        self.CurFlag = 13 # 速度偏航高度控制模式
        if abs(yaw)<0.00001:
            yaw = 6.28
        self.offMode=0 # SET_POSITION_TARGET_LOCAL_NED
        self.type_mask=int("000111000000", 2)
        self.coordinate_frame = 1
        self.pos=[0,0,alt]
        self.vel = [yaw,vel,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = 0
    
    def sendMavTakeOff(self,xM=0,yM=0,zM=0,YawRad=0,PitchRad=20/180.0*math.pi):
        """ Send command to make aircraft takeoff to the desired local position (m)
        """ 
        self.CurFlag=10 # 进入起飞模式

        self.MaxSpeed = 20

        # 填充号后续起飞模式的参数
        self.TkOffYaw = YawRad # 滑跑的偏航方向
        self.TkOffAccSpeed = 1 # 滑跑的加速度
        self.TkOffMaxSpeed = 10 # 滑跑的目标速度（达到目标速度后起飞）
        self.TkOffNextFlag = 11 # 达到目标速度后，切换到的模式，这里11模式表示固定斜率爬升
        
        # Flag = 11 -> 爬升模式，需要输入的参数如下
        self.ClimbSlope = PitchRad # 爬升的斜率
        self.ClimbStartSpeed = 10 # 爬升的加速度（速度增加）
        self.ClimbAccSpeed = 1 # 爬升的加速度（速度增加）
        self.ClimbMaxSpeed = 20 # 爬升的目标速度（达到目标速度后起飞）
        self.ClimbMaxAlt = zM # 爬升的最大高度，这里取zM
        self.ClimbNextFlag = 12 # 到达爬升高度后的下一个模式，12是平飞模式
        
        # Flag = 12 -> 平飞模式，需要输入的参数如下
        
        self.ForwardSpeed = 20 # 平飞速度
        self.ForwardPosXYZ = [xM,yM,zM] # 下一个航路点的位置
        self.ForwardNextFlag = 13 # 平飞模式的下一个模式，13是盘旋模式
        
        # Flag = 13 -> 盘旋模式，需要输入的参数如下
        self.CircleSpeed = 20 # 盘旋的速度
        self.CircleRadius = 20 # 盘旋的半径
        self.CirclePosXYZ = [xM,yM,zM] # 盘旋的中心坐标
        self.CircleDir = 1 # 盘旋的方向，这里1是顺时针  2是逆时针
        self.CircleStep = 0  # 盘旋步长
        
    def sendMavTakeOffGPS(self,lat,lon,alt,yawDeg=0,pitchDeg=15):
        """ Send command to make aircraft takeoff to the desired global position (degree)
        """
        xyz = self.geo.lla2ned([lat,lon,alt],self.GPSOrigin)
        self.sendMavTakeOff(self,xyz[0],xyz[1],xyz[2],yawDeg/180.0*math.pi,pitchDeg/180.0*math.pi)
        
        #self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_TAKEOFF, pitchDeg,0,0,yawDeg,lat,lon,alt)
    
    
class UEMapServe:
    def __init__(self,name=''):
        if name == '':
            self.PosOffsetX=0
            self.PosScaleX=0
            self.PosOffsetY=0
            self.PosScaleY=0
            self.xMax=0
            self.yMax=0
            self.binmap= []
        else:
            self.LoadPngData(name)
        
    def LoadPngData(self,name):
        fileLoc=os.path.join(sys.path[0],name)
        fileLocTxt = fileLoc+'.txt'
        if not os.path.exists(fileLocTxt):
            print("Failed to find file ",fileLocTxt)
            sys.exit(0)
        fileLocPng = fileLoc+'.png'
        if not os.path.exists(fileLocPng):
            print("Failed to find file ",fileLocPng)
            sys.exit(0)        
        txtFile = open(fileLocTxt)
        line = txtFile.readline()
        txtFile.close()
        m_readData = line.split(',')
        if len(m_readData)!= 9:
            print("Failed to read data from ",fileLocTxt)
            sys.exit(0)  
        
        for i in range(len(m_readData)):
            m_readData[i] = float(m_readData[i])
        #print(fileLocPng)
        rowmap = cv2.imread(fileLocPng,cv2.IMREAD_ANYDEPTH)
        rowmap=rowmap.astype(np.float32)
        rowmap = rowmap-32768
        rows=np.size(rowmap,0)
        columns=np.size(rowmap,1)
        
        PosScaleX = (m_readData[0]-m_readData[3])/(columns-1)
        PosScaleY = (m_readData[1]-m_readData[4])/(rows-1)

        PosOffsetX = m_readData[3]
        PosOffsetY = m_readData[4]

        intCol = int((m_readData[6]-PosOffsetX)/PosScaleX )
        intRow = int((m_readData[7]-PosOffsetY)/PosScaleY )
        
        heightInit = float(rowmap[0,0])
        heightFirst = float(rowmap[rows-1,columns-1])
        heightThird = float(rowmap[intRow,intCol])

        if abs(heightThird-heightFirst)<=abs(heightThird-heightInit):
            if abs((heightInit-heightThird))>10:
                PosScaleZ = (m_readData[5]-m_readData[8])/((heightInit-heightThird))
            else:
                PosScaleZ = 1
        else:
            if abs(heightThird-heightFirst)>10:
                PosScaleZ = (m_readData[2]-m_readData[8])/((heightFirst-heightThird))
            else:
                PosScaleZ = 1

        intPosInitZ = heightInit
        PosOffsetZ = m_readData[5]

        self.PosOffsetX=PosOffsetX
        self.PosScaleX=PosScaleX
        self.PosOffsetY=PosOffsetY
        self.PosScaleY=PosScaleY
        self.xMax=abs(m_readData[0]/100)
        self.yMax=abs(m_readData[1]/100)
        self.rows=rows
        self.columns=columns
        self.binmap= -(PosOffsetZ + ((rowmap)-intPosInitZ)*PosScaleZ)/100.0
        
    def getTerrainAltData(self,xin,yin):
        if len(self.binmap)==0:
            print("Please load a map first!")
            sys.exit(0) 
        intCol = (xin*100-self.PosOffsetX)/self.PosScaleX+1
        intRow = (yin*100-self.PosOffsetY)/self.PosScaleY+1

        intColInt=int(intCol)
        intRowInt = int(intRow)
        a=intCol-intColInt
        b=intRow-intRowInt

        intRowInt1=intRowInt+1; 
        intColInt1=intColInt+1; 
        m=self.rows
        n=self.columns
        if intColInt<1:
            intColInt=1
            intColInt1=1
            a=0

        if intColInt>=n:
            intColInt=n
            intColInt1=intColInt
            a=0

        if intRowInt<1:
            intRowInt=1
            intRowInt1=1
            b=0

        if intRowInt>=m:
            intRowInt=m
            intRowInt1=intRowInt
            b=0
        binmap=self.binmap
        intRowInt=intRowInt-1
        intColInt=intColInt-1
        intRowInt1=intRowInt1-1
        intColInt1=intColInt1-1
        zz=binmap[intRowInt,intColInt]*(1-b)*(1-a)+binmap[intRowInt1,intColInt]*b*(1-a)+binmap[intRowInt,intColInt1]*(1-b)*a+binmap[intRowInt1,intColInt1]*b*a
        return zz


                
class EarthModel():
    def __init__(self):
        self.wgs84_a = 6378137
        self.wgs84_b = 6356752.3142
        self.wgs84_f = (self.wgs84_a - self.wgs84_b) / self.wgs84_a
        self.pow_e_2 = self.wgs84_f * (2-self.wgs84_f)
 
    def lla2ecef(self, lat, lon, h):
        # (lat, lon) in degrees
        # h in meters
        lamb = math.radians(lat)
        phi = math.radians(lon)
        s = math.sin(lamb)
        N = self.wgs84_a / math.sqrt(1 - self.pow_e_2 * s * s)
    
        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
    
        x = (h + N) * cos_lambda * cos_phi
        y = (h + N) * cos_lambda * sin_phi
        z = (h + (1 - self.pow_e_2) * N) * sin_lambda
    
        return x, y, z
    
    def ecef2enu(self, x, y, z, lat0, lon0, h0):
        lamb = math.radians(lat0)
        phi = math.radians(lon0)
        s = math.sin(lamb)
        N = self.wgs84_a / math.sqrt(1 - self.pow_e_2 * s * s)
    
        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
    
        x0 = (h0 + N) * cos_lambda * cos_phi
        y0 = (h0 + N) * cos_lambda * sin_phi
        z0 = (h0 + (1 - self.pow_e_2) * N) * sin_lambda
    
        xd = x - x0
        yd = y - y0
        zd = z - z0
    
        t = -cos_phi * xd -  sin_phi * yd
    
        xEast = -sin_phi * xd + cos_phi * yd
        yNorth = t * sin_lambda  + cos_lambda * zd
        zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd
    
        return xEast, yNorth, zUp
    
    def enu2ecef(self, xEast, yNorth, zUp, lat0, lon0, h0):
        lamb = math.radians(lat0)
        phi = math.radians(lon0)
        s = math.sin(lamb)
        N = self.wgs84_a / math.sqrt(1 - self.pow_e_2 * s * s)
    
        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
    
        x0 = (h0 + N) * cos_lambda * cos_phi
        y0 = (h0 + N) * cos_lambda * sin_phi
        z0 = (h0 + (1 - self.pow_e_2) * N) * sin_lambda
    
        t = cos_lambda * zUp - sin_lambda * yNorth
    
        zd = sin_lambda * zUp + cos_lambda * yNorth
        xd = cos_phi * t - sin_phi * xEast 
        yd = sin_phi * t + cos_phi * xEast
    
        x = xd + x0 
        y = yd + y0 
        z = zd + z0 
    
        return x, y, z
    
    def ecef2lla(self, x, y, z):
    # Convert from ECEF cartesian coordinates to 
    # latitude, longitude and height.  WGS-84
        x2 = x ** 2 
        y2 = y ** 2 
        z2 = z ** 2 
    
        self.wgs84_a = 6378137.0000    # earth radius in meters
        self.wgs84_b = 6356752.3142    # earth semiminor in meters 
        e = math.sqrt (1-(self.wgs84_b/self.wgs84_a)**2) 
        b2 = self.wgs84_b*self.wgs84_b 
        e2 = e ** 2 
        ep = e*(self.wgs84_a/self.wgs84_b) 
        r = math.sqrt(x2+y2) 
        r2 = r*r 
        E2 = self.wgs84_a ** 2 - self.wgs84_b ** 2 
        F = 54*b2*z2 
        G = r2 + (1-e2)*z2 - e2*E2 
        c = (e2*e2*F*r2)/(G*G*G) 
        s = ( 1 + c + math.sqrt(c*c + 2*c) )**(1/3) 
        P = F / (3 * (s+1/s+1)**2 * G*G) 
        Q = math.sqrt(1+2*e2*e2*P) 
        ro = -(P*e2*r)/(1+Q) + math.sqrt((self.wgs84_a*self.wgs84_a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2) 
        tmp = (r - e2*ro) ** 2 
        U = math.sqrt( tmp + z2 ) 
        V = math.sqrt( tmp + (1-e2)*z2 ) 
        zo = (b2*z)/(self.wgs84_a*V) 
    
        height = U*( 1 - b2/(self.wgs84_a*V) ) 
        
        lat = math.atan( (z + ep*ep*zo)/r ) 
    
        temp = math.atan(y/x) 
        if x >=0 :    
            long = temp 
        elif (x < 0) & (y >= 0):
            long = math.pi + temp 
        else :
            long = temp - math.pi 
    
        lat0 = lat/(math.pi/180) 
        lon0 = long/(math.pi/180) 
        h0 = height 
    
        return lat0, lon0, h0
    
    
    def lla2enu(self, lat, lon, h, lat_ref, lon_ref, h_ref):
    
        x, y, z = self.lla2ecef(lat, lon, h)
        
        return self.ecef2enu(x, y, z, lat_ref, lon_ref, h_ref)
    
    def enu2lla(self, xEast, yNorth, zUp, lat_ref, lon_ref, h_ref):
    
        x,y,z = self.enu2ecef(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref)
    
        return self.ecef2lla(x,y,z)
    
    def lla2ned(self, lla, lla0):
        lat=lla[0]
        lon=lla[1]
        h=lla[2]
        lat_ref=lla0[0]
        lon_ref=lla0[1]
        h_ref=lla0[2]
        xEast, yNorth, zUp=self.lla2enu(lat, lon, h, lat_ref, lon_ref, h_ref)
        return [yNorth,xEast,-zUp]   #  转换为北东地
    
    
    def ned2lla(self,ned,lla0):
        xEast=ned[1]
        yNorth=ned[0]
        zUp=-ned[2]
        lat_ref=lla0[0]
        lon_ref=lla0[1]
        h_ref=lla0[2]
        return self.enu2lla(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref)

