import time
import numpy as np
import math
import UE4CtrlAPI
ue = UE4CtrlAPI.UE4CtrlAPI()

## @file
#  该模块包含Sleep和Command两个类，用于处理无人机的任务执行过程中的等待和控制命令发送。
#  @anchor AutoMavCmd接口库文件

## @class Sleep
#  @brief 用于实现等待功能的类。
#  @details Sleep类包含用于实现等待操作的方法，可以等待固定时间或等待到达特定位置。
#  @param mav 无人机对象，用于获取无人机当前位置等信息。
class Sleep:
    ## @brief 初始化Sleep类。
    # - @anchor Sleep.__init__
    #  @param mav 无人机对象，用于获取无人机状态。
    def __init__(self,mav):
        self.CID = 1
        self.mav = mav
        self.isDone = 0 
        self.WaitFlag = 0
        self.WaitResetFlag = 0
        self.start_time = 0

    ## @brief 使无人机等待指定的时间。
    # - @anchor Wait
    #  @param times 等待的秒数。
    def Wait(self,times): 
        self.isDone = 0
        if self.WaitFlag == 0:
            print('wait {}s'.format(times[0]))
            self.start_time = time.time() + times[0]
            self.WaitFlag = 1
    
        if self.start_time - time.time() < 0: 
            self.isDone = 1
            self.WaitFlag = 0
    
    ## @brief 等待固定翼无人机到达指定位置并重置。
    # - @anchor WaitResetForFixwing
    #  @param targetPos 目标位置坐标。
    def WaitResetForFixwing(self,targetPos): 
        self.isDone = 0
        curPos=self.mav.uavPosNED
        if self.WaitResetFlag == 0:
            print('wait reset')
            ue.sendUE4Cmd('RflyShowTextTime "Wait Reset:%.2f %.2f %.2f " 10'%(targetPos[0],targetPos[1],targetPos[2]))
            self.WaitResetFlag = 1

        dis = math.sqrt((curPos[0]-targetPos[0])**2+(curPos[1]-targetPos[1])**2)
        if dis < 30:
            print('Arrive at the destination')
            ue.sendUE4Cmd('RflyShowTextTime "Arrive at the destination" 10')
            self.isDone = 1
            self.WaitResetFlag = 0

## @class Command
#  @brief 用于发送控制命令的类。
#  @details Command类包含用于发送各种控制命令的方法，包括起飞、位置控制、速度控制等。
#  @param mav 无人机对象，用于发送控制命令。
class Command:
    ## @brief 初始化Command类。
    # - @anchor Command.__init__
    #  @param mav 无人机对象，用于发送控制命令。
    def __init__(self,mav):
        self.CID = 2
        self.mav = mav
        self.ARMFLAG = False 
        self.isDone = 0 
        self.RECORDFLAG = False
        self.LANDFLAG = False
        self.LANDFLAGTAG = False
        self.silInt = np.zeros(8).astype(int).tolist()
        self.silFloats = np.zeros(20).astype(float).tolist()
        self.INJECTFLAG = False
        self.FAULTID = 0
        self.isInitOff = 0
    
    ## @brief 发送起飞命令给无人机。
    # - @anchor Arm
    def Arm(self): 
        self.isDone = 0
        self.mav.SendMavArm(1)
        print('Armed')
        ue.sendUE4Cmd('RflyShowTextTime "Armed" 10')
        self.ARMFLAG = True
        self.isDone = 1
        self.RECORDFLAG = True
    
    ## @brief 发送解除准备命令给无人机。
    # - @anchor DisArm
    def DisArm(self): 
        self.isDone = 0
        self.mav.SendMavArm(0) 
        print('DisArmed') 
        ue.sendUE4Cmd('RflyShowTextTime "DisArmed" 10')
        self.isDone = 1
    
    ## @brief 发送四旋翼无人机位置控制命令。
    # - @anchor QuadPos
    #  @param pos 目标位置坐标。
    def QuadPos(self,pos):
        self.isDone = 0
        self.mav.SendMavArm(1)
        self.mav.SendPosNED(pos[0],pos[1],pos[2])
        print('Send Pos {}'.format(pos))
        ue.sendUE4Cmd('RflyShowTextTime "Send Pos Cmd:%.2f %.2f %.2f" 10'%(pos[0],pos[1],pos[2]))
        self.isDone = 1

    ## @brief 发送固定翼无人机位置控制命令。
    # - @anchor FixWingPos
    #  @param pos 目标位置坐标。
    def FixWingPos(self,pos): 
        self.isDone = 0
        if self.isInitOff == 0:
            self.mav.initOffboard()
            self.isInitOff = 1
        print('start init Offboard mode')
        self.mav.SendPosNED(pos[0],pos[1],pos[2])
        print('Send Pos {}'.format(pos))
        ue.sendUE4Cmd('RflyShowTextTime "Send Pos Cmd:%.2f %.2f %.2f" 10'%(pos[0],pos[1],pos[2]))
        ue.sendUE4Cmd('RflyShowTextTime "Start Init Offboard Mode" 10')
        self.isDone = 1
    
    ## @brief 发送无人船位置控制命令。
    # - @anchor USVPos
    #  @param pos 目标位置坐标。
    def USVPos(self,pos): 
        self.isDone = 0
        self.mav.SendPosNEDNoYaw(pos[0],pos[1],pos[2])
        print('Send Pos {}'.format(pos))
        ue.sendUE4Cmd('RflyShowTextTime "Send Pos Cmd:%.2f %.2f %.2f" 10'%(pos[0],pos[1],pos[2]))
        self.isDone = 1

    ## @brief 发送四旋翼无人机速度控制命令。
    # - @anchor QuadVel
    #  @param vel 目标速度向量。
    def QuadVel(self,vel): 
        self.isDone = 0
        self.mav.SendVelNED(vel[0],vel[1],vel[2])
        print('Send Vel {}'.format(vel))
        ue.sendUE4Cmd('RflyShowTextTime "Send Vel Cmd:%.2f %.2f %.2f" 10'%(vel[0],vel[1],vel[2]))
        self.isDone = 1

    ## @brief 发送无人船速度控制命令。
    # - @anchor USVVel
    #  @param vel 目标速度向量。
    def USVVel(self,vel): 
        self.isDone = 0
        self.mav.SendVelNEDNoYaw(vel[0],vel[1],vel[2])
        print('Send vel {}'.format(vel))
        ue.sendUE4Cmd('RflyShowText Send Speed Command')
        ue.sendUE4Cmd('RflyShowTextTime "Send Vel Cmd:%.2f %.2f %.2f" 10'%(vel[0],vel[1],vel[2]))
        self.isDone = 1
    
    ## @brief 设置无人船的地面速度。
    # - @anchor USVGroundSpeed
    #  @param vel 地面速度值。
    def USVGroundSpeed(self,vel): 
        self.isDone = 0
        self.mav.SendGroundSpeed(vel[0])
        print('Set GroundSpeed: {}'.format(vel[0]))
        ue.sendUE4Cmd('RflyShowTextTime "Set GroundSpeed:%.2f " 10'%(vel[0]))
        self.isDone = 1

    ## @brief 发送无人机着陆命令。
    # - @anchor UAVLand
    #  @param pos 着陆位置坐标。
    def UAVLand(self,pos): 
        self.isDone = 0
        self.LANDFLAG = True
        if self.LANDFLAGTAG == False:
            self.mav.sendMavLand(pos[0],pos[1],pos[2])
            print('Start Landing')
            ue.sendUE4Cmd('RflyShowTextTime "Start Landing:%.2f %.2f %.2f" 10'%(pos[0],pos[1],pos[2]))
            self.LANDFLAGTAG = True
        if abs(self.mav.truePosNED[2]) < 1.5:
            print('Landed')
            self.isDone = 1
            self.LANDFLAGTAG = False

    ## @brief 发送固定翼无人机起飞命令。
    # - @anchor FixWingTakeOff
    #  @param targetpos 起飞位置坐标。
    def FixWingTakeOff(self,targetpos): 
        self.isDone = 0
        self.mav.sendMavTakeOff(targetpos[0],targetpos[1],targetpos[2])
        ue.sendUE4Cmd('RflyShowTextTime "TakeOff cmd:%.2f %.2f %.2f" 10'%(targetpos[0],targetpos[1],targetpos[2]))
        print('Start TakeOff')
        ue.sendUE4Cmd('RflySetActuatorPWMs 1 500')
        self.isDone = 1

    ## @brief 设置固定翼无人机的巡航半径。
    # - @anchor FixWingSetCruiseRadius
    #  @param radius 巡航半径。
    def FixWingSetCruiseRadius(self,radius): 
        self.isDone = 0
        self.mav.SendCruiseRadius(radius[0])
        print('CruiseRadius is {}'.format(radius[0]))
        ue.sendUE4Cmd('RflyShowTextTime "Set CruiseRadius:%.2f" 10'%(radius[0]))
        self.isDone = 1

    ## @brief 注入故障参数。
    # - @anchor FaultInject
    #  @param param 故障参数ID列表。    
    def FaultInject(self,param): 
        self.isDone = 0
        self.inInts = np.array([])
        self.inFloats = np.array([])
        for i in range(len(param)):
            if param[i] >= 123450:
                self.inInts = np.append(self.inInts,param[i])
            else:
                self.inFloats = np.append(self.inFloats,param[i])
        
        for i in range(len(self.inInts)):
            self.silInt[i] = self.inInts[i].astype(int)
        for i in range(len(self.inFloats)):
            self.silFloats[i] = self.inFloats[i].astype(np.double)

        if self.silInt[0] == 123450 or self.silInt[0] == 123451:
            ue.sendUE4Cmd('RflySetActuatorPWMsExt 1 1')

        print('Start Inject Fault')
        ue.sendUE4Cmd('RflyShowTextTime "Fault Params: %.2f %.2f %.2f" 10'%(self.silFloats[0],self.silFloats[1],self.silFloats[2]))
        ue.sendUE4Cmd('RflyShowTextTime "Start Inject %d Fault" 10'%(self.silInt[0]))
        self.mav.sendSILIntFloat(self.silInt,self.silFloats)
        self.mav.SendMavCmdLong(183,999,999,999,999,999,999,999)
        self.FAULTID = self.silInt[0]
        self.isDone = 1
        self.INJECTFLAG = True

## @class CmdCtrl
#  @brief 用于控制命令序列的类。
#  @details CmdCtrl类用于根据不同的框架获取对应的命令序列和等待序列。
#  @param mav 无人机对象，用于发送控制命令。
#  @param frame 指定当前的框架类型。
class CmdCtrl:
    ## @brief 初始化CmdCtrl类。
    # - @anchor CmdCtrl.__init__
    #  @param mav 无人机对象，用于发送控制命令。
    #  @param frame 指定当前的框架类型。
    def __init__(self,mav,frame):
        self.mav = mav
        self.frame = frame
        self.CID = {
        '1':Sleep(mav),
        '2':Command(mav)
        }
        self.CID1 = self.CID['1']
        self.CID2 = self.CID['2']
        self.FID = 0
    
    ## @brief 获取等待序列的方法。
    # - @anchor GetWaitseq
    #  @return 返回等待序列的字典。
    def GetWaitseq(self):
        Waitseq = {
                '1':self.CID1.Wait,
                '2':self.CID1.WaitResetForFixwing
        }
        return Waitseq

    ## @brief 根据不同框架获取命令序列的方法。
    # - @anchor GetCmdseq
    #  @return 返回命令序列的字典。
    def GetCmdseq(self):
        if self.frame == 1: 
            Cmdseq = {
                '1':self.CID2.Arm,
                '2':self.CID2.DisArm,
                '3':self.CID2.QuadPos,
                '4':self.CID2.QuadVel,
                '5':self.CID2.UAVLand,
                '6':self.CID2.FaultInject
            }
        elif self.frame == 2: 
            Cmdseq = {
                '1':self.CID2.Arm,
                '2':self.CID2.DisArm,
                '3':self.CID2.FixWingTakeOff,
                '4':self.CID2.FixWingPos,
                '5':self.CID2.UAVLand,
                '6':self.CID2.FaultInject
            }
        elif self.frame == 3:
            Cmdseq = {
                '1':self.CID2.Arm,
                '2':self.CID2.DisArm,
                '3':self.CID2.USVPos,
                '4':self.CID2.USVVel,
                '5':self.CID2.USVGroundSpeed,
                '6':self.CID2.FaultInject
            }
        return Cmdseq
    
    ## @brief 根据命令CID获取命令序列或等待序列的方法。
    # - @anchor FIDPro
    #  @param cmdCID 命令序列或等待序列的标识。
    #  @return 返回对应的序列。
    def FIDPro(self,cmdCID):
        if cmdCID == '1':
            self.FID = CmdCtrl.GetWaitseq(self)
        elif cmdCID == '2':
            self.FID = CmdCtrl.GetCmdseq(self)
        return self.FID
