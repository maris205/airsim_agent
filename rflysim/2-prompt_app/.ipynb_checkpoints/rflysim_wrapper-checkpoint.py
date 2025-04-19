#先运行软件在环仿真环境 SITLRun.bat
import time
import math
import sys
import PX4MavCtrlV4 as PX4MavCtrl #无人机控制
import UE4CtrlAPI

objects_dict = {
    "turbine1": "fdj005",
    "turbine2": "fdj012",
    "solarpanels": "solar_panels_roof_texture_solar_panels_roof_45",
    "crowd": "SK_BioWorker9",
    "car": "dfz82",
}


class RflySimWrapper:
    def __init__(self):
        #ue对象
        self.ue = UE4CtrlAPI.UE4CtrlAPI()
        time.sleep(1)
        
        #设定地图
        self.ue.sendUE4Cmd('RflyChangeMapbyName cj')
        time.sleep(1)

        #启动UE目标位置监听
        self.ue.initUE4MsgRec()
        time.sleep(3)
        
        
        #Create a new MAVLink communication instance, UDP sending port (CopterSim’s receving port) is 20100
        self.mav = PX4MavCtrl.PX4MavCtrler(1)
        time.sleep(1)

        ###############高精度模式################################
        # #Turn on MAVLink to monitor CopterSim data and update it in real time. 
        # self.mav.InitMavLoop()
        # time.sleep(1)
        
        # #Turn on Offboard mode
        # self.mav.initOffboard()
        # time.sleep(1)
        
        # #Send arm command to arm the drone,解锁
        # self.mav.SendMavArm(True) 

        ###############质点模式################################
        #简单模式,只需要启动rflysim3d
        self.mav.initPointMassModel(-1,[-115,1,0])


    def takeoff(self):
        """
        takeoff the drone
        """
        self.mav.SendPosNED(0, 0, -3, 0) 

    def land(self):
        """
        land the drone
        """
        self.mav.SendVelNED(0, 0, 0.2, 0) 
        time.sleep(1)


    def get_drone_position(self):
        """
        get the current position of the drone
        :return: position, the current position of the drone
        """
        pose = self.mav.uavPosNED
        return pose

    def fly_to(self, point):
        """
        fly the drone to a specific point
        :param point: the target point
        """
        if point[2] > 0:
            self.mav.SendPosNED(point[0], point[1],-point[2], 0)
        else:
            self.mav.SendPosNED(point[0], point[1],point[2], 0)

        time.sleep(1)

        
    def fly_path(self, points):
        """
        fly the drone along a specific path
        :param points: the path
        """
        for point in points:
            self.fly_to(point)
            time.sleep(1)
            

    def set_yaw(self, yaw_degree):
        """
        set the yaw angle of the drone，度数
        """
        point = self.get_drone_position()
        yaw_angel =  math.radians(yaw_degree) #转成弧度
        self.mav.SendPosNED(point[0], point[1],point[2], yaw_angel)

    def get_yaw(self):
        """
        get the yaw degree of the drone
        :return: yaw_degree, the yaw angle of the drone in degree
        """
        yaw = self.mav.yaw #弧度
        yaw_degree = math.degrees(yaw)
        return yaw_degree # return the yaw angle in degree

    def get_position(self, object_name):
        """
        get the position of a specific object
        :param object_name: the name of the object
        :return: position, the position of the object
        """
        # 注：飞机位置发生改变时，数据会立刻传出；飞机位置未更新，则1s发出一次
        ue.reqCamCoptObj(2,object_name)#发送请求到RflySim3D，返回物体数据，名字为Landscape_1的物体
        time.sleep(2)

        Obj = ue.getCamCoptObj(2,object_name) #获取目标1号飞机的结构体引用
        
        if isinstance(Obj,UE4CtrlAPI.ObjReqData):
            pose = Obj.PosUE
        else:
            pose = (0, 0, 0)

        return [pose[0], pose[1], pose[2]]



if __name__ == "__main__":
    rw = RflySimWrapper()
    rw.takeoff()
    rw.fly_to([20, 0, -10])
    rw.land()
    print("done")
