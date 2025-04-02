import numpy as np
import time
from .PX4MavCtrlV4 import PX4MavCtrler, RflySimCP
## @file
#  
#  @anchor api接口库文件

class Ctrl(PX4MavCtrler):
    def __init__(self, copter_id=1, ip='127.0.0.1', com='udp', port=0, simulink_dll=False):
        super().__init__(ID=copter_id, ip=ip, Com=com, port=port, simulinkDLL=simulink_dll)
        self.rflysim_cmd = RflySimCP.CmdBase + RflySimCP.CmdArmed

    def init_loop(self, ct_mode=2, offboard=False):
        self.InitMavLoop(ct_mode)
        time.sleep(5)
        if offboard:
            self.initOffboard()

    def is_full_send(self):
        return self.UDPMode == 0 or self.UDPMode == 2 or self.UDPMode == 3 or self.UDPMode == 6

    def takeoff(self, height=0, pos_x=0, pos_y=0):
        """
        该高度为NED坐标系下的高度，PX4版本的固定翼支持指定起飞后位置，综合模型只支持指定高度
        """
        if self.simulinkDLL:
            self.TakeoffSyn(height)
        else:
            self.sendMavTakeOffLocal(pos_x, pos_y, height)

    def return_home(self, height=0):
        """
        水平位置到达home点，高度保持不变
        """
        if self.simulinkDLL:
            self.ReturnHomeSyn(height)

    def land(self, height=0):
        """
        高度降落到home点高度
        """
        if self.simulinkDLL:
            self.LandSyn(height)

    def send_pos_ned(self, pos, yaw):
        """
        PX4版本时每架飞机相对于起飞点，综合模型时相对UE中心
        出现这种差异是综合模型使用True pos控制，而PX4使用滤波后位置控制
        :param pos: NED坐标系下x,y,z方向位置，单位m
        :param yaw: 偏航角，单位rad
        """
        if self.simulinkDLL:
            self.SendSynPos(pos[0], pos[1], pos[2], yaw)
        else:
            self.SendPosNED(pos[0], pos[1], pos[2], yaw)

    def send_pos_speed_fw(self, pos, speed):
        """
        固定翼综合模型同时支持位置和速度控制
        """
        if self.simulinkDLL:
            self.SendPosSpeedFWSyn(pos[0], pos[1], pos[2], speed)

    def send_pos_global(self, pos, yaw):
        if self.simulinkDLL:
            pass
        else:
            self.SendPosGlobal(pos[0], pos[1], pos[2], yaw, 1)

    def send_vel_ned(self, vel, yaw_rate):
        if self.simulinkDLL:
            self.SendSynVel(vel[0], vel[1], vel[2], yaw_rate)
        else:
            self.SendVelNED(vel[0], vel[1], vel[2], yaw_rate)

    def send_vel_body(self, vel, yaw_rate):
        if self.simulinkDLL:
            pass
        else:
            self.SendVelFRD(vel[0], vel[1], vel[2], yaw_rate)

    def send_acc(self, acc):
        """
        仅适用于旋翼无人机
        :param acc: NED坐标系下期望加速度，x， y方向响应非常灵敏，z方向为了定高响应缓慢
        """
        if self.simulinkDLL:
            self.SendSynAcc(acc[0], acc[1], acc[2])

    def send_att_thrust(self, att, thrust):
        """
        :param att: 欧拉角[滚转角、俯仰角、偏航角]
        :param thrust: 油门
        """
        if self.simulinkDLL:
            self.SendSynAttThrust(att[0], att[1], att[2], thrust)

    def send_att(self, att):
        """
        :param att: 欧拉角[滚转角、俯仰角、偏航角]
        """
        if self.simulinkDLL:
            self.SendSynAtt(att[0], att[1], att[2])

    def send_cruise_speed_fw(self, speed):
        """
        仅支持固定翼，对应QGC中FW_AIRSPD_TRIM
        速度限制10-20m/s，默认15m/s
        速度限制定义在src/modules/fw_pos_control_l1/fw_pos_control_l1_params.c
        :param speed: 水平速度
        """
        self.SendCruiseSpeed(speed)

    def send_cruise_radius_fw(self, radius):
        """
        仅支持固定翼，对应QGC中NAV_LOITER_RAD
        :param radius:盘旋半径，25m-1000m，正常误差可达10-20m，设置超出范围的值在QGC中会显示修改，但实际不会超过范围
        """
        self.SendCruiseRadius(radius)

    def get_vehicle_id(self):
        """
        :return: 返回载具ID，默认从1开始编号
        """
        return self.CopterID

    def get_time_s(self):
        return self.uavTimeStmp

    def get_pos_ned(self):
        """
        :return: PX4返回局部NED坐标系下的位置，每一架飞机都以起飞点为原点；综合模型返回全局NED(相对UE中心) TRUEpos
        """
        if self.simulinkDLL:
            return np.array(self.truePosNED)
        return np.array(self.uavPosNED)

    def get_pos_ned_global(self):
        """
        :return: 返回全局NED(相对UE中心)位置
        """
        if self.simulinkDLL:
            return np.array(self.truePosNED)
        return np.array(self.uavGlobalPos)

    def get_pos_global(self):
        """
        :return: WGS84坐标系下位置,纬经高
        """
        return np.array(self.uavPosGPS)

    def get_home_pos(self):
        """
        :return: WGS84坐标系下Home点位置，纬经高
        """
        return np.array(self.uavPosGPSHome)

    def get_vel_ned(self):
        if self.simulinkDLL:
            return np.array(self.trueVelNED)
        return np.array(self.uavVelNED)

    def get_acc(self):
        """
        目前仅综合模型支持返回加速度
        :return: 载体坐标系中加速度
        """
        if self.simulinkDLL:
            return np.array(self.trueAccB)
        return np.array(self.uavAccB)

    def get_euler(self):
        if self.simulinkDLL:
            return np.array(self.trueAngEular)
        return np.array(self.uavAngEular)

    def get_angular_rate(self):
        if self.simulinkDLL:
            return np.array(self.trueAngRate)
        return np.array(self.uavAngRate)

    def get_quaternion(self):
        """
        目前仅综合模型支持返回四元数
        :return: 四元数描述的姿态
        """
        if self.simulinkDLL:
            return np.array(self.trueAngQuatern)
        return np.array(self.uavAngQuatern)

    def get_actuator_speed(self):
        """
        :return: 电机转速，转/min
        """
        if self.simulinkDLL:
            return np.array(self.trueMotorRPMS)
        return np.array(self.uavMotorRPMS)

    def get_max_vel_xy(self):
        """
        :return: 最大水平速度，m/s
        """
        return self.maxVelXy

    def get_max_vel_z(self):
        """
        :return: 最大竖直速度，m/s
        """
        return self.maxVelZ

    def get_max_acc_xy(self):
        """
        :return: 最大水平加速度，m/s2
        """
        return self.maxAccXy

    def get_max_acc_z(self):
        """
        :return: 最大竖直，m/s2
        """
        return self.maxAccZ


class CtMode:
    """
    Redis模式仅企业版支持
    """
    UDP_Full = 0
    UDP_Simple = 1
    MAVLink_Full = 2
    MAVLink_Simple = 3
    MAVLink_NoSend = 4
    MAVLink_NoGPS = 5
    Mavlink_Vision = 6
    Redis_Full = 7
    Redis_Simple = 8

    def __init__(self):
        self.mode = CtMode.MAVLink_Full

    def set(self, mode):
        try:
            md = int(mode)
            if md < CtMode.UDP_Full or md > CtMode.Redis_Simple:
                print("ConnectionMode should in [{}, {}], but actual is {}"
                      .format(CtMode.UDP_Full, CtMode.Redis_Simple, md))
                return
        except Exception as e:
            print("ConnectionMode is invalid, {}".format(e))
        self.mode = mode

    def is_udp_full(self):
        return self.mode is CtMode.UDP_Full

    def is_udp_simple(self):
        return self.mode is CtMode.UDP_Simple

    def is_mav_full(self):
        return self.mode is CtMode.MAVLink_Full

    def is_mav_simple(self):
        return self.mode is CtMode.MAVLink_Simple

    def is_mav_no_send(self):
        return self.mode is CtMode.MAVLink_NoSend

    def is_mav_no_gps(self):
        return self.mode is CtMode.MAVLink_NoGPS

    def is_redis_full(self):
        return self.mode is CtMode.Redis_Full

    def is_redis_simple(self):
        return self.mode is CtMode.Redis_Simple

    def is_udp(self):
        return self.mode is CtMode.UDP_Full or \
               self.mode is CtMode.UDP_Simple

    def is_mav(self):
        return self.mode is CtMode.MAVLink_Full or \
               self.mode is CtMode.MAVLink_Simple

    def is_redis(self):
        return self.mode is CtMode.Redis_Full or \
               self.mode is CtMode.Redis_Simple

    def is_full(self):
        return self.mode is CtMode.UDP_Full or \
               self.mode is CtMode.MAVLink_Full or \
               self.mode is CtMode.Redis_Full


class PID:
    """
    本控制器适用于调用时间间隔在一个常值附近的情形
    在控制器中不计算时间间隔，通过PID参数进行调整
    """

    def __init__(self, p=0.0, i=0.0, d=0.0):
        self.p = p
        self.i = i
        self.d = d
        self.sum_err = 0
        self.sum_limit = 100
        self.last_err = 0

    def pid(self, err):
        rec = self.p * err
        self.sum_err = self.sum_err + err
        if self.sum_err > self.sum_limit:
            self.sum_err = self.sum_limit
        elif self.sum_err < -self.sum_limit:
            self.sum_err = -self.sum_limit
        rec = rec + self.sum_err * self.i
        rec = rec + (self.sum_err - self.last_err) * self.d
        self.last_err = err
        return rec

    def reset(self):
        self.sum_err = 0
