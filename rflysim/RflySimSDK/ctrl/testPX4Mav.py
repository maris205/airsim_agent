import PX4MavCtrlV4 as PX4MavCtrl
import time
import numpy as np
import threading


class PID:
    """
    本控制器适用于调用时间间隔在一个常值附近的情形
    在控制器中不计算时间间隔，通过PID参数进行调整
    """

    def __init__(self, p=0.0, i=0.0, d=0.0):
        self.p = p
        self.i = i
        self.d = d
        self.sumErr = 0
        self.sumLimit = 100
        self.lastErr = 0

    def pid(self, err):
        rec = self.p * err
        self.sumErr = self.sumErr + err
        if self.sumErr > self.sumLimit:
            self.sumErr = self.sumLimit
        elif self.sumErr < -self.sumLimit:
            self.sumErr = -self.sumLimit
        rec = rec + self.sumErr * self.i
        rec = rec + (self.sumErr - self.lastErr) * self.d
        self.lastErr = err
        return rec

    def reset(self):
        self.sumErr = 0


def takeoff_full_vel():
    """
    使用完整的offboard接口控制飞机起飞
    bat脚本以mavlink_full模式启动
    """
    mav = PX4MavCtrl.PX4MavCtrler(Com='udp')
    time.sleep(2)
    mav.InitMavLoop(2)
    time.sleep(2)
    # 进入Offboard模式
    mav.initOffboard()

    mask = PX4MavCtrl.PosTypeMask()
    mav.SendOffAll(mask, vel=[0, 0, -1])
    time.sleep(10)
    mav.stopRun()


def takeoff_full_acc():
    """
    使用完整的offboard接口控制飞机起飞
    bat脚本以mavlink_full模式启动
    Z轴方向设置加速度起不了飞，所以设置速度
    """
    mav = PX4MavCtrl.PX4MavCtrler(Com='udp')
    time.sleep(2)
    mav.InitMavLoop(2)
    time.sleep(2)
    # 进入Offboard模式
    mav.initOffboard()

    mask = PX4MavCtrl.PosTypeMask(ignore_all=True)
    mask.enVel_z = True
    mask.enAcc_x = True
    mask.enAcc_y = True
    mask.enYawRate = True
    mav.SendOffAll(mask, vel=[0, 0, -1])
    time.sleep(10)
    mav.stopRun()


def takeoff_full_att():
    """
    使用完整的offboard接口控制飞机起飞
    bat脚本以mavlink_full模式启动
    """
    mav = PX4MavCtrl.PX4MavCtrler(Com='udp')
    time.sleep(2)
    mav.InitMavLoop(2)
    time.sleep(2)
    mav.initOffboard()
    mask = PX4MavCtrl.AttTypeMask()
    mav.SendAttAll(type_mask=mask, thrust=0.9)
    time.sleep(10)
    mav.stopRun()


def takeoff(mode):
    """
    旋翼机位置控制起飞
    """
    if mode >= 6:
        mav_mode = 'redis'
        mode = mode - 6
    else:
        mav_mode = 'udp'
    mav = PX4MavCtrl.PX4MavCtrler(Com=mav_mode)
    time.sleep(2)
    mav.InitMavLoop(mode)
    time.sleep(2)
    # 进入Offboard模式
    mav.initOffboard()
    time.sleep(2)
    mav.SendPosNED(0, 0, -10, 0)
    time.sleep(15)
    mav.endOffboard()
    time.sleep(2)
    mav.stopRun()


def takeoff_udp_full():
    """
    运行C:/PX4PSP/RflySimAPIs/SITLRun.bat
    """
    takeoff(0)


def takeoff_udp_simple():
    """
    需启动udp_simple对应的CopterSim和RflySim3D
    """
    takeoff(1)


def takeoff_mavlink_full():
    """
    运行C:/PX4PSP/RflySimAPIs/SITLRunMAVLink.bat
    """
    takeoff(2)


def takeoff_mavlink_simple():
    """
    运行C:/PX4PSP/RflySimAPIs/SITLRun.bat
    """
    takeoff(3)


def takeoff_redis_full():
    """
    需要启动redis_full对应的CopterSim和RflySim3D
    """
    takeoff(6)


def takeoff_redis_simple():
    """
    需要启动redis_simple对应的CopterSim和RflySim3D
    """
    takeoff(7)


def takeoff_vel(mode):
    """
    旋翼机速度控制起飞
    """
    if mode >= 6:
        mav_mode = 'redis'
        mode = mode - 6
    else:
        mav_mode = 'udp'
    mav = PX4MavCtrl.PX4MavCtrler(Com=mav_mode)
    time.sleep(2)
    mav.InitMavLoop(mode)
    time.sleep(2)
    # 进入Offboard模式
    mav.initOffboard()
    time.sleep(2)
    pidPos = PID(p=0.5)
    posTarget = np.array([0, 0, -10])
    while True:
        posCurrent = mav.uavPosNED
        vel_z = pidPos.pid(posTarget[2] - posCurrent[2])
        mav.SendVelNED(0, 0, vel_z, 0)
        if np.linalg.norm(posTarget - posCurrent) < 1:
            mav.SendVelNED(0, 0, 0, 0)
            break
    time.sleep(15)
    mav.endOffboard()
    time.sleep(2)
    mav.stopRun()


def takeoff_vel_mavlink_full():
    takeoff_vel(2)


def takeoff_vel_redis_full():
    takeoff_vel(6)


def takeoff_vel_redis_simple():
    takeoff_vel(7)


class TestSynModel:
    """
    四旋翼综合模型基本接口测试
    """

    def __init__(self, num=1, ip='127.0.0.1', mode=7):
        """
        :param num: 载具数量
        :param ip: 支持配置redis ip，可在一台电脑上控制另一台电脑的模型
        :param mode: 通信模式
        """
        self.vehicleNum = num
        self.vehicles = list()
        self.iniPoses = list()
        self.interval = 2
        self.height = -20
        self.ip = ip
        self.mode = mode
        if mode >= 6:
            self.mavMode = 'redis'
        else:
            self.mavMode = 'udp'

    def init_poses(self):
        sqrtNum = 1
        while True:
            if sqrtNum * sqrtNum >= self.vehicleNum:
                break
            sqrtNum = sqrtNum + 1

        for i in range(0, sqrtNum):
            for j in range(0, sqrtNum):
                self.iniPoses.append((i * self.interval, j * self.interval))
        self.iniPoses = self.iniPoses[:self.vehicleNum]

    def loop(self, *args):
        args[0].InitMavLoop(self.mode)
        time.sleep(2)
        args[0].initOffboard()
        time.sleep(2)

        print("起飞")
        target_pos = np.array([args[1][0], args[1][1], self.height])
        args[0].TakeoffSyn(self.height)
        time.sleep(10)
        posCurrent = args[0].truePosNED
        print(posCurrent)

        print("加速度控制")
        start_time = time.time()
        acc = np.zeros(3)
        acc[0] = 3
        while True:
            args[0].SendSynAcc(acc[0], acc[1], acc[2])
            time_interval = time.time() - start_time
            if 20 < time_interval < 40:
                acc[0] = -3
            if time_interval > 40:
                break

        print("调整为偏航180°")
        args[0].SendSynPos(target_pos[0], target_pos[1], target_pos[2], 3.14)
        time.sleep(5)

        print("匀速1rad/s转圈")
        target_vel = np.array([0, 0, 0])
        args[0].SendSynVel(target_vel[0], target_vel[1], target_vel[2], 1)
        time.sleep(5)

        print("速度控制向前飞")
        target_vel[0] = 3
        args[0].SendSynVel(target_vel[0], target_vel[1], target_vel[2], 0)
        time.sleep(15)

        print("姿态控制")
        args[0].SendSynAtt(0, 0.2, 0)
        time.sleep(20)

        print("返航")
        args[0].ReturnHomeSyn(self.height - 10)
        time.sleep(20)

        print("着陆")
        args[0].LandSyn()

        while True:
            posCurrent = args[0].truePosNED
            if abs(posCurrent[2]) < 8.5:
                break
            print(posCurrent)
            time.sleep(1)

        # 结束程序
        args[0].stopRun()

    def run(self):
        self.init_poses()
        for i in range(self.vehicleNum):
            self.vehicles = self.vehicles + [
                PX4MavCtrl.PX4MavCtrler(ID=i + 1, Com=self.mavMode, simulinkDLL=True)]

        for (vehicle, iniPos) in zip(self.vehicles, self.iniPoses):
            task = threading.Thread(target=self.loop, args=(vehicle, iniPos))
            task.start()


def takeoff_syn_redis_simple():
    ctrl = TestSynModel()
    ctrl.run()


def arm_test():
    """
    测试真机是否能通过udp连上。
    分析连通问题方法
    1) 检查ip是否能ping通，不能ping通极有可能是a)板卡是坏的，b)静态ip问题。
    2) 避免offboard干扰，传感器数据不足时，不能进offboard，因此最好不要用混杂了设置offboard的代码检查连通性。
    3) 直接用运行模式运行（不要用debug），pymavlink的调试功能不完善，用debug控制真机会出现一些意想不到的错误。
    4) 打开qgc查看是否能连接，能连上但本例还失败，大概率是路由端口没对上。
    """
    mav = PX4MavCtrl.PX4MavCtrler(1, '10.42.0.1', 'direct:15552')
    mav.InitMavLoop(2)
    print("arm")
    mav.SendMavArm(True)
    time.sleep(5)
    print("disarm")
    mav.SendMavArm(False)
    mav.stopRun()


if __name__ == "__main__":
    print("开始测试")
    takeoff_full_att()
