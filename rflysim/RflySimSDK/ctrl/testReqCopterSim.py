import ReqCopterSim
import socket
import PX4MavCtrlV4 as PX4MavCtrl
import UE4CtrlAPI
import VisionCaptureApi

# 注意，测试时，可以直接使用SITLRun.bat来测试

# 创建一个CopterSim状态获取实例，并监听2s钟，获取当前所有CopterSim列表数据
req = ReqCopterSim.ReqCopterSim()

# 获取ID和IP列表
IPList=req.getSimIpList()
print(IPList)

# 获取指定ID的电脑IP地址
IP = req.getSimIpID(1)
print(IP)


# 下面展示，如何使用本接口，不需要知道目标电脑IP的情况下，能够连上远程电脑的CopterSim
CopterID=1  # 几乎仿真的飞机ID


## 获取目标电脑IP，并且配置CopterSim回传数据到本电脑
# 获取到指定CopterID的CopterSim所在电脑的IP
TargetIP = req.getSimIpID(CopterID)

# 请求目标CopterSim将数据返回到本电脑
# 通过本接口，可以不用再去bat脚本里面填写IP地址了
req.sendReSimIP(CopterID)

# 通过本接口，可以强制修改CopterSim的UDP_Mode，这里只有要时，才发送
new_UDP_mode=2
req.sendReSimUdpMode(CopterID,new_UDP_mode) # 强制申请CopterSIM转换为MAVLink_Full通信模式


## 开始利用获取的IP，进行正常的初始化与操作
# 创建mav，同时指定ID和IP（也适用于本机的例子）
mav = PX4MavCtrl.PX4MavCtrler(CopterID,TargetIP)

# 刚才修改了UDP_mode为2，这里使用2模式进行连接CopterSim
mav.InitMavLoop(new_UDP_mode)

# 创建ue，使得支持远程的电脑
ue = UE4CtrlAPI.UE4CtrlAPI(TargetIP)

# 创建vis，使得支持远程的电脑
vis = VisionCaptureApi.VisionCaptureApi(TargetIP)


## TargetIP还可以用于其他的一些情形

# 例如，请求发送IMU数据到本电脑
vis.sendImuReqCopterSim(CopterID,TargetIP)

