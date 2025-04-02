import socket
import threading
import time
import struct
import sys
import copy
import os
import cv2
import numpy as np
## @file
#  这是一个与RflySim3D进行交互的模块。
#  @anchor UE4CtrlAPI接口库文件
#  对应例程链接见
#  @ref md_ue_2md_2UE4CtrlAPI



## @class PX4SILIntFloat
#  @brief PX4SILIntFloat结构体类.
#  @details PX4SILIntFloat 类包含将在 CopterSim 模拟中使用的整数和浮点数数据数组。它支持使用提供的值列表进行初始化。
class PX4SILIntFloat:
    """
    输出到CopterSim DLL模型的SILints和SILFloats数据
    struct PX4SILIntFloat{
        int checksum;//1234567897
        int CopterID;
        int inSILInts[8];
        float inSILFLoats[20];
    };
    struct.pack 10i20f
    """
            
    ## @brief PX4SILIntFloat的构造函数
    # @param 初始化一个包含校验和、CopterID、inSILInts 和 inSILFloats 初始值的列表。如果未提供，则默认为零值。   
    # @return 返回一个PX4SILIntFloat类型的实例。
    def __init__(self, iv=None):

        if iv != None:
            ##  @var PX4SILIntFloat.checksum
            #   @brief 这是校验和的值，初始化为 1234567897。
            self.checksum = iv[0]
            ## @var PX4SILIntFloat.CopterID
            #  @brief 用于识别直升机的标识符。
            self.CopterID = iv[1]
            ## @var PX4SILIntFloat.inSILInts
            #  @brief 一个包含 8 个整数的数组，用于 SIL 数据。
            self.inSILInts = iv[2:10]
            ## @var PX4SILIntFloat.inSILFLoats
            #  @brief 一个包含 20 个浮点数的数组，也用于 SIL 数据。
            self.inSILFLoats = iv[10:30]

            return
        self.checksum = 0
        self.CopterID = 0
        self.inSILInts = [0, 0, 0, 0, 0, 0, 0, 0]
        self.inSILFLoats = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    # def __init__(self, iv):
    #     self.checksum = iv[0]
    #     self.CopterID = iv[1]
    #     self.inSILInts = iv[2:10]
    #     self.inSILFLoats = iv[10:30]



## @class reqVeCrashData
#  @brief reqVeCrashData结构体类.
#  @details
class reqVeCrashData:
    '''
    struct reqVeCrashData {
    	int checksum; //数据包校验码1234567897
    	int copterID; //当前飞机的ID号
    	int vehicleType; //当前飞机的样式
    	int CrashType;//碰撞物体类型，-2表示地面，-1表示场景静态物体，0表示无碰撞，1以上表示被碰飞机的ID号
    	double runnedTime; //当前飞机的时间戳
    	float VelE[3]; // 当前飞机的速度
    	float PosE[3]; //当前飞机的位置
    	float CrashPos[3];//碰撞点的坐标
    	float targetPos[3];//被碰物体的中心坐标
    	float AngEuler[3]; //当前飞机的欧拉角
    	float MotorRPMS[8]; //当前飞机的电机转速
        float ray[6]; //飞机的前后左右上下扫描线
    	char CrashedName[16];//被碰物体的名字
     } 4i1d29f20s
    '''
    def __init__(self, iv=None):
        if iv != None:
            self.CopyData(iv)
            return
        self.checksum = 1234567897
        self.copterID = 0
        self.vehicleType = 0
        self.CrashType = 0
        self.runnedTime = 0
        self.VelE = [0, 0, 0]
        self.PosE = [0, 0, 0]
        self.CrashPos = [0, 0, 0]
        self.targetPos = [0, 0, 0]
        self.AngEuler = [0, 0, 0]
        self.MotorRPMS = [0, 0, 0, 0, 0, 0, 0, 0]
        self.ray = [0, 0, 0, 0, 0, 0]
        self.CrashedName = ""
        self.hasUpdate = True

    # def __init__(self, iv):
    #     self.CopyData(iv)

    def CopyData(self, iv):
        self.checksum = iv[0]
        self.copterID = iv[1]
        self.vehicleType = iv[2]
        self.CrashType = iv[3]
        self.runnedTime = iv[4]
        self.VelE = iv[5:8]
        self.PosE = iv[8:11]
        self.CrashPos = iv[11:14]
        self.targetPos = iv[14:17]
        self.AngEuler = iv[17:20]
        self.MotorRPMS = iv[20:28]
        self.ray = iv[28:34]
        self.CrashedName = iv[34].decode("UTF-8")
        self.CrashedName = self.CrashedName.strip(b"\x00".decode())
        self.hasUpdate = True



## @class CoptReqData
#  @brief CoptReqData结构体类.
#  @details
class CoptReqData:  # 长度80, 2i16f1d
    '''
    struct CoptReqData { //80
    	int checksum = 0; //1234567891作为校验
    	int CopterID;//飞机ID
    	float PosUE[3]; //物体中心位置（人为三维建模时指定，姿态坐标轴，不一定在几何中心）
    	float angEuler[3];//物体欧拉角
    	float angQuat[4];//物体四元数
    	float boxOrigin[3];//物体几何中心坐标
    	float BoxExtent[3];//物体外框长宽高的一半
    	double timestmp;//时间戳
    };
    '''
    def __init__(self, iv=None):
        if iv != None:
            self.CopyData(iv)
            return
        self.checksum = 0  # 1234567891作为校验
        self.CopterID = 0  # 飞机ID
        self.PosUE = [0, 0, 0]
        self.angEuler = [0, 0, 0]
        self.angQuat = [0, 0, 0, 0]
        self.boxOrigin = [0, 0, 0]
        self.BoxExtent = [0, 0, 0]
        self.timestmp = 0
        self.hasUpdate = True

    # def __init__(self, iv):
    #     self.CopyData(iv)

    def CopyData(self, iv):
        self.checksum = iv[0]
        self.CopterID = iv[1]
        self.PosUE = iv[2:5]
        self.angEuler = iv[5:8]
        self.angQuat = iv[8:12]
        self.boxOrigin = iv[12:15]
        self.BoxExtent = iv[15:18]
        self.timestmp = iv[18]
        self.hasUpdate = True

    def CopyDataOld(self, iv):
        self.checksum = iv[0]
        self.CopterID = iv[1]
        self.PosUE = iv[2:5]
        self.angEuler = iv[5:8]
        self.boxOrigin = iv[8:11]
        self.BoxExtent = iv[11:14]
        self.timestmp = iv[14]
        self.hasUpdate = True



## @class ObjReqData
#  @brief ObjReqData结构体类.
#  @details
class ObjReqData:  # 长度112, 2i16f1d32s
    '''
    struct ObjReqData { //112
    	int checksum = 0; //1234567891作为校验
    	int seqID = 0;
    	float PosUE[3]; //物体中心位置（人为三维建模时指定，姿态坐标轴，不一定在几何中心）
    	float angEuler[3];//物体欧拉角
    	float angQuat[4];//物体四元数
    	float boxOrigin[3];//物体几何中心坐标
    	float BoxExtent[3];//物体外框长宽高的一半
    	double timestmp;//时间戳
    	char ObjName[32] = { 0 };//碰物体的名字
    };
    '''
    def __init__(self, iv=None):
        if iv != None:
            self.CopyData(iv)
            return
        self.checksum = 0  # 1234567891作为校验
        self.seqID = 0
        self.PosUE = [0, 0, 0]
        self.angEuler = [0, 0, 0]
        self.angQuat = [0, 0, 0, 0]
        self.boxOrigin = [0, 0, 0]
        self.BoxExtent = [0, 0, 0]
        self.timestmp = 0
        self.ObjName = ""
        self.hasUpdate = True

    # def __init__(self, iv):
    #     self.CopyData(iv)

    def CopyData(self, iv):
        self.checksum = iv[0]
        self.seqID = iv[1]
        self.PosUE = iv[2:5]
        self.angEuler = iv[5:8]
        self.angQuat = iv[8:12]
        self.boxOrigin = iv[12:15]
        self.BoxExtent = iv[15:18]
        self.timestmp = iv[18]
        self.ObjName = iv[19].decode("UTF-8")
        self.ObjName = self.ObjName.strip(b"\x00".decode())
        self.hasUpdate = True

    def CopyDataOld(self, iv):
        self.checksum = iv[0]
        self.seqID = iv[1]
        self.PosUE = iv[2:5]
        self.angEuler = iv[5:8]
        self.boxOrigin = iv[8:11]
        self.BoxExtent = iv[11:14]
        self.timestmp = iv[14]
        self.ObjName = iv[15].decode("UTF-8")
        self.ObjName = self.ObjName.strip(b"\x00".decode())
        self.hasUpdate = True

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
    def __init__(self, iv = None):
        if iv != None:
            self.CopyData(iv)
            return
        self.checksum = 0
        self.SeqID = 0
        self.bitmask = 0
        self.TypeID = 0
        self.TargetCopter = 0
        self.TargetMountType = 0
        self.DataWidth = 0
        self.DataHeight = 0
        self.DataCheckFreq = 0
        self.SendProtocol = [0] * 8
        self.CameraFOV = 0
        self.EularOrQuat = 0
        self.SensorAngQuat = [0, 0, 0, 0]
        self.SensorPosXYZ = [0, 0, 0]
        self.SensorAngEular = [0, 0, 0]
        self.otherParams = [0] * 16
    def CopyData(self, iv):
        self.checksum = iv[0]
        self.SeqID = iv[1]
        self.bitmask = iv[2]
        self.TypeID = iv[3]
        self.TargetCopter = iv[4]
        self.TargetMountType = iv[5]
        self.DataWidth = iv[6]
        self.DataHeight = iv[7]
        self.DataCheckFreq = iv[8]
        self.SendProtocol = iv[9:17]
        self.CameraFOV = iv[17]
        self.SensorPosXYZ = iv[18:21]
        self.EularOrQuat = iv[21]
        self.SensorAngEular = iv[22:25]
        self.SensorAngQuat = iv[25:29]
        self.otherParams = iv[29:45]



## @class CameraData
#  @brief CameraData结构体类.
#  @details
class CameraData:  # 长度72, 5i11f1d
    '''
    struct CameraData { //72
        int checksum = 0;//1234567891
        int SeqID; //相机序号
        int TypeID;//相机类型
        int DataHeight;//像素高
        int DataWidth;//像素宽
        float CameraFOV;//相机视场角
        float PosUE[3]; //相机中心位置
        float angEuler[3];//相机欧拉角
        float angQuat[4];//相机四元数
        double timestmp;//时间戳
    };
    '''
    def __init__(self, iv=None):
        if iv != None:
            self.CopyData(iv)
            return
        self.checksum = 0  # 1234567891作为校验
        self.SeqID = 0
        self.TypeID = 0
        self.DataHeight = 0
        self.DataWidth = 0
        self.CameraFOV = 0
        self.PosUE = [0, 0, 0]
        self.angEuler = [0, 0, 0]
        self.angQuat = [0, 0, 0, 0]
        self.timestmp = 0
        self.hasUpdate = True

    # def __init__(self, iv):
    #     self.CopyData(iv)

    def CopyData(self, iv):
        self.checksum = iv[0]
        self.SeqID = iv[1]
        self.TypeID = iv[2]
        self.DataHeight = iv[3]
        self.DataWidth = iv[4]
        self.CameraFOV = iv[5]
        self.PosUE = iv[6:9]
        self.angEuler = iv[9:12]
        self.angQuat = iv[12:16]
        self.timestmp = iv[16]
        self.hasUpdate = True

    def CopyDataOld(self, iv):
        self.checksum = iv[0]
        self.SeqID = iv[1]
        self.TypeID = iv[2]
        self.DataHeight = iv[3]
        self.DataWidth = iv[4]
        self.CameraFOV = iv[5]
        self.PosUE = iv[6:9]
        self.angEuler = iv[9:12]
        self.timestmp = iv[12]
        self.hasUpdate = True

import socket
import struct
import time



##  @brief RflySim3D控制接口类.
#  @details UE4CtrlAPI 类包含在指定IP和端口上监听RflySim3D消息并控制RflySim3D的方法。

# PX4 ueLink listen and control API and RflySim3D control API
class UE4CtrlAPI:
    """ """

    # constructor function
    ## @brief UE4CtrlAPI的构造函数
    # @param ip 要连接的PC的IP地址，默认为"127.0.0.1"
    # @return UE4CtrlAPI对象
    def __init__(self, ip="127.0.0.1"):
        '''
        参数初始化：

        ip：要连接的PC的IP地址，默认为"127.0.0.1"。
        self.startTime：使用当前时间初始化 startTime 属性，用于记录对象创建时间。
        创建UDP套接字：

        self.udp_socket：创建一个UDP套接字用于通信。
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)：设置套接字选项以允许广播。
        self.udp_socketUE4：创建另一个UDP套接字用于与UE4通信。
        self.udp_socketUE4.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)：设置套接字选项以允许地址重用。
        初始化向量：

        self.inSilVect：用于存储传入的Sil数据的向量（列表）。
        self.inReqVect：用于存储传入请求数据的向量（列表）。
        初始化控制UE4的标志和向量：

        self.stopFlagUE4：用于控制UE4停止状态的标志，初始值为 True。
        self.CoptDataVect：用于存储飞行器数据的向量。
        self.ObjDataVect：用于存储对象数据的向量。
        self.CamDataVect：用于存储相机数据的向量。
        初始化线程事件：

        self.hasMsgEvent：用于处理消息存在的事件。
        self.trueMsgEvent：用于处理真实消息状态的事件。
        通过这些步骤，UE4CtrlAPI 类的构造函数创建并初始化了一个可以与RflySim3D进行通信和控制的实例。
        '''
        self.ip = ip
        # self.multicast_ip = multicast_ip
        # self.multicast_port = multicast_port
        self.startTime = time.time()  # 添加这行以初始化 startTime 属性

        self.udp_socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )  # Create socket
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_socketUE4 = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )  # Create socket
        self.udp_socketUE4.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.inSilVect = []
        self.inReqVect = []

        self.stopFlagUE4 = True
        self.CoptDataVect = []
        self.ObjDataVect = []
        self.CamDataVect = []
        self.CamDataVect1 = []      # SensorType == 9

        self.hasMsgEvent = threading.Event()
        self.trueMsgEvent = threading.Event()

    ## @brief 发送RflySim3D控制台命令的方法
    #  - @anchor sendUE4Cmd
    # @param cmd 要发送给RflySim3D的控制台命令
    # @param windowID 要发送给指定UE4窗口的ID，默认为-1，表示发送给所有窗口
    # @return 无
    def sendUE4Cmd(self, cmd, windowID=-1):
        """
        发送命令以控制 RflySim3D 的显示样式
        可用的命令如下，命令字符串形如'RflyShowTextTime txt time'
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
           # 发送到组播地址
            #self.udp_socket.sendto(buf, (self.multicast_ip, self.multicast_port))

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
            #self.udp_socket.sendto(buf, (self.multicast_ip, self.multicast_port))

        # 如果是改变场景的命令，需要等待场景切换完成
        if "RflyChangeMap" in cmd.decode():
            time.sleep(0.5)

        if "RflyChangeMap" in cmd.decode():
            time.sleep(0.5)

    def fillList(self,data,inLen):
        if isinstance(data, np.ndarray):
            data = data.tolist()
            
        if isinstance(data, list) and len(data)==inLen:
            return data
        else:
            if isinstance(data, list):
                datLen = len(data)
                if datLen<inLen:
                    data = data + [0]* (inLen-datLen)
                    
                if datLen>inLen:
                    data = data[0:inLen]
            else:
                data = [data] + [0]* (inLen-1)
        return data

    ## @brief 向同一局域网内所有RflySim3D发送RflySim3D控制台命令的方法
    #  - @anchor sendUE4CmdNet
    # @param cmd 要发送给RflySim3D的控制台命令
    # @return 无
    def sendUE4CmdNet(self, cmd):
        """
        send command to control all RflySim3D in LAN
        struct Ue4CMDNet { //总长度为96
            int checksum; //校验位，这里应该是1234567897
            char data[92];
        } i92s
        
        命令类型检查与转换:

        首先检查 cmd 是否为字符串类型。如果是，使用 encode() 方法将其转换为字节串，因为网络通信要求数据以字节串形式发送。
        命令长度和打包:

        检查命令长度是否超过91字节。因为结构 Ue4CMDNet 的 data 字段设计为92字节，其中最后一个字节通常留给空终止符（如果使用C风格字符串）。
        使用 struct.pack("i92s", 1234567897, cmd) 对命令进行打包。这里 i92s 表示传输数据格式，其中 i 是一个整型的校验位，值为 1234567897；后面的 92s 是92字节的字符串。这样打包确保数据按照指定的格式发送，前四个字节为整型校验位，紧随其后的是命令数据。
        发送命令:

        使用 self.udp_socket.sendto(buf, ("224.0.0.10", 20009)) 将打包后的数据发送到多播地址 224.0.0.10 和端口 20009。所选的多播地址和端口应该与接收端一致，以便所有的 RflySim3D 实例都能接收到这条消息。
        额外操作-命令效果等待:

        如果发送的命令是 "RflyChangeMap"，该命令可能涉及到仿真环境的重大变更（例如，场景切换）。为了确保场景切换能顺利完成，在命令发送后，函数暂停0.5秒，以便给RflySim3D足够的响应时间。


        """

        # 如果是str类型，则转换为bytes类型
        if isinstance(cmd, str):
            cmd = cmd.encode()

        # print(type(cmd))
        if len(cmd) <= 91:
            buf = struct.pack("i92s", 1234567897, cmd)
        else:
            print("Error: Cmd is too long")
            return
        self.udp_socket.sendto(
            buf, ("224.0.0.10", 20009)
        )  # multicast address, send to all RflySim3Ds on all PC in LAN

        # 如果是改变场景的命令，需要等待场景切换完成
        if "RflyChangeMap" in cmd.decode():
            time.sleep(0.5)

        # 如果是改变场景的命令，需要等待场景切换完成
        if "RflyChangeMap" in cmd.decode():
            time.sleep(0.5)

    ## @brief 设置一个自行创建物体的头顶ID位置的显示内容
    #  - @anchor sendUE4LabelID
    # @param CopterID 要设置 ID 位置内容的 自行创建物体ID
    # @param Txt 要在 ID 位置显示的文本内容
    # @param fontSize 文本的字体大小
    # @param RGB 文本的 RGB 颜色。可以用表示颜色值的字符串进行指定，例如 "FF0000" 表示红色，"00FF00" 表示绿色。
    # @param windowID 要显示标签的RflySim3D窗口 ID
    # @return 无
    def sendUE4LabelID(
        self, CopterID=0, Txt="", fontSize=30, RGB=[255, 0, 0], windowID=-1
    ):
        dispTime = 1
        dispFlag = 0
        # dispFlag =0 且 dispTime>=0 表示更新ID行（第0行数据）
        self.sendUE4LabelMsg(CopterID, Txt, fontSize, RGB, dispTime, dispFlag, windowID)

    ## @brief 设置一个自行创建物体的头顶各行Message标签显示内容
    #  - @anchor sendUE4LabelMsg
    # @param CopterID 要设置 Message 显示内容的自行创建物体ID
    # @param Txt 要在 Message 位置显示的文本内容
    # @param fontSize 文本的字体大小
    # @param RGB 文本的 RGB 颜色。可以用表示颜色值的字符串进行指定，例如 "FF0000" 表示红色，"00FF00" 表示绿色。
    # @param dispTime 显示持续时间，单位为秒。如果设置为0，则表示永久显示；如果设置为大于0的数，则表示显示指定秒数后消失；如果设置为小于0的数，则立刻消失。
    # @param dispFlag 显示标志。根据不同的值，有不同的显示效果：
    #                   - 如果 dispFlag < 0，则消息依次累加，显示最多5条。
    #                   - 如果 dispFlag = 0，则清除所有消息。
    #                   - 如果 dispFlag > 0，则更新特定消息行（索引从1开始）。如果更新的行号大于当前消息总数，则切换到累加模式。    
    # @param windowID 要显示标签的RflySim3D窗口 ID
    # @return 无
    def sendUE4LabelMsg(
        self,
        CopterID=0,
        Txt="",
        fontSize=30,
        RGB=[255, 0, 0],
        dispTime=0,
        dispFlag=-1,
        windowID=-1,
    ):
        '''
        
        struct CopterMsg {
            int checksum; //校验位，这里必须设定为1234567899
            int CopterID; //飞机的ID号，具体显示哪一个飞机。注意，如果CopterID<=0，则所有飞机都显示消息；如果CopterID大于0，则对应飞机显示消息
            int dispFlag;//显示需要，如果flag<0，则消息会逐层累加，最多显示5条消息；如果flag=0，会清理所有消息；如果flag>0，则会更新对应的消息；如果flag大于当前消息总数，则消息顺延在末尾。
            int RGB[3];//RGB的颜色，0~255，分别表示红、绿、蓝
            float dispTime;//消失时间（单位秒）。如果<0，则立刻消失；如果=0，则永远显示；如果>0,则设定秒数后消失
            float fontSize;//字体大小；默认是20；
            char data[120];//显示的文字。
        };6i2f120s
        dispFlag <0 且 dispTime>=0 表示依次累加的方式添加消息
        dispFlag <0 且 dispTime<0 表示清空所有消息
        dispFlag =0 且 dispTime>=0 表示更新ID行（第0行数据）
        dispFlag >=1 and <5 表示更新1到5行的消息，注意此时dispTime<0会删除本消息，若>=0会更新消息
        dispFlag>当前消息数，则切换到累加模式
        '''
        data = Txt.encode("UTF-8")
        buf = struct.pack(
            "6i2f120s", 1234567899, CopterID, dispFlag, *RGB, dispTime, fontSize, data
        )
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

    ## @brief 设置在UE4中将一个或多个自建物体附加到其他物体上
    #  - @anchor sendUE4Attatch
    # @param CopterIDs 要附加的飞机的ID列表，最大长度为25
    # @param AttatchIDs 被附加的物体的ID列表，最大长度为25
    # @param AttatchTypes 附加类型的列表，最大长度为25。附加类型包括：
    #                     - 0：正常模式
    #                     - 1：相对位置但不相对姿态
    #                     - 2：相对位置+偏航（不相对俯仰和滚转）
    #                     - 3：相对位置+全姿态（俯仰、滚转、偏航）
    # @param windowID 要显示的RflySim3D窗口ID。默认为-1，表示所有窗口。
    # @return 无    
    def sendUE4Attatch(self, CopterIDs, AttatchIDs, AttatchTypes, windowID=-1):
        """
        发送消息到UE4，将一个或多个飞机附加到其他物体上（最多25个飞机）；
        CopterIDs、AttatchIDs、AttatchTypes可以是长度为25的列表Send msg to UE4 to attach a vehicle to another (25 vehicles);
        CopterIDs,AttatchIDs,AttatchTypes can be a list with max len 25

        struct VehicleAttatch25 {
        	int checksum;//1234567892
        	int CopterIDs[25];
        	int AttatchIDs[25];
        	int AttatchTypes[25];//0：正常模式，1：相对位置不相对姿态，2：相对位置+偏航（不相对俯仰和滚转），3：相对位置+全姿态（俯仰滚转偏航）
        }i25i25i25i
                
        """
        # change the 1D variable to 1D list
        CopterIDs=self.fillList(CopterIDs,25)
        AttatchIDs=self.fillList(AttatchIDs,25)
        AttatchTypes=self.fillList(AttatchTypes,25)


        buf = struct.pack(
            "i25i25i25i", 1234567892, *CopterIDs, *AttatchIDs, *AttatchTypes
        )
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


    ## @brief 在指定位置创建/更新模型
    #  - @anchor sendUE4Pos
    #  @param copterID（默认值为 1）：飞行器的ID，指定要创建或更新状态的飞行器。
    #   @param vehicleType（默认值为 3）：飞行器的类型。根据实际情况，可以指定不同的类型来创建或更新不同类型的飞行器模型。
    #   @param MotorRPMSMean（默认值为 0）：电机转速的均值。这个参数用于控制电机的转速，可以影响模型的运动状态。
    #   @param PosE（默认值为 [0, 0, 0]）：位置信息，表示飞行器当前的位置。这是一个包含三个元素的列表，分别表示飞行器在 x、y、z 坐标轴上的位置。
    #   @param AngEuler（默认值为 [0, 0, 0]）：欧拉角信息，表示飞行器当前的姿态。这是一个包含三个元素的列表，分别表示飞行器的俯仰、滚转和偏航角度。
    #   @param windowID（默认值为 -1）：窗口的ID。如果指定了窗口ID，位置和角度信息将仅应用于指定的窗口。如果 windowID 为负数，则位置和角度信息将应用于所有窗口。
    def sendUE4Pos(
        self,
        copterID=1,
        vehicleType=3,
        MotorRPMSMean=0,
        PosE=[0, 0, 0],
        AngEuler=[0, 0, 0],
        windowID=-1,
    ):
        """
        send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        """
        VelE = [0, 0, 0]
        self.sendUE4PosNew(
            copterID,
            vehicleType,
            PosE,
            AngEuler,
            VelE,
            [MotorRPMSMean] * 8,
            -1,
            windowID,
        )

    ## @brief 在指定位置创建/更新模型
    #  - @anchor sendUE4Pos2Ground
    #  @param copterID（默认值为 1）：飞行器的ID，指定要创建或更新状态的飞行器。
	#   @param vehicleType（默认值为 3）：飞行器的类型。根据实际情况，可以指定不同的类型来创建或更新不同类型的飞行器模型。
	#   @param MotorRPMSMean（默认值为 0）：电机转速的均值。这个参数用于控制电机的转速，可以影响模型在地面上的状态。
	#   @param PosE（默认值为 [0, 0, 0]）：位置信息，表示飞行器当前在地面上的位置。这是一个包含三个元素的列表，分别表示飞行器在 x、y、z 坐标轴上的位置。
	#   @param AngEuler（默认值为 [0, 0, 0]）：欧拉角信息，表示飞行器当前的姿态。这是一个包含三个元素的列表，分别表示飞行器的俯仰、滚转和偏航角度。
	#   @param windowID（默认值为 -1）：窗口的ID。如果指定了窗口ID，位置和角度信息将仅应用于指定的窗口。如果 windowID 为负数，则位置和角度信息将应用于所有窗口。
    def sendUE4Pos2Ground(
        self,
        copterID=1,
        vehicleType=3,
        MotorRPMSMean=0,
        PosE=[0, 0, 0],
        AngEuler=[0, 0, 0],
        windowID=-1,
    ):
        """
        send the position & angle information to RflySim3D to create a new 3D model or update the old model's states on the ground
        checksum =1234567891 is adopted here to tell UE4 to genearete a object always fit the ground
        struct SOut2SimulatorSimple {
            int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
            int copterID;  //Vehicle ID
            int vehicleType;  //Vehicle type
            float MotorRPMSMean; // mean motor speed
            float PosE[3];   //NED vehicle position in earth frame (m)
            float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
        }  3i7f
        """
        PosE=self.fillList(PosE,3)
        AngEuler=self.fillList(AngEuler,3)
        
        buf = struct.pack(
            "3i7f",
            1234567891,
            copterID,
            vehicleType,
            MotorRPMSMean,
            PosE[0],
            PosE[1],
            PosE[2],
            AngEuler[0],
            AngEuler[1],
            AngEuler[2],
        )
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

    ## @brief 在指定位置创建/更新模型
    #  - @anchor sendUE4PosScale
    #  @param self：表示类的实例对象，用于访问类的成员变量和其他方法。
	#   @param copterID（默认值为 1）：飞行器的ID，用于指定要创建或更新状态的飞行器。
	#   @param vehicleType（默认值为 3）：飞行器的类型，根据实际情况可以指定不同的类型来创建或更新不同类型的飞行器模型。
	#   @param MotorRPMSMean（默认值为 0）：电机转速的均值，用于控制电机的转速，影响模型的运动状态。
	#   @param PosE（默认值为 [0, 0, 0]）：位置信息，表示飞行器当前的位置。这是一个包含三个元素的列表，分别表示飞行器在 x、y、z 坐标轴上的位置。
	#   @param AngEuler（默认值为 [0, 0, 0]）：欧拉角信息，表示飞行器当前的姿态。这是一个包含三个元素的列表，分别表示飞行器的俯仰、滚转和偏航角度。
	#   @param Scale（默认值为 [1, 1, 1]）：比例信息，用于更改模型的比例。这是一个包含三个元素的列表，分别表示飞行器在 x、y、z 轴上的比例。
	#   @param windowID（默认值为 -1）：窗口的ID。如果指定了窗口ID，位置、姿态和比例信息将仅应用于指定的窗口。如果 windowID 为负数，则位置、姿态和比例信息将应用于所有窗口。

    def sendUE4PosScale(
        self,
        copterID=1,
        vehicleType=3,
        MotorRPMSMean=0,
        PosE=[0, 0, 0],
        AngEuler=[0, 0, 0],
        Scale=[1, 1, 1],
        windowID=-1,
    ):
        """
        send the position & angle information to RflySim3D to create a new 3D model or update the old model's states with changing the scale
        struct SOut2SimulatorSimple1 {
            int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
            int copterID;  //Vehicle ID
            int vehicleType;  //Vehicle type
            float MotorRPMSMean; // mean motor speed
            float PosE[3];   //NED vehicle position in earth frame (m)
            float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            float Scale[3];
        }  3i10f
        """
        PosE=self.fillList(PosE,3)
        AngEuler=self.fillList(AngEuler,3)
        Scale=self.fillList(Scale,3)
        buf = struct.pack(
            "3i10f",
            1234567890,
            copterID,
            vehicleType,
            MotorRPMSMean,
            PosE[0],
            PosE[1],
            PosE[2],
            AngEuler[0],
            AngEuler[1],
            AngEuler[2],
            Scale[0],
            Scale[1],
            Scale[2],
        )
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
    
    ## @brief 在指定位置创建/更新模型
    #  - @anchor sendUE4PosScale2Ground
    #  @param copterID（默认值为 1）：飞行器的ID，用于指定要创建或更新状态的飞行器。
	#   @param vehicleType（默认值为 3）：飞行器的类型，根据实际情况可以指定不同的类型来创建或更新不同类型的飞行器模型。
	#   @param    MotorRPMSMean（默认值为 0）：电机转速的均值，用于控制电机的转速，影响模型的运动状态。
	#   @param PosE（默认值为 [0, 0, 0]）：位置信息，表示飞行器当前的位置。这是一个包含三个元素的列表，分别表示飞行器在 x、y、z 坐标轴上的位置。
	#   @param AngEuler（默认值为 [0, 0, 0]）：欧拉角信息，表示飞行器当前的姿态。这是一个包含三个元素的列表，分别表示飞行器的俯仰、滚转和偏航角度。
	#   @param Scale（默认值为 [1, 1, 1]）：比例信息，用于更改模型的比例。这是一个包含三个元素的列表，分别表示飞行器在 x、y、z 轴上的比例。
	#   @param windowID（默认值为 -1）：窗口的ID。如果指定了窗口ID，位置、姿态和比例信息将仅应用于指定的窗口。如果 windowID 为负数，则位置、姿态和比例信息将应用于所有窗口。

    def sendUE4PosScale2Ground(
        self,
        copterID=1,
        vehicleType=3,
        MotorRPMSMean=0,
        PosE=[0, 0, 0],
        AngEuler=[0, 0, 0],
        Scale=[1, 1, 1],
        windowID=-1,
    ):
        """
        send the position & angle information to RflySim3D to create a new 3D model or update the old model's states with changing the scale
        checksum =1234567891 is adopted here to tell UE4 to genearete a object always fit the ground
        struct SOut2SimulatorSimple1 {
            int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
            int copterID;  //Vehicle ID
            int vehicleType;  //Vehicle type
            float MotorRPMSMean; // mean motor speed
            float PosE[3];   //NED vehicle position in earth frame (m)
            float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            float Scale[3];
        }  3i10f
        """
        PosE=self.fillList(PosE,3)
        AngEuler=self.fillList(AngEuler,3)
        Scale=self.fillList(Scale,3)
        buf = struct.pack(
            "3i10f",
            1234567891,
            copterID,
            vehicleType,
            MotorRPMSMean,
            PosE[0],
            PosE[1],
            PosE[2],
            AngEuler[0],
            AngEuler[1],
            AngEuler[2],
            Scale[0],
            Scale[1],
            Scale[2],
        )
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

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    ## @brief 在指定位置创建/更新模型
    #  - @anchor sendUE4PosFull
    #  @param copterID：飞行器的ID，用于指定要创建或更新状态的飞行器。
	#   @param vehicleType：飞行器的类型，根据实际情况可以指定不同的类型来创建或更新不同类型的飞行器模型。
	#   @param MotorRPMS（默认值为 [0，0，0，0，0，0，0，0] ）：电机转速信息，用于控制飞行器的电机转速。
	#   @param VelE：速度信息，表示飞行器当前水平面坐标系（NED）的速度。
	#   @param PosE：位置信息，表示飞行器当前水平面坐标系（NED）的位置。
	#   @param RateB：角速度信息，表示飞行器当前机体坐标系的角速度。
	#   @param AngEuler：欧拉角信息，表示飞行器当前的姿态。
	#   @param windowID（默认值为 -1）：窗口的ID。如果指定了窗口ID，上述信息将仅应用于指定的窗口。如果 windowID 为负数，则这些信息将应用于所有窗口。

    def sendUE4PosFull(
        self, copterID, vehicleType, MotorRPMS=[0]*8, VelE=[0,0,0], PosE=[0,0,0], RateB=[0,0,0], AngEuler=[0,0,0], windowID=-1
    ):
        """
        send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        # //输出到模拟器的数据
        # struct SOut2Simulator {
        #     int checksum; // 123456789
        #     int copterID;  //Vehicle ID
        #     int vehicleType;  //Vehicle type
        #     int reserv; //备用标志位
        #     float VelE[3];   //NED vehicle velocity in earth frame (m/s)
        #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
        #     float AngQuatern[4]; //Vehicle attitude in Quaternion
        #     float MotorRPMS[8];  //Motor rotation speed (RPM)
        #     float AccB[3];       //Vehicle acceleration in body frame x y z (m/s/s)
        #     float RateB[3];      //Vehicle angular speed in body frame x y z (rad/s)
        #     double runnedTime; //Current  stamp (s)
        #     double PosE[3];   //NED vehicle position in earth frame (m)
        #     double PosGPS[3];    //vehicle longitude, latitude and altitude (degree,degree,m)
        # } 4i24f7d
        """
        checksum=123456789
        PosE=self.fillList(PosE,3)
        AngEuler=self.fillList(AngEuler,3)
        MotorRPMS=self.fillList(MotorRPMS,8)
        RateB=self.fillList(RateB,3)
        VelE=self.fillList(VelE,3)
        
        runnedTime = -1
        reserv=0
        # VelE=[0,0,0]
        AngQuatern = [0, 0, 0, 0]
        AccB = [0, 0, 0]
        # RateB=[0,0,0]
        PosGPS = [0, 0, 0]
        # buf for SOut2Simulator, len=152
        buf = struct.pack(
            "4i24f7d",
            checksum,
            copterID,
            vehicleType,
            reserv,
            *VelE,
            *AngEuler,
            *AngQuatern,
            *MotorRPMS,
            *AccB,
            *RateB,
            runnedTime,
            *PosE,
            *PosGPS,
        )
        # print(len(buf))
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
        # print('Message Send')
    
    ## @brief 触发扩展蓝图接口
    #  - @anchor sendUE4ExtAct
    #  @param	copterID（默认值为 1）：飞行器的ID，用于指定要执行外部动作的飞行器。
	#   @param ActExt（默认值为 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]）：外部动作信息，用于指定飞行器执行的外部动作。这是一个包含 16 个元素的列表，每个元素表示一个外部动作的值。
	#   @param windowID（默认值为 -1）：窗口的ID。如果指定了窗口ID，外部动作信息将仅应用于指定的窗口。如果 windowID 为负数，则外部动作信息将应用于所有窗口。
    def sendUE4ExtAct(
        self,
        copterID=1,
        ActExt=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        windowID=-1,
    ):
        '''
        struct Ue4ExtMsg {
            int checksum;//1234567894
            int CopterID;
            double runnedTime; //Current  stamp (s)
            double ExtToUE4[16];
        }
        struct.pack 2i1d16f
        '''
        ActExt=self.fillList(ActExt,16)
        
        runnedTime = time.time() - self.startTime
        checkSum = 1234567894
        buf = struct.pack("2i1d16d", checkSum, copterID, runnedTime, *ActExt)
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
        # print('Message Send')

 
    ## @brief 创建/更新模型
    #  - @anchor sendUE4PosSimple
    #  @param copterID：飞行器的ID，用于指定要创建或更新状态的飞行器。这是一个任意类型的参数，表示飞行器的标识。
	#   @param vehicleType：飞行器的类型，根据实际情况可以指定不同的类型来创建或更新不同类型的飞行器模型。
	#   @param PWMs（默认值[0，0，0，0，0，0，0，0]）：PWM（脉宽调制）信息，表示飞行器的电机控制信号。
	#   @param VelE：速度信息，表示飞行器当前的速度。
	#   @param PosE：位置信息，表示飞行器当前的位置。
	#   @param AngEuler：欧拉角信息，表示飞行器当前的姿态。
	#   @param runnedTime（默认值为 -1）：时间戳，以毫秒为单位
	#   @param windowID（默认值为 -1）：窗口的ID。如果指定了窗口ID，上述信息将仅应用于指定的窗口。如果 windowID 为负数，则这些信息将应用于所有窗口。

    def sendUE4PosSimple(
        self,
        copterID,
        vehicleType,
        PWMs,
        VelE,
        PosE,
        AngEuler,
        runnedTime=-1,
        windowID=-1,
    ):
        """
        send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        //输出到模拟器的数据
        struct SOut2SimulatorSimpleTime {
            int checkSum;
            int copterID;  //Vehicle ID
            int vehicleType;  //Vehicle type
            float PWMs[8];
            float PosE[3];   //NED vehicle position in earth frame (m)
            float VelE[3];
            float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            double runnedTime; //Current Time stamp (s)
        };
        struct.pack 3i17f1d
        """
        
        PWMs=self.fillList(PWMs,8)
        PosE=self.fillList(PosE,3)
        VelE=self.fillList(VelE,3)
        AngEuler=self.fillList(AngEuler,3)
        
        # if runnedTime<0:
        #     runnedTime = time.time()-self.startTime
        checkSum = 1234567890
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack(
            "3i17f1d",
            checkSum,
            copterID,
            vehicleType,
            *PWMs,
            *PosE,
            *VelE,
            *AngEuler,
            runnedTime,
        )
        # print(len(buf))
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
        # print('Message Send')

    ## @brief 创建/更新模型
    #  - @anchor sendUE4PosNew
    #  @param  copterID（默认值为 1）：飞行器的ID，用于指定要创建或更新状态的飞行器。
	#   @param vehicleType（默认值为 3）：飞行器的类型，根据实际情况可以指定不同的类型来创建或更新不同类型的飞行器模型。
	#   @param PosE（默认值为 [0, 0, 0]）：位置信息，表示飞行器当前的位置。这是一个包含三个元素的列表，分别表示飞行器在 x、y、z 坐标轴上的位置。
	#   @param AngEuler（默认值为 [0, 0, 0]）：欧拉角信息，表示飞行器当前的姿态。这是一个包含三个元素的列表，分别表示飞行器的俯仰、滚转和偏航角度。
	#   @param VelE（默认值为 [0, 0, 0]）：速度信息，表示飞行器当前的速度。这是一个包含三个元素的列表，分别表示飞行器在 x、y、z 轴上的速度。
	#   @param PWMs（默认值为 [0，0，0，0，0，0，0，0] ）：PWM（脉宽调制）信息，表示飞行器的电机控制信号。这是一个包含八个元素的列表，用于控制飞行器电机的转速或推力。
	#   @param runnedTime（默认值为 -1）：运行时间，以毫秒为单位，表示飞行器的运行时间。
	#   @param windowID（默认值为 -1）：窗口的ID。如果指定了窗口ID，位置、姿态、速度等信息将仅应用于指定的窗口。如果 windowID 为负数，则这些信息将应用于所有窗口。
  
    def sendUE4PosNew(
        self,
        copterID=1,
        vehicleType=3,
        PosE=[0, 0, 0],
        AngEuler=[0, 0, 0],
        VelE=[0, 0, 0],
        PWMs=[0] * 8,
        runnedTime=-1,
        windowID=-1,
    ):
        """
        send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        # //输出到模拟器的数据
        # struct SOut2SimulatorSimpleTime {
        #     int checkSum; //1234567891
        #     int copterID;  //Vehicle ID
        #     int vehicleType;  //Vehicle type
        #     int PosGpsInt[3];   //lat*10^7,lon*10^7,alt*10^3，int型发放节省空间
        #     float MotorRPMS[8];
        #     float VelE[3];
        #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
        #     double PosE[3];   //NED vehicle position in earth frame (m)
        #     double runnedTime; //Current Time stamp (s)
        # }6i14f4d
        """
        # if runnedTime<0:
        #     runnedTime = time.time()-self.startTime
        
        PosE=self.fillList(PosE,3)
        AngEuler=self.fillList(AngEuler,3)
        VelE=self.fillList(VelE,3)
        PWMs=self.fillList(PWMs,8)
        PosGpsInt=[0,0,0]
        checkSum = 1234567891
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack(
            "6i14f4d",
            checkSum,
            copterID,
            vehicleType,
            *PosGpsInt,
            *PWMs,
            *VelE,
            *AngEuler,
            *PosE,
            runnedTime
        )
        # print(len(buf))
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
        # print('Message Send')

    # //输出到CopterSim DLL模型的SILints和SILFloats数据
    # struct PX4SILIntFloat{
    #     int checksum;//1234567897
    #     int CopterID;
    #     int inSILInts[8];
    #     float inSILFLoats[20];
    # };
    # struct.pack 10i20f

    ## @brief 创建100个Copter
    #  - @anchor sendUE4PosScale100
    #  @param copterID：飞行器的ID，用于指定要创建或更新状态的飞行器。这是一个任意类型的参数，表示飞行器的标识。
	#   @param vehicleType：飞行器的类型，根据实际情况可以指定不同的类型来创建或更新不同类型的飞行器模型。
	#   @param PosE：位置信息，表示飞行器当前的位置。
	#   @param AngEuler：欧拉角信息，表示飞行器当前的姿态。
	#   @param MotorRPMSMean（默认值为0）：电机转速的均值，用于控制电机的转速，影响模型的运动状态。
	#   @param Scale：比例信息，用于更改模型的比例。
	#   @param isFitGround（默认值为 False）：是否适应地面。如果设置为 True，则模型会根据地面结构进行调整；如果设置为 False，则不会进行调整。
	#   @param windowID（默认值为 -1）：窗口的ID。如果指定了窗口ID，上述信息将仅应用于指定的窗口。如果 windowID 为负数，则这些信息将应用于所有窗口。

    def sendUE4PosScale100(
        self,
        copterID,
        vehicleType,
        PosE,
        AngEuler,
        MotorRPMSMean,
        Scale,
        isFitGround=False,
        windowID=-1,
    ):
        """
        send the position & angle information to RflySim3D to create 100 vehicles once
        #struct Multi3DData100New {
        #    int checksum;
        #    uint16 copterID[100];
        #    uint16 vehicleType[100];
        #    float PosE[300];
        #    float AngEuler[300];
        #    uint16 Scale[300];
        #    float MotorRPMSMean[100];
        #}
        #checksum是数据校验位，1234567890表示正常数据，1234567891表示飞机始终贴合地面
        #copterID是100维整型数组，飞机ID，这个从1给到100即可,如果ID给0则不显示次飞机
        #vehicleType是100维整型数组，飞机类型，四旋翼给3即可
        #MotorRPMSMean是100维数组，飞机螺旋桨转速，单位RPM，四旋翼默认给1000即可
        #PosE是300维数组，飞机位置，可以随机生成，单位是m
        #AngEuler是300维数组，飞机姿态角，单位弧度，默认都给0，表示正常放置
        #Scale是300维的数组，数据是xyz显示的尺寸*100，默认都给100表示实际大小即1倍
        """
        copterID=self.fillList(copterID,100)
        vehicleType=self.fillList(vehicleType,100)
        PosE=self.fillList(PosE,300)
        AngEuler=self.fillList(AngEuler,300)
        Scale=self.fillList(Scale,300)
        MotorRPMSMean=self.fillList(MotorRPMSMean,100)
        
        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack(
            "i200H600f300H100f",
            checksum,
            *copterID,
            *vehicleType,
            *PosE,
            *AngEuler,
            *Scale,
            *MotorRPMSMean,
        )
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

    ## @brief 创建20个Copter
    #  - @anchor sendUE4PosScalePwm20
    #  @param copterID：飞行器的ID，用于指定要创建或更新状态的飞行器。这是一个任意类型的参数，表示飞行器的标识。
	#   @param vehicleType：飞行器的类型，根据实际情况可以指定不同的类型来创建或更新不同类型的飞行器模型。
	#   @param PosE：位置信息，表示飞行器当前的位置。
	#   @param AngEuler：欧拉角信息，表示飞行器当前的姿态。
	#   @param Scale：比例信息，用于更改模型的比例。
	#   @param PWMs（默认值为0）：PWM（脉宽调制）信息，表示飞行器的电机控制信号。
	#   @param isFitGround（默认值为 False）：是否适应地面。如果设置为 True，则模型会根据地面结构进行调整；如果设置为 False，则不会进行调整。
	#   @param windowID（默认值为 -1）：窗口的ID。如果指定了窗口ID，上述信息将仅应用于指定的窗口。如果 windowID 为负数，则这些信息将应用于所有窗口。

    def sendUE4PosScalePwm20(
        self,
        copterID,
        vehicleType,
        PosE,
        AngEuler,
        Scale,
        PWMs,
        isFitGround=False,
        windowID=-1,
    ):
        """
        send the position & angle information to RflySim3D to create 20 vehicles once
        # struct Multi3DData2New {
        #   int checksum;
        # 	uint16 copterID[20];
        # 	uint16 vehicleType[20];
        # 	float PosE[60];
        # 	float AngEuler[60];
        # 	uint16 Scale[60];
        # 	float PWMs[160];
        # }
        #checksum是数据校验位，1234567890表示正常数据，1234567891表示飞机始终贴合地面
        #copterID是20维整型数组，飞机ID，这个从1给到20即可,如果ID给0则不显示次飞机
        #vehicleType是20维整型数组，飞机类型，四旋翼给3即可
        #PosE是60维数组，飞机位置，可以随机生成，单位是m
        #AngEuler是60维数组，飞机姿态角，单位弧度，默认都给0，表示正常放置
        #Scale是300维的数组，数据是xyz显示的尺寸*100，默认都给100表示实际大小即1倍
        #PWMs是160维数组，对应20个飞机各8个旋翼转速，单位RPM，四旋翼默认给1000即可
        """
        
        copterID=self.fillList(copterID,20)
        vehicleType=self.fillList(vehicleType,20)
        PosE=self.fillList(PosE,60)
        AngEuler=self.fillList(AngEuler,60)
        Scale=self.fillList(Scale,60)
        PWMs=self.fillList(PWMs,160)
        
        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack(
            "i40H120f60H160f",
            checksum,
            *copterID,
            *vehicleType,
            *PosE,
            *AngEuler,
            *Scale,
            *PWMs,
        )
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

    ## @brief 获取Copter位置
    #  - @anchor getUE4Pos
    #  @param
    def getUE4Pos(self, CopterID=1):
        if self.stopFlagUE4:  # 如果没有启用监听程序
            self.initUE4MsgRec()
            time.sleep(1)

        for i in range(len(self.inReqVect)):  # 遍历数据列表，飞机ID有没有出现过
            if self.inReqVect[i].copterID == CopterID:
                posE = self.inReqVect[i].PosE
                return [posE[0], posE[1], posE[2], 1]
        return [0, 0, 0, 0]

    ## @brief 获取Copter数据
    #  - @anchor getUE4Data
    #  @param CopterID
    def getUE4Data(self, CopterID=1):
        if self.stopFlagUE4:  # 如果没有启用监听程序
            self.initUE4MsgRec()
            time.sleep(1)

        for i in range(len(self.inReqVect)):  # 遍历数据列表，飞机ID有没有出现过
            if self.inReqVect[i].copterID == CopterID:
                return self.inReqVect[i]
        return 0

    ## @brief 启用监听
    #  - @anchor initUE4MsgRec
    def initUE4MsgRec(self):
        """Initialize the UDP data linsening from UE4,
        currently, the crash data is listened
        """
        self.stopFlagUE4 = False
        # print("InitUE4MsgRec", self.stopFlagUE4)
        self.inSilVect = []
        self.inReqVect = []
        self.inReqUpdateVect = []
        MYPORT = 20006
        MYGROUP = "224.0.0.10"
        ANY = "0.0.0.0"
        # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.udp_socketUE4.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socketUE4.bind((ANY, MYPORT))
        try:
            status = self.udp_socketUE4.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                socket.inet_aton(MYGROUP) + socket.inet_aton(ANY),
            )
        except:
            print('Failed to Init multicast!')
        self.t4 = threading.Thread(target=self.UE4MsgRecLoop, args=())
        self.t4.start()

    ## @brief 终止监听
    #  - @anchor endUE4MsgRec
    def endUE4MsgRec(self):
        """End UE4 message listening"""
        self.stopFlagUE4 = True
        time.sleep(0.5)
        self.t4.join()
        self.udp_socketUE4.close()

    ## @brief 监听循环
    #  - @anchor UE4MsgRecLoop
    def UE4MsgRecLoop(self):
        """
        UE4 message listening dead loop
        它是监听224.0.0.10：20006，以及自身的20006端口的处理函数，用于处理RflySim3D或CopterSim返回的消息，一共有6种消息：
        1)	长度为12字节的CopterSimCrash：
        struct CopterSimCrash {
            int checksum;
            int CopterID;
            int TargetID;
        }
        由RflySim3D返回的碰撞数据，RflySim3D中按P键时RflySim3D会开启碰撞检测模式，如果发生了碰撞，会传回这个数据，P+数字可以选择发送模式（0本地发送，1局域网发送，2局域网只碰撞时发送），默认为本地发送。
        2)	长度为120字节的PX4SILIntFloat：
        struct PX4SILIntFloat{
            int checksum;//1234567897
            int CopterID;
            int inSILInts[8];
            float inSILFLoats[20];
        };
        3)	长度为160字节的reqVeCrashData：
        struct reqVeCrashData {
            int checksum; //数据包校验码1234567897
            int copterID; //当前飞机的ID号
            int vehicleType; //当前飞机的样式
            int CrashType;//碰撞物体类型，-2表示地面，-1表示场景静态物体，0表示无碰撞，1以上表示被碰飞机的ID号
            double runnedTime; //当前飞机的时间戳
            float VelE[3]; // 当前飞机的速度
            float PosE[3]; //当前飞机的位置
            float CrashPos[3];//碰撞点的坐标
            float targetPos[3];//被碰物体的中心坐标
            float AngEuler[3]; //当前飞机的欧拉角
            float MotorRPMS[8]; //当前飞机的电机转速
            float ray[6]; //飞机的前后左右上下扫描线
            char CrashedName[20] = {0};//被碰物体的名字
        }
        前面介绍过这个类了，在开启数据回传的情况下，RflySim3D会为所有Copter发送reqVeCrashData（数据变化时发送，每秒一次）。
        4)	长度为56字节的CameraData：
        struct CameraData { //56
            int checksum = 0;//1234567891
            int SeqID; //相机序号
            int TypeID;//相机类型
            int DataHeight;//像素高
            int DataWidth;//像素宽
            float CameraFOV;//相机视场角
            float PosUE[3]; //相机中心位置
            float angEuler[3];//相机欧拉角
            double timestmp;//时间戳
        };
        RflySim3D根据reqCamCoptObj函数发送的命令，定时返回的该结构体，该程序接收到后会存放在self.CamDataVect中。
        5)	长度为64字节的CoptReqData：
        struct CoptReqData { //64
            int checksum = 0; //1234567891作为校验
            int CopterID;//飞机ID
            float PosUE[3]; //物体中心位置（人为三维建模时指定，姿态坐标轴，不一定在几何中心）
            float angEuler[3];//物体欧拉角
            float boxOrigin[3];//物体几何中心坐标
            float BoxExtent[3];//物体外框长宽高的一半
            double timestmp;//时间戳
        };
        RflySim3D根据reqCamCoptObj函数发送的命令，定时返回的该结构体，该程序接收到后会存放在self.CoptDataVect中。
        6)	长度为96字节的ObjReqData：
        struct ObjReqData { //96
            int checksum = 0; //1234567891作为校验
            int seqID = 0;
            float PosUE[3]; //物体中心位置（人为三维建模时指定，姿态坐标轴，不一定在几何中心）
            float angEuler[3];//物体欧拉角
            float boxOrigin[3];//物体几何中心坐标
            float BoxExtent[3];//物体外框长宽高的一半
            double timestmp;//时间戳
            char ObjName[32] = { 0 };//碰物体的名字
        };
        RflySim3D根据reqCamCoptObj函数发送的命令，定时返回的该结构体，该程序接收到后会存放在self.ObjDataVect中。
        收到的数据可以使用getCamCoptObj函数进行获取。

        """
        # lastTime = time.time()
        # while True:
        #     if self.stopFlagUE4:
        #         break
        #     lastTime = lastTime + 0.01
        #     sleepTime = lastTime - time.time()
        #     if sleepTime > 0:
        #         time.sleep(sleepTime)
        #     else:
        #         lastTime = time.time()
        #     # print(time.time())

        # struct CopterSimCrash {
        # 	int checksum;
        # 	int CopterID;
        # 	int TargetID;
        # }
        while True:
            if self.stopFlagUE4:
                break

            try:
                buf, addr = self.udp_socketUE4.recvfrom(65500)
                # print('Data Received!')
                if len(buf) == 12:
                    checksum, CopterID, targetID = struct.unpack("iii", buf[0:12])
                    if checksum == 1234567890:
                        if targetID > -0.5 and CopterID == self.CopterID:
                            self.isVehicleCrash = True
                            self.isVehicleCrashID = targetID
                        print(
                            "Vehicle #", CopterID, " Crashed with vehicle #", targetID
                        )

                if len(buf) == 120:
                    iValue = struct.unpack("10i20f", buf[0:120])
                    if iValue[0] == 1234567897:
                        isCopterExist = False
                        for i in range(len(self.inSilVect)):  # 遍历数据列表，飞机ID有没有出现过
                            if self.inSilVect[i].CopterID == iValue[1]:  # 如果出现过，就直接更新数据
                                isCopterExist = True
                                self.inSilVect[i].checksum = iValue[0]
                                self.inSilVect[i].inSILInts = iValue[2:10]
                                self.inSilVect[i].inSILFLoats = iValue[10:30]
                                # break
                        if not isCopterExist:  # 如果没有出现过，就创建一个结构体
                            vsr = PX4SILIntFloat(iValue)
                            self.inSilVect = self.inSilVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                if len(buf) == 160:
                    isCopterExist = False
                    iValue = struct.unpack("4i1d29f20s", buf[0:160])
                    if iValue[0] == 1234567897:
                        # print(vsr.copterID,vsr.vehicleType)
                        for i in range(len(self.inReqVect)):  # 遍历数据列表，飞机ID有没有出现过
                            if self.inReqVect[i].copterID == iValue[1]:  # 如果出现过，就直接更新数据
                                isCopterExist = True
                                self.inReqVect[i].CopyData(
                                    iValue
                                )  # =copy.deepcopy(vsr)
                                self.inReqUpdateVect[i] = True
                                break
                                # break
                        if not isCopterExist:  # 如果没有出现过，就创建一个结构体
                            vsr = reqVeCrashData(iValue)
                            self.inReqVect = self.inReqVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素
                            self.inReqUpdateVect = self.inReqUpdateVect + [True]

                # 旧版协议，无四元数输出
                if len(buf) == 56:  # CameraData: #长度56, 5i7f1d
                    # print('hello')
                    iValue = struct.unpack("5i7f1d", buf[0:56])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按SeqID来构建相机列表
                        for i in range(len(self.CamDataVect)):  # 遍历数据列表，相机ID有没有出现过
                            if self.CamDataVect[i].SeqID == iValue[1]:  # 如果出现过，就直接更新数据
                                self.CamDataVect[i].CopyDataOld(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = CameraData()
                            vsr.CopyDataOld(iValue)
                            self.CamDataVect = self.CamDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                if len(buf) == 72:  # CameraData: #长度72, 5i11f1d
                    # print('hello')
                    iValue = struct.unpack("5i11f1d", buf[0:72])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按SeqID来构建相机列表
                        for i in range(len(self.CamDataVect)):  # 遍历数据列表，相机ID有没有出现过
                            if self.CamDataVect[i].SeqID == iValue[1]:  # 如果出现过，就直接更新数据
                                self.CamDataVect[i].CopyData(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = CameraData(iValue)
                            self.CamDataVect = self.CamDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素
                # 接收吊舱数据 
                if len(buf) == 148:     # SensorType == 9
                    iValue = struct.unpack("2H1I14H28f", buf[0:148])
                    if iValue[0] == 12346:
                        isDataExist = False
                        # 按SeqID来构建相机列表
                        for i in range(len(self.CamDataVect1)):  # 遍历数据列表，相机ID有没有出现过
                            if self.CamDataVect1[i].SeqID == iValue[1]:  # 如果出现过，就直接更新数据
                                self.CamDataVect1[i].CopyData(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = VisionSensorReqNew(iValue)
                            self.CamDataVect1 = self.CamDataVect1 + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                # 旧版协议，无四元数输出
                if len(buf) == 64:  # CoptReqData: #长度64, 2i12f1d
                    iValue = struct.unpack("2i12f1d", buf[0:64])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按CopterID来构建物体飞机
                        for i in range(len(self.CoptDataVect)):  # 遍历数据列表，飞机ID有没有出现过
                            if (
                                self.CoptDataVect[i].CopterID == iValue[1]
                            ):  # 如果出现过，就直接更新数据
                                self.CoptDataVect[i].CopyDataOld(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = CoptReqData()
                            vsr.CopyDataOld(iValue)
                            self.CoptDataVect = self.CoptDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                if len(buf) == 80:  # CoptReqData: #长度80, 2i16f1d
                    iValue = struct.unpack("2i16f1d", buf[0:80])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按CopterID来构建物体飞机
                        for i in range(len(self.CoptDataVect)):  # 遍历数据列表，飞机ID有没有出现过
                            if (
                                self.CoptDataVect[i].CopterID == iValue[1]
                            ):  # 如果出现过，就直接更新数据
                                self.CoptDataVect[i].CopyData(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = CoptReqData(iValue)
                            self.CoptDataVect = self.CoptDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                # 旧版协议，无四元数输出
                if len(buf) == 96:  # ObjReqData: #长度96, 2i12f1d32s
                    iValue = struct.unpack("2i12f1d32s", buf[0:96])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按reqID来构建物体列表
                        for i in range(len(self.ObjDataVect)):  # 遍历数据列表，物体ID有没有出现过
                            if self.ObjDataVect[i].seqID == iValue[1]:  # 如果出现过，就直接更新数据
                                self.ObjDataVect[i].CopyDataOld(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = ObjReqData()
                            vsr.CopyDataOld(iValue)
                            self.ObjDataVect = self.ObjDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                if len(buf) == 112:  # ObjReqData: #长度112, 2i16f1d32s
                    iValue = struct.unpack("2i16f1d32s", buf[0:112])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按reqID来构建物体列表
                        for i in range(len(self.ObjDataVect)):  # 遍历数据列表，物体ID有没有出现过
                            if self.ObjDataVect[i].seqID == iValue[1]:  # 如果出现过，就直接更新数据
                                self.ObjDataVect[i].CopyData(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = ObjReqData(iValue)
                            self.ObjDataVect = self.ObjDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                self.trueMsgEvent.set()

            except:
                self.stopFlagUE4 = True
                print("Error Data")
                break

    ## @brief 获取场景内指定物体数据
    #  - @anchor getCamCoptObj
    #  @param type_id
    #  @param objName
    #  @return
    #  - 0表示未找到，返回0
    #  - 1表示找到，返回对应的对象
    def getCamCoptObj(self, type_id=1, objName=1):
        '''
        
        type=0表示相机，1表示手动创建带copter_id的目标，2表示场景中自带的物体
        相机时，objName对应相机seqID；指定copterid目标时，objName对应CopterID，请求场景中自带的物体时，objName对应物体名字(字符串)
        '''
        if (
            type_id == 1
            and type(objName) == str
            or type_id == 2
            and type(objName) == int
        ):
            print("get type_id not macthing objName")
            return 0

        if type_id == 0:
            for i in range(len(self.CamDataVect)):  # 遍历数据列表，相机ID有没有出现过
                if self.CamDataVect[i].SeqID == objName:
                    return self.CamDataVect[i]

        if type_id == 1:
            for i in range(len(self.CoptDataVect)):  # 遍历数据列表，飞机ID有没有出现过
                if self.CoptDataVect[i].CopterID == objName:
                    return self.CoptDataVect[i]

        if type_id == 2:
            for i in range(len(self.ObjDataVect)):  # 遍历数据列表，物体ID有没有出现过
                if self.ObjDataVect[i].ObjName == objName:
                    return self.ObjDataVect[i]
        return 0

    ## @brief 请求场景内指定物体数据回传
    #  - @anchor reqCamCoptObj
    #  @param type_id
    #  @param objName
    #  @param windowID
    #  @return
    #  - 0表示未找到，返回0
    #  - 1表示找到，返回对应的对象
    def reqCamCoptObj(self, type_id=1, objName=1, windowID=0):
        '''
        type=0表示相机，1表示手动创建带copter_id的目标，2表示场景中自带的物体
        相机时，objName对应相机seqID；指定copterid目标时，objName对应CopterID，请求场景中自带的物体时，objName对应物体名字(字符串)
        windowID 表示想往哪个RflySim3D发送消息，默认是0号窗口

        RflyReqObjData(int opFlag, FString objName, FString colorflag)函数命令
        opFlag 0创建相机，1创建copter_id对应的目标，2创建物体
        '''

        # 为防止误操作，这里加上一个保护,这里在UE端判断最好，UE端
        if type_id == 0:  # 如果请求相机需要确保相机存在,在UE端处理
            pass

        if isinstance(objName, list):
            for i in range(len(objName)):
                val = objName[i]
                if (
                    type_id == 2
                    and type(val) != str
                    or type_id == 1
                    and type(val) != int
                ):
                    print(
                        "current request type_id({0}) not matching objname type({1})".format(
                            type_id, val
                        )
                    )
                    continue
                cmd = "RflyReqObjData %d %s %d" % (type_id, str(val), 0)
                self.sendUE4Cmd(cmd.encode(), windowID)
                time.sleep(0.1)
        else:
            if (
                type_id == 2
                and type(objName) != str
                or type_id == 1
                and type(objName) != int
            ):
                print(
                    "current request type_id({0}) not matching objname type({1})".format(
                        type_id, val
                    )
                )
                return
            cmd = "RflyReqObjData %d %s %d" % (type_id, str(objName), 0)
            print(cmd)
            self.sendUE4Cmd(cmd.encode(), windowID)

    def reqCam(self, SeqIDList=[0], windowID=0):
        """
        请求相机数据。

        参数:
            SeqIDList (list): 相机的序列ID列表。默认值为[0]。
            windowID (int): 发送消息的RflySim3D窗口ID，默认是0号窗口。
        """
        self.reqCamCoptObj(type_id=0, objName=SeqIDList, windowID=windowID)

    def reqCopt(self, CopterIDList=[0], windowID=0):
        """
        请求带copter_id的目标数据。

        参数:
            CopterIDList (list): copter_id的ID列表。默认值为[0]。
            windowID (int): 发送消息的RflySim3D窗口ID，默认是0号窗口。
        """
        self.reqCamCoptObj(type_id=1, objName=CopterIDList, windowID=windowID)

    def reqObj(self, ObjNameList=[''], windowID=0):
        """
        请求场景中自带的物体数据。

        参数:
            ObjNameList (list): 物体名称的列表。默认值为['']。
            windowID (int): 发送消息的RflySim3D窗口ID，默认是0号窗口。
        """
        self.reqCamCoptObj(type_id=2, objName=ObjNameList, windowID=windowID)


    ##
    # @class Radiation
    # @brief 此类用于定义和处理给定场景中各种对象的辐射属性。
    #
    # 辐射类初始化时带有预定义的对象集，每个对象都有特定的温度（以开尔文为单位），
    # 发射率（与理想黑体相比），以及一个标志表示名称是否引用角色Actor。
    # 
    # 根据预设的二维列表，计算普朗克公式的积分，得到一个辐射值，并
    # 归一到0~255范围内，发送给UE，然后UE将这个辐射值直接设置为
    # 物体的模板值，然后由 @ref CameraCapture 上写的材质规则去渲染纹理图片，然后用原来的图像传感器的接口返回到python。
    #
    class Radiation:
        ##
        # @brief 辐射类的构造函数。
        #
        # @param ue 与辐射属性关联的Unreal Engine对象。
        #
        # 使用预定义的值（包括对象名称、温度、发射率和是否为角色Actor的标志）初始化tempEmissivity数组。
        # 
        # tempEmissivity数组示例如下
        # @anchor tempEmissivity    
        def __init__(self, ue: any):
            # self.tempEmissivity = np.array([['Floor',298,0.96,True],
            #                             ['zebra',307,0.98,True],
            #                             ['rhinoceros',299,0.96,True],
            #                             ['hippopotamus',298,0.96,True],
            #                             ['crocodile',303,0.96,True],
            #                             ['human',301,0.985,True],
            #                             ['tree',293,0.952,True],
            #                             ['grass',293,0.958,True],
            #                             ['soil',288,0.914,True],
            #                             ['shrub',293,0.986,True],
            #                             ['truck',293,0.8,True],
            #                             ['water',293,0.96,True]])
            '''
            self.tempEmissivity = np.array(
                [
                        ["Floor", 276, 0.96, True],
                        ["SK_Man_Arms", 309, 0.98, False],
                        ["SK_Man_Head", 309, 0.98, False],
                ]
            )                
            这里需要预先给定场景中1：所有物体的名字（或名字中的某一段）、2：温度(开尔文)、
            3：发射率（与理想黑体）、4：是否是Actor的名字（如果是Actor的名字，那么它所有的Mesh的辐射值都会被设置，否则会设置一个Mesh）
            每个地图都会不一样，所以需要预先给定。
            
            摄氏度与开尔文的转换关系为：
            K = C + 273.15
            '''
            self.ue = ue
            self.tempEmissivity = np.array(
                [
                    ["Floor", 276, 0.96, True],
                    ["SK_Man_Arms", 309, 0.98, False],
                    ["SK_Man_Head", 309, 0.98, False],
                    ["SK_Man_Bag", 278, 0.98, False],
                    ["SK_Man_Cloth_Face", 300, 0.98, False],
                    ["SK_Man_Jacket", 283, 0.98, False],
                    ["SK_Man_Pants", 285, 0.98, False],
                    ["SK_Man_Shoes", 285, 0.98, False],
                    ["Copter", 310, 0.98, True],
                    ["Cube", 273, 0.98, True],
                    ["tree", 293, 0.952, True],
                    ["House", 295, 0.85, True],
                    ["BodyMesh_1", 297, 0.98, False],
                    ["Rotor", 285, 0.95, False],
                    ["Road", 293, 0.90, True],
                    ["Birch", 293, 0.952, True],
                    ["Door", 276, 0.96, False],
                    ["Chimney", 310, 0.98, True],
                    ["Wall", 293, 0.952, True],
                    ["Hedge", 293, 0.952, True],
                    ["Fence", 295, 0.85, True],
                    ["Fir", 293, 0.952, True],
                ]
            )

        ## @brief 由Actor名向UE发送辐射设置模板值的命令
        #  - @anchor sendUE4SetStencilValueByActorName
        #  @param Actorname Mesh组件名
        #  @param StencilValue 物体辐射模板值
        #  @param is_name_regex 是否为Actor名
        #  @param windowID RflySim3D窗口ID 
        def sendUE4SetStencilValueByActorName(
            self, Actorname, StencilValue, is_name_regex=False, windowID=-1
        ):
            cmd = f"RflySetStencilValueByActorName {Actorname} {StencilValue} {is_name_regex}"
            self.ue.sendUE4Cmd(cmd.encode(), windowID)
            time.sleep(0.1)

        ## @brief 由Actor的Mesh组件名向UE发送设置辐射模板值的命令
        #  - @anchor sendUE4SetStencilValueByMeshComponentName
        #  @param Meshname Mesh组件名
        #  @param StencilValue 物体辐射模板值
        #  @param is_name_regex 是否为Actor名
        #  @param windowID RflySim3D窗口ID
        def sendUE4SetStencilValueByMeshComponentName(
            self, Meshname, StencilValue, is_name_regex=False, windowID=-1
        ):
            cmd = f"RflySetStencilValueByMeshComponentName {Meshname} {StencilValue} {is_name_regex}"
            self.ue.sendUE4Cmd(cmd.encode(), windowID)
            time.sleep(0.1)

        ## @brief 由CopterID向UE发送设置辐射模板值的命令
        #  - @anchor sendUE4SetStencilValueByCopterID
        #  @param CopterID 物体的序号
        #  @param StencilValue 辐射模板值
        #  @param windowID RflySim3D窗口ID
        def sendUE4SetStencilValueByCopterID(self, CopterID, StencilValue, windowID=-1):
            cmd = f"RflySetStencilValueByCopterID {CopterID} {StencilValue}"
            self.ue.sendUE4Cmd(cmd.encode(), windowID)
            time.sleep(0.1)

        ## @brief 物体辐射的计算
        #  - @anchor radiance
        #   \f{eqnarray*}{
        #       L\left(T, \epsilon_{\text {avg }}, R_{\lambda}\right)=\epsilon_{\text {avg }} \int_{\lambda=8 \mu m}^{\lambda=14 \mu m} R_{\lambda}\left(\frac{2 h c^{2}}{\lambda^{5}} \frac{1}{\exp \left(\frac{h c}{k T \lambda}\right)-1}\right) d \lambda 
        #   \f}
        #  <center><b>Fomula 1:</b> 对普朗克函数在8~14μm波长上积分得到物体的辐射强度。<br>其中\f(R_\lambda \f)是相机的接受率（程序中默认为1），<br>\f(\varepsilon_{avg} \f)是物体的红外发射率（它和温度T都是 @ref tempEmissivity 预先给定的值），<br>它们俩乘以积分得到的辐射功率密度就是相机接收到的该物体的辐射功率密度。</center>
        #  
        #  @param absoluteTemperature 温度
        #  @param emissivity 发射率
        #  @param dx 用于积分的步长
        #  @param response 响应函数（默认为None）,实际是相机的接受率
        #  @return radiance 积分得到的物体辐射强度
        #  
        #  用物体的温度和发射率（提前给定的），由普朗克公式的积分计算出物体的辐射强度。
        #  
        #  普朗克函数可以计算一个黑体在某个温度下某个波长的光谱辐射度（W/m²/μm），然后光源发出的总功率密度（单位是W/m²）可以通过光谱辐照度在所有波长（或能量）上积分来计算
        #  
        def radiance(self, absoluteTemperature, emissivity, dx=0.01, response=None):
            '''
            详细实现
                1. 定义积分步长和边界 
                定义波长范围 wavelength 为8到14微米之间，以 dx 为步长。
                2. 定义普朗克公式中的常数
                定义常数 c1 和 c2：
                - c1 = 1.19104e8，是一个与普朗克常数和光速相关的常数。
                - c2 = 1.43879e4，是另一个与普朗克常数、光速和波尔兹曼常数相关的常数。
                3. 定义相机接受率
                根据是否提供了响应函数 response，计算辐射强度 radiance：
                - 如果提供了 response，则计算时会考虑响应函数的影响。
                - 否则默认response=1，直接根据普朗克公式和发射率计算辐射强度。
                4.  计算积分
                对辐射强度数组进行积分，计算总辐射强度：
                - 如果 absoluteTemperature 是多维数组，则在每个维度上分别进行积分。
                - 否则，对整个数组进行积分。
                5. 返回总辐射强度。
            '''
            wavelength = np.arange(8, 14, dx)
            c1 = 1.19104e8  # (2 * 6.62607*10^-34 [Js] *
            # (2.99792458 * 10^14 [micron/s])^2 * 10^12 to convert
            # denominator from microns^3 to microns * m^2)       乘pi结果也不会改变，因为最后还是要归一化，乘了等于没乘
            c2 = 1.43879e4  # (hc/k) [micron * K]
            if response is not None:
                radiance = (
                    response
                    * emissivity
                    * (
                        c1
                        / (
                            (wavelength**5)
                            * (np.exp(c2 / (wavelength * absoluteTemperature)) - 1)
                        )
                    )
                )
            else:
                radiance = emissivity * (
                    c1
                    / (
                        (wavelength**5)
                        * (np.exp(c2 / (wavelength * absoluteTemperature)) - 1)
                    )
                )
            if absoluteTemperature.ndim > 1:
                return radiance, np.trapz(radiance, dx=dx, axis=1)
            else:
                return radiance, np.trapz(radiance, dx=dx)

        ## @brief 由辐射强度计算模板值
        #  - @anchor get_new_temp_emiss_from_radiance
        #  @param tempEmissivity 包含物体温度和发射率的二维数组。数组的第一列是物体的ID，第二列是温度，第三列是发射率。
        #  @param response 响应函数，用于调整辐射强度的计算。默认为None，即不调整。
        #  @return tempEmissivityNew 新的二维数组，包含物体的ID和计算得到的辐射强度ID值。
        #  调用 @ref radiance 函数，获得辐射强度        
        #  然后用这个辐射强度归一化后乘以255（0~255）作为ID，这个ID传给UE后会作为物体的stencil值，UE根据这个值来显示物体的辐射强度
        #  这个值越大，物体越亮，反之越小越暗
        # 
        #  <img src="IR.png" align="left">
        #  <div style="clear: both"></div>
        # 
        # 这个值越大，物体越亮，反之越小越暗
        def get_new_temp_emiss_from_radiance(self, tempEmissivity, response):
            '''
            详细实现
                1. 获取物体的数量 numObjects。
                2. 调用 radiance 函数计算每个物体的辐射强度：
                   - 提取 tempEmissivity 数组中的温度和发射率，并转换为浮点数。
                   - 使用提取的温度和发射率调用 radiance 函数，得到每个物体的辐射强度数组 L。
                3. 对辐射强度数组 L 进行归一化处理，并将其缩放到0到255的范围内，转换为8位无符号整数类型。
                4. 构建新的二维数组 tempEmissivityNew，包含物体的ID和归一化后的辐射强度ID值。
                5. 返回新的二维数组 tempEmissivityNew。
            '''
            numObjects = tempEmissivity.shape[0]

            L = self.radiance(
                tempEmissivity[:, 1].reshape((-1, 1)).astype(np.float64),
                tempEmissivity[:, 2].reshape((-1, 1)).astype(np.float64),
                response=response,
            )[1].flatten()
            L = ((L / L.max()) * 255).astype(np.uint8)

            tempEmissivityNew = np.hstack(
                (
                    tempEmissivity[:, 0].reshape((numObjects, 1)),
                    L.reshape((numObjects, 1)),
                )
            )
            return tempEmissivityNew

    ## @brief 将 @ref Radiation 类计算得到目标的辐射值设置给UE（只要名字中某段能匹配就会设置）
    #  - @anchor SetUE4RadianceValue
    #  @param UEObjectName 目标物体的名称（指向mesh或actor）
    #  @param windows
    #  通过 @ref get_new_temp_emiss_from_radiance 获取物体的ID（0~255），然后这个ID传给UE，作为物体的自定义模板值，然后UE根据ID和颜色的映射表，
    #  给出该物体的颜色（如果是分割图），或者直接由ID给出一个灰度图（如果是红外图）。 
    #
    #  红外图是根据物体的温度和发射率计算出的辐射功率密度的灰度图，它反映了物体在红外波段的辐射特性。分割图是根据物体的ID和颜色的映射表生成的彩色图，它可以区分不同的物体，并提供物体的位置信息。
    #  
    #  每个颜色占用4个像素，所以一共是256种颜色，对应0~255的ID，ID与颜色的映射关系就是简单的从左到右的顺序对应：Color= palette[0, (ID+1)*4-1]。
    #  @image html COLORMAP.png "ID与颜色映射表，一共是1×1024个像素"  
    def SetUE4RadianceValue(self, UEObjectName, windows=-1):
        response = None
        self.radiation = self.Radiation(self)
        tempEmissivityNew = radiation.get_new_temp_emiss_from_radiance(
            radiation.tempEmissivity, response
        )
        index = np.where(radiation.tempEmissivity[:, 0] == UEObjectName)
        key = tempEmissivityNew[index][0]
        if tempEmissivityNew[index][0][3] == "True":
            radiation.sendUE4SetStencilValueByActorName(
                "[\\w]*" + key + "[\\w]*", tempEmissivityNew[index][1], True, windows
            )
        else:
            radiation.sendUE4SetStencilValueByMeshComponentName(
                "[\\w]*" + key + "[\\w]*", tempEmissivityNew[index][1], True, windows
            )

    ## @brief 设置 @ref tempEmissivity 数组中所有物体的辐射值给UE4
    #  - @anchor SetUE4RadianceValue__All
    #  @param windows
    #  与 @ref SetUE4RadianceValue 一样，但它设置 Radiation.tempEmissivity 中所有物体的辐射值
    def SetUE4RadianceValue__All(self, windows=-1):
        response = None
        radiation = self.Radiation(self)
        tempEmissivityNew = radiation.get_new_temp_emiss_from_radiance(
            radiation.tempEmissivity, response
        )
        print(tempEmissivityNew)
        # index = np.where(radiation.tempEmissivity[:, 0] == UEObjectName)
        for index, element in enumerate(tempEmissivityNew):
            key = tempEmissivityNew[index][0]
            print(f"key:{key},{radiation.tempEmissivity[index][3]}")
            if radiation.tempEmissivity[index][3] == "True":
                radiation.sendUE4SetStencilValueByActorName(
                    "[\\w]*" + key + "[\\w]*",
                    tempEmissivityNew[index][1],
                    True,
                    windows,
                )
                print("Actor!!")
            else:
                radiation.sendUE4SetStencilValueByMeshComponentName(
                    "[\\w]*" + key + "[\\w]*",
                    tempEmissivityNew[index][1],
                    True,
                    windows,
                )
                print("Mesh!!!")

    ## @brief 将物体的模板值发送给UE（由Actor）
    #  - @anchor sendUE4SetStencilValueByActorName
    #  @param Actorname Actor名字
    #  @param StencilValue 物体的模板值
    #  @param is_name_regex 物体是否是Actor
    #  @param windowID RflySim3D窗口ID    
    def sendUE4SetStencilValueByActorName(
        self, Actorname, StencilValue, is_name_regex=False, windowID=-1
    ):
        cmd = f"RflySetStencilValueByActorName {Actorname} {StencilValue} {is_name_regex}"
        self.sendUE4Cmd(cmd.encode(), windowID)
        time.sleep(0.1)

    ## @brief 将物体的模板值发送给UE（由mesh）
    #  - @anchor sendUE4SetStencilValueByMeshComponentName
    #  @param Meshname MeshComponent名字
    #  @param StencilValue 物体的模板值
    #  @param is_name_regex 物体是否是Actor
    #  @param windowID RflySim3D窗口ID
    def sendUE4SetStencilValueByMeshComponentName(
        self, Meshname, StencilValue, is_name_regex=False, windowID=-1
    ):
        cmd = f"RflySetStencilValueByMeshComponentName {Meshname} {StencilValue} {is_name_regex}"
        self.sendUE4Cmd(cmd.encode(), windowID)
        time.sleep(0.1)

    ## @brief 将物体的模板值发送给UE（由CopterID）
    #  - @anchor sendUE4SetStencilValueByCopterID
    #  @param CopterID
    #  @param StencilValue
    #  @param windowID        
    def sendUE4SetStencilValueByCopterID(self, CopterID, StencilValue, windowID=-1):
        cmd = f"RflySetStencilValueByCopterID {CopterID} {StencilValue}"
        self.sendUE4Cmd(cmd.encode(), windowID)
        time.sleep(0.1)
