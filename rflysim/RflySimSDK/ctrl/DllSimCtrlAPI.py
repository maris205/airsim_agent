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
import ctypes
import EarthModel
import UE4CtrlAPI
import UEMapServe
import shutil

## @file
#  @brief 这是一个集成与CopterSim中dll模型交互接口的模块。
#  @anchor DllSimCtrl接口库文件
#  对应例程链接见
#  @ref md_ctrl_2md_2DllSimCtrl


##  @brief 仿真绝对数据接口类
#  
#   用于存储和更新与载具（如无人机）相关的三维运动和状态数据。
class Data3D:
    ## @brief 构造函数
    #  - @anchor __init__
    #  @param copter_data 默认值为 None，是一个包含飞行器数据的列表。
    #  @param CopterID  飞行器的ID，默认值为 -1。
    #  @param ClassID  载具三维样式ID，默认值为 -1。
    def __init__(self, copter_data=None,CopterID=-1,ClassID=-1):
        """             
        解析
            如果提供了 copter_data，则调用 update 方法进行初始化。
            否则，初始化所有成员变量为默认值。
            初始化的成员变量包括飞行器ID、三维样式、运行时间、速度、位置、姿态（欧拉角和四元数）、电机转速、加速度、角速度、GPS位置以及一些保留数据。
        """
        if copter_data != None:
            self.update(copter_data,CopterID,ClassID)
            return

        self.CopterID = 0
        self.vehicleType = 0
        self.runnedTime = 0
        self.VelE = [0,0,0]
        self.PosE = [0,0,0]
        self.AngEuler = [0,0,0]
        self.AngQuatern = [1,0,0,0]
        self.MotorRPMS = [0]*8
        self.AccB = [0,0,0]
        self.RateB = [0,0,0]
        self.PosGPS = [0,0,0]
        self.reserved = [0]*7
        
    ## @brief 由copter_data更新状态变量
    #  - @anchor update
    #  @param copter_data 默认值为 None，是一个包含飞行器数据的列表。
    #  @param CopterID  飞行器的ID，默认值为 -1。
    #  @param ClassID  载具三维样式ID，默认值为 -1。
    def update(self, copter_data,CopterID=-1,ClassID=-1):
        """             
        解析
            根据传入的 CopterID 和 ClassID，决定是否使用 copter_data 中的数据更新三维样式和飞机ID。
            更新所有的状态变量：包括运行时间、速度、位置、姿态（欧拉角和四元数）、电机转速、加速度、角速度、GPS位置以及保留数据。
        """

        if CopterID<=0:
            self.CopterID = int(copter_data[0])
        else:
            self.CopterID = int(CopterID)
        if ClassID<=0:
            self.vehicleType = int(copter_data[1])
        else:
            self.vehicleType = int(ClassID)
        self.runnedTime = copter_data[2]
        self.VelE = copter_data[3:6]
        self.PosE = copter_data[6:9]
        self.AngEuler = copter_data[9:12]
        self.AngQuatern = copter_data[12:16]
        self.MotorRPMS = copter_data[16:24]
        self.AccB = copter_data[24:27]
        self.RateB = copter_data[27:30]
        self.PosGPS = copter_data[30:33]
        self.reserved = copter_data[33:60]
        
    ## @brief 由UIV更新状态变量
    #  - @anchor updateUdp
    #  @param UIV 是一个包含飞行器更新数据的列表。
    def updateUdp(self, UIV):
        self.VelE[0]=UIV[4]
        self.VelE[1]=UIV[5]
        self.VelE[2]=UIV[6]
        self.AngEuler[0]=UIV[7]
        self.AngEuler[1]=UIV[8]
        self.AngEuler[2]=UIV[9]
        self.AngQuatern[0]=UIV[10]
        self.AngQuatern[1]=UIV[11]
        self.AngQuatern[2]=UIV[12]
        self.AngQuatern[3]=UIV[13]
        self.MotorRPMS[0]=UIV[14]
        self.MotorRPMS[1]=UIV[15]
        self.MotorRPMS[2]=UIV[16]
        self.MotorRPMS[3]=UIV[17]
        self.MotorRPMS[4]=UIV[18]
        self.MotorRPMS[5]=UIV[19]
        self.MotorRPMS[6]=UIV[20]
        self.MotorRPMS[7]=UIV[21]
        self.AccB[0]=UIV[22]
        self.AccB[1]=UIV[23]
        self.AccB[2]=UIV[24]
        self.RateB[0]=UIV[25]
        self.RateB[1]=UIV[26]
        self.RateB[2]=UIV[27]
        self.runnedTime = UIV[28]
        self.PosE[0]=UIV[29]
        self.PosE[1]=UIV[30]
        self.PosE[2]=UIV[31]
        self.PosGPS[0]=UIV[32]
        self.PosGPS[1]=UIV[33]
        self.PosGPS[2]=UIV[34]
    
##  @brief 飞机相对数据接口类
#  
#   用于用于存储与硬件在环仿真（HIL）相关的数据。
class DataHIL:
    def __init__(self):
        self.gpsHome = [0,0,0]
        self.AngEular = [0,0,0]
        self.localPos = [0,0,0]
        self.localVel = [0,0,0]
        


##  @brief 类集成了调用受控模型的DLL接口的方法。
#  
#   此类通过UDP协议与无人机模拟器进行通信，控制模拟器的操作。

class DllSimCtrlAPI:

    ## @brief 构造函数初始化无人机控制接口。
    #  - @anchor __init__
    #  @param CopterID 初始化时设置的无人机ID，默认值为1。
    #  @param ip 控制器连接的目标IP地址，默认值为'127.0.0.1'。
    def __init__(self, CopterID=1, ip='127.0.0.1'):
        """            
            
        
        解析
            self.ip = ip：
            这行代码将ip参数赋值给self.ip。self指的是对象本身，ip是传入的参数。这表示该对象有一个属性ip，用于存储IP地址。

            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)：
            这行代码创建了一个套接字对象，并将其赋值给self.udp_socket。socket.socket(socket.AF_INET, socket.SOCK_DGRAM)用于创建一个基于IPv4和UDP协议的套接字。

            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)：
            这行代码设置了套接字对象的一个选项。它允许套接字发送广播消息。

            self.CopterID = CopterID：
            这行代码将CopterID参数赋值给self.CopterID。这表示该对象有一个属性CopterID，用于存储无人机的ID。
            
        """
        self.ip = ip
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.CopterID = CopterID
        
        self.out3Ddata=Data3D()
        self.outCopterVect=[0]*32


    ## @brief 根据需要的长度调整数据列表，以确保其长度符合要求。
    #  - @anchor fillList
    #  @param data 输入的原始数据，可以是numpy数组或列表（维度不定）。
    #  @param inLen 期望的数据列表维度。
    #  @return 调整后符合长度要求的数据列表。
    def fillList(self, data, inLen):
        """ 
            
            此方法接收数组或列表，根据提供的长度参数进行调整。如果数据长度不足，则填充0；如果数据长度过长，则截断。
 
            
        解析    
            if isinstance(data, np.ndarray):
            首先检查输入的数据data是否为NumPy数组。如果是，将其转换为Python列表。

            if isinstance(data, list) and len(data)==inLen
            接下来检查数据的类型和长度。如果数据是列表且长度与预期的inLen相等，则直接返回数据。

            if isinstance(data, list):
            如果数据是列表，则进一步检查数据的长度。如果列表的长度小于inLen，则在列表末尾添加足够的零，使其长度等于inLen。如果列表的长度大于inLen，则仅保留列表的前inLen个元素。

            else:
            如果数据不是列表，说明数据可能是其他类型（例如整数、浮点数等）。此时将该数据包装成一个列表，并在列表中添加足够的零，使其长度等于inLen。

            return data
            最后返回处理后的数据。    
        """
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

    ## @brief 发送一个包含16维double型数据的自定义数据结构到指定的IP和端口，可用于与Simulink通信。
    #  - @anchor sendCustomData
    #  @ref sendCustomData
    #  @param  CopterID (int): 飞行器的标识ID，默认为0。
    #  @param  data (list or numpy.array): 可以是1到16维的向量，自定要发送的数据，默认为16个0。      
    #  @param  checksum (int): 用于校验的检验和，默认为123456。      
    #  @param  port (int): 目标端口号，默认为50000。      
    #  @param  IP (str): 目标IP地址，默认为'127.0.0.1'。      
    def sendCustomData(self, CopterID=0, data=[0]*16, checksum=123456, port=50000, IP='127.0.0.1'):
        """
                    

        Description:
            本方法封装了一个结构体 CustomData，该结构体包含一个整型校验和，一个整型飞行器ID和一个包含16个double类型数据的数组。
            方法首先确保提供的数据向量长度为16，不足时自动补零。
            使用 struct.pack 方法按照 "ii16d" 的格式打包数据，其中 'ii' 表示两个整型数据，'16d' 表示16个双精度浮点数。
            然后通过 UDP socket 发送打包后的数据到指定的 IP 地址和端口。
        
        Note:
            调用此方法前需确保已创建并配置好 udp_socket 对象。
        """
        # 确保数据列表长度为16，不足部分将使用0补齐
        data = self.fillList(data, 16)
        # 将校验和、飞行器ID和数据打包成符合预设格式的二进制数据
        buf = struct.pack("ii16d", checksum, CopterID, *data)
        # 通过UDP协议发送数据包到指定的IP和端口
        self.udp_socket.sendto(buf, (IP, port))

    ## @brief 向 DLL 模型的inSILInts and inSILFLoats接口发送包含整数和浮点数组的数据。
    #  - @anchor sendSILIntFloat
    #  @param   inSILInts (list or numpy.array): 1到8维整数数组，自定数据，默认为8个0。
    #  @param   inSILFLoats (list or numpy.array): 1到20维浮点数数组，自定数据，默认为20个0。
    #  @param   copterID (int): 指定要发送数据的飞行器ID，默认为-1。
    def sendSILIntFloat(self, inSILInts=[0]*8, inSILFLoats=[0]*20, copterID=-1):
        """
        
        Description:
            该函数作用为向DLL模型的inSILInts和inSILFloats接口传入数据。

            原理上，该函数封装了一个PX4SILIntFloat结构体的调用方法，结构体定义见下方，该结构体对应了DLL模型的inSILInts
            和inSILFloats输入接口。结构体中包含了整形校验和checksum、飞行器序号CopterID、整型数组inSILInts[8]
            和浮点型数组inSILFLoats[16]。

            函数内容包括：以PX4SILIntFloat结构体的校验和传入checksum，函数预留copterID参数传入位，当函数传入的CopterID
            大于0时，函数采用传入的CopterID，否则取对象本身的CopterID。在RflySim平台中，PX4SILIntFloat结构体通过
            30100+(copterID-1)*2 UDP端口号通讯（copterID从1开始计数）。将函数传入的inSILInts和inSILFLoats进行长度
            校正后将结构体相关数据按10i20f的格式打包，并通过UDP的方式发出，其中 '10i' 表示10个整型数据（包括校验和和ID），'20f' 表示
            20个单精度浮点数。

        结构体定义：
            struct PX4SILIntFloat{
                int checksum;//1234567897
                int CopterID;
                int inSILInts[8];
                float inSILFLoats[16];
            };
            
        Note:
            调用此方法前需确保已创建并配置好 udp_socket 对象及 self.ip 属性已被设定。
        """
        # 设定校验和
        checkSum = 1234567897
        # 确定飞行器ID
        ID = copterID if copterID > 0 else self.CopterID
        # 计算端口号
        PortNum = 30100 + (ID - 1) * 2
        # 校正整数和浮点数数组的长度
        inSILInts_0 = self.fillList(inSILInts, 8)
        inSILInts=[0]*8
        for i in range(8):
            inSILInts[i]=inSILInts_0[i]
        
        inSILFLoats = self.fillList(inSILFLoats, 20)
        # 打包数据
        buf = struct.pack("10i20f", checkSum, ID, *inSILInts, *inSILFLoats)
        # 发送数据
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    ## @brief 向 DLL 模型的inDoubCtrls接口发送包含整数和双精度浮点数组的数据。
    #  - @anchor sendSILIntDouble
    #  @param inSILInts (list): 支持1-8维的numpy或list向量
    #  @param inSILDoubs (list): 支持1-20维的numpy或list向量
    #  @param  copterID (int): 指定要发送数据的飞行器ID，默认为-1。     
    #  
    def sendSILIntDouble(self, inSILInts=[0]*8, inSILDoubs=[0]*20, copterID=-1):
        """            

        Description:
            该函数作用为向DLL模型的inDoubCtrls接口传入数据，数据格式为前8维整型，后8维双精度浮点型。

            原理上，该函数封装了一个DllInDoubCtrls结构体的调用方法，结构体定义见下方，该结构体对应了DLL模型的inDoubCtrls输入接口。
            结构体中包含了整形校验和checksum、飞行器序号CopterID、双精度浮点型数组inDoubCtrls[28]。

            函数内容包括：以DllInDoubCtrls结构体的校验和传入checksum，函数预留copterID参数传入位，当函数传入的CopterID大于0时，
            函数采用传入的CopterID，否则取对象本身的CopterID。在RflySim平台中，DllInDoubCtrls结构体通过
            30100+(copterID-1)*2 UDP端口号通讯（copterID从1开始计数）。将函数传入的inSILInts和inSILDoubs进行长度
            校正后将结构体相关数据按2i28d的格式打包，并通过UDP的方式发出，其中 '2i' 表示2个整型数据（校验和和ID），'28d' 表示28个双精度浮点数。
            这里以整型+双精度浮点型数据传入参数实际发出的还是28维double型，这样做的目的是为了兼容PX4SILIntFloat结构体前8维整型后20维
            浮点型的数据格式。

        结构体定义：
            struct DllInDoubCtrls{
                int checksum;//校验码1234567897
                int CopterID; // 飞机的ID
                double inDoubCtrls[28];//28维的double型输入
            };

        Note:
            调用此方法前需确保已创建并配置好 udp_socket 对象及 self.ip 属性已被设定。
        """
        # 设定校验和
        checkSum = 1234567897
        # 确定飞行器ID
        ID = copterID if copterID > 0 else self.CopterID
        # 计算端口号
        PortNum = 30100 + (ID - 1) * 2
        # 校正整数和双精度浮点数数组的长度
        inSILInts = self.fillList(inSILInts, 8)
        inSILDoubs = self.fillList(inSILDoubs, 20)
        # 打包数据
        buf = struct.pack("2i28d", checkSum, ID, *inSILInts,*inSILDoubs)
        # 发送数据
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    ## @brief 发送一个包含双精度浮点数数组的数据包到模型inDoubCtrls接口，通过UDP。
    #  - @anchor sendInDoubCtrls
    #  @param inDoubsCtrls (list 或 numpy.array): 1-28维包含双精度浮点数的numpy或list向量。默认为28个零。
    #  @param copterID (int): 要发送数据的无人机ID。如果为-1，则默认使用当前实例的无人机ID。       
    def sendInDoubCtrls(self, inDoubsCtrls=[0]*28, copterID=-1):
        """            

        Description:
            该函数作用为向DLL模型的inDoubCtrls传入数据，数据格式为28维double型。

            原理上，该函数封装了一个DllInDoubCtrls结构体的调用方法，结构体定义见下方，该结构体对应了DLL模型的inDoubCtrls输入接口。
            结构体中包含了整形校验和checksum、飞行器序号CopterID、双精度浮点型数组inDoubCtrls[28]。

            函数内容包括：以DllInDoubCtrls结构体的校验和传入checksum，函数预留copterID参数传入位，当函数传入的CopterID大于0时，
            函数采用传入的CopterID，否则取对象本身的CopterID。在RflySim平台中，DllInDoubCtrls结构体通过30100+(copterID-1)*2 UDP
            端口号通讯（copterID从1开始计数）。将函数传入的inSILDoubs进行长度校正后将结构体相关数据按2i28d的格式打包，并通过UDP的方式发出，
            其中 '2i' 表示2个整型数据（校验和和ID），'28d' 表示28个双精度浮点数。

        结构体定义：
            struct DllInDoubCtrls{
                int checksum;//校验码1234567897
                int CopterID; // 飞机的ID
                double inDoubCtrls[28];//28维的double型输入
            };

        注意:
            确保在调用此方法前已创建并适当配置了udp_socket对象，并且设置了self.ip属性。
        """
        checkSum = 1234567897  # 预定义的常量校验和
        ID = self.CopterID if copterID <= 0 else copterID  # 使用提供的copterID或默认到自己的
        PortNum = 30100 + (ID - 1) * 2  # 计算端口号
        inDoubsCtrls = self.fillList(inDoubsCtrls, 28)  # 确保列表正好28个元素
        buf = struct.pack("2i28d", checkSum, ID, *inDoubsCtrls)  # 打包数据
        self.udp_socket.sendto(buf, (self.ip, PortNum))  # 发送数据

    ## @brief 通过UDP发送控制信号到指定无人机的模拟器。
    #  - @anchor sendInCtrlExt
    #  @param inSILInts (list 或 numpy.array): 1到8维包含整数的向量。
    #  @param inSILFLoats (list 或 numpy.array): 1到20维包含浮点数的向量。
    #  @param iDxNum (int): 指定使用哪一个InCtrlExt端口（1到5之间）。
    #  @param copterID (int): 目标无人机ID。如果为-1，则发送到自身ID。
    def sendInCtrlExt(self, inSILInts=[0]*8, inSILFLoats=[0]*20, iDxNum=1, copterID=-1):
        """
        描述:
            该函数作用为向DLL模型inCtrlExt 1-5系列接口中的指定接口传入数据，接口由iDxNum决定，iDxNum=1时表示向DLL模型
            inCtrlExt 1接口传入数据，以此类推，数据类型为8维整型+20维单精度浮点型。

            原理上，该函数调用的结构体格式与DllInDoubCtrls一致，区别在于结构体中的checksum，inCtrlExt 1-5接口对应结构体的checksum
            为1234567800 + iDxNum，另外传入的数据精度及格式也存在区别，该函数传入参数中数据类型为8维整型+20维单精度浮点型，实际发给
            DLL模型为28维double型（注意，DLL模型inCtrlExt系列接口默认设置为float型）。

            函数内容包括，以1234567800 + iDxNum作为校验和传入checksum，函数预留copterID参数传入位，当函数传入的CopterID
            大于0时，函数采用传入的CopterID，否则取对象本身的CopterID。在RflySim平台中，该结构体通过
            30100+(copterID-1)*2 UDP端口号通讯（copterID从1开始计数）。将函数传入的inSILInts和inSILFLoats进行长度
            校正后将结构体相关数据按10i20f的格式打包，并通过UDP的方式发出，其中 '10i' 表示10个整型数据（包括校验和和ID），'20f' 表示
            20个单精度浮点数。
            
        """
        checkSum = 1234567800 + iDxNum  # 基于端口号iDxNum动态计算校验和
        ID = self.CopterID if copterID <= 0 else copterID  # 判断是否使用当前实例的无人机ID
        PortNum = 30100 + (ID - 1) * 2  # 根据无人机ID计算端口号

        inSILInts = self.fillList(inSILInts, 8)  # 确保整数数组长度为8
        inSILFLoats = self.fillList(inSILFLoats, 20)  # 确保浮点数数组长度为20

        buf = struct.pack("10i20f", checkSum, ID, *inSILInts, *inSILFLoats)  # 打包数据
        self.udp_socket.sendto(buf, (self.ip, PortNum))  # 发送数据

    ## @brief 通过UDP发送double型控制信号到指定无人机的模拟器端口。
    #  - @anchor sendInCtrlExtDoub
    #  @param inDoubsCtrls (list 或 numpy.array): 1到28维包含double的向量。
    #  @param iDxNum (int): 指定使用哪一个InCtrlExt端口（1到5之间）。
    #  @param copterID (int): 目标无人机ID。如果为-1，则发送到自身ID。
    #  @param update (bool): 是否立即更新
    def sendInCtrlExtDoub(self, inDoubsCtrls=[0]*28, iDxNum=1, copterID=-1, update=True):
        """            

        描述:
            该函数作用为向DLL模型inCtrlExt 1-5系列接口中的指定接口传入数据，接口由iDxNum决定，iDxNum=1时表示向DLL模型
            inCtrlExt 1接口传入数据，以此类推，数据类型为28维双精度浮点型。该函数与sendInCtrlExt()的主要区别在于传入DLL模型
            数据精度与数据格式。

            原理上，该函数调用的结构体格式与DllInDoubCtrls一致，区别在于结构体中的checksum，inCtrlExt 1-5接口对应结构体的checksum
            为1234567800 + iDxNum，传入inDoubsCtrls参数数据类型为28维double型，该数据会传入到DLL模型的inCtrlExt接口。（注意，
            DLL模型inCtrlExt系列接口默认设置为float型，在使用该函数给DLL模型传入数据时，需要将DLL模型inCtrlExt接口数据类型修改为double后重新
            生成DLL函数才会生效）。

            函数内容包括，以1234567800 + iDxNum作为校验和传入checksum，函数预留copterID参数传入位，当函数传入的CopterID
            大于0时，函数采用传入的CopterID，否则取对象本身的CopterID。在RflySim平台中，该结构体通过
            30100+(copterID-1)*2 UDP端口号通讯（copterID从1开始计数）。将函数传入的inDoubsCtrls进行长度
            校正后将结构体相关数据按2i28d的格式打包，并通过UDP的方式发出，其中 '2i' 表示2个整型数据（校验和和ID），'28d' 表示28个双精度浮点数。
            
        """
        if iDxNum >= 1 and iDxNum <= 5:  # 确认端口号在有效范围内
            checkSum = 1234567800 + iDxNum  # 计算校验和

        if not update:
            checkSum += 100  # 修改校验和以表示不立刻更新状态

        ID = self.CopterID if copterID <= 0 else copterID  # 设置目标无人机ID
        PortNum = 30100 + (ID - 1) * 2  # 计算端口号

        inDoubsCtrls = self.fillList(inDoubsCtrls, 28)  # 调整向量维度

        buf = struct.pack("2i28d", checkSum, ID, *inDoubsCtrls)  # 打包数据
        self.udp_socket.sendto(buf, (self.ip, PortNum))  # 发送数据

    ## @brief 将一个较长的浮点数向量分段并通过UDP发送到多个控制端口。
    #  - @anchor sendInCtrlExtAll
    #  @param inDoubsCtrls (list 或 numpy.array): 1到140维包含double的向量。
    #  @param copterID (int): 目标无人机ID。如果为-1，则发送到self里的ID。    
    def sendInCtrlExtAll(self, inDoubsCtrls=[0]*140, copterID=-1):
        """
        描述:
            该函数基于sendInCtrlExtDoub()实现，作用为支持DLL模型inCtrlExt 1-5接口数据输入，会将传入的inDoubsCtrls参数按照
            长度分段通过inCtrlExt 1-5接口传入到DLL模型，每段最多28个元素，从inCtrlExt 1 接口开始发送。最后一个端口发送时，
            设置update为True以立即更新，其他端口设置为False。

            具体原理见sendInCtrlExtDoub()函数描述。
        """
        lenCtrl = min(math.ceil(len(inDoubsCtrls) / 28.0), 5)  # 计算需要使用的端口数量，最多5个
        inDoubsCtrls = self.fillList(inDoubsCtrls, 140)  # 确保向量长度为140

        for i in range(lenCtrl):
            segment = inDoubsCtrls[28 * i:28 * (i + 1)]
            update = (i == lenCtrl - 1)
            self.sendInCtrlExtDoub(segment, i + 1, copterID, update)

    ## @brief 通过UDP向CopterSim模型发送仿真参数。
    #  - @anchor sendModelInParams
    #  @param Bitmask (uint32): 一个32位无符号的位掩码，指定各种标志或设置。
    #  @param InParams (list或numpy.array): 32维包含双精度浮点数的列表或数组。
    #  @param copterID (int): 可选；指定目标无人机ID。如果为-1或未提供，则默认为self.CopterID。
    def sendModelInParams(self, Bitmask, InParams, copterID=-1):
        """
        描述:
            该函数作用为向DLL模型FaultParamAPI.FaultInParams接口传入数据，其中，参数InParams为使用者需要传入DLL模型FaultInParams接口的参数，
            数据类型为double；参数Bitmask为32位无符号的位掩码，用来指定传入到DLL模型FaultInParams接口参数生效的位数，比如当Bitmask为0b01时，
            传入到DLL模型的FaultInParams参数仅第1位有效。

            该函数调用的结构体如下，checksum要求为1234567891。函数预留copterID参数传入位，当函数传入的CopterID大于0时，函数采用传入的
            CopterID，否则取对象本身的CopterID。通过30100+(copterID-1)*2 UDP端口号通讯（copterID从1开始计数）。将函数传入的InParams进行
            长度校正后将结构体相关数据按iI32d的格式打包，并通过UDP的方式发出，其中 'i' 表示1个整型数据（校验位），'I'表示无符号整型，'32d' 
            表示32个双精度浮点数。

            struct PX4ModelInParams{
                int checksum;//1234567891 for PX4ModelInParams, and 1234567892 for PX4InitInParams
                uint32_t Bitmask;
                double InParams[32];
            };

        """
        checkSum = 1234567891  # 预定义的校验和值
        ID = copterID if copterID > 0 else self.CopterID  # 确定目标无人机ID
        PortNum = 30100 + (ID - 1) * 2  # 根据无人机ID计算UDP端口号

        InParams = self.fillList(InParams, 32)  # 确保InParams正好有32个元素

        # 创建要发送的二进制数据包
        buf = struct.pack("iI32d", checkSum, Bitmask, *InParams)  # 按指定格式打包数据

        # 将打包好的数据发送到指定的IP和端口
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    ## @brief 发送初始化参数至CopterSim
    #  - @anchor fillList
    #  @param Bitmask (int, 可选): 一个32位整数，表示要修改的参数。默认值为0。
    #  @param InParams (list, 可选): 32维包含double的列表，代表初始化参数。默认值为[0]*32。
    #  @param copterID (int, 可选): 要发送参数的CopterSim的ID。默认值为-1，这意味着参数将发送至存储在实例变量`CopterID`中的CopterSimID。
    def sendInitInParams(self, Bitmask=0, InParams=[0]*32, copterID=-1):
        """
        描述:
            该函数作用为向DLL模型FaultParamAPI.InitInParams接口传入数据，其中，参数InParams为使用者需要传入DLL模型FaultInParams接口的参数，
            数据类型为double；参数Bitmask为32位无符号的位掩码，用来指定传入到DLL模型InitInParams接口参数生效的位数，比如当Bitmask为0b01时，
            传入到DLL模型的InitInParams参数仅第1位有效。

            该函数调用的结构体如下，checksum要求为1234567892。函数预留copterID参数传入位，当函数传入的CopterID大于0时，函数采用传入的
            CopterID，否则取对象本身的CopterID。通过30100+(copterID-1)*2 UDP端口号通讯（copterID从1开始计数）。将函数传入的InParams进行
            长度校正后将结构体相关数据按iI32d的格式打包，并通过UDP的方式发出，其中 'i' 表示1个整型数据（校验位），'I'表示无符号整型，'32d' 
            表示32个双精度浮点数。

            struct PX4ModelInParams{
                int checksum;//1234567891 for PX4ModelInParams, and 1234567892 for PX4InitInParams
                uint32_t Bitmask;
                double InParams[32];
            };          
            
        注意:
            - 在此实现中，`checkSum` 参数被设置为1234567892。
            - `PortNum` 基于`copterID`计算。
            - 使用`struct.pack`方法创建`buf`参数。
            - 使用`udp_socket.sendto`方法发送数据。

        """
        checkSum = 1234567892
        ID = copterID
        if copterID <= 0:
            ID = self.CopterID
        PortNum = 30100 + (ID - 1) * 2
        InParams = self.fillList(InParams, 32)
        buf = struct.pack("ii32d", checkSum, Bitmask, *InParams)
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    ## @brief 发送动态参数修改至指定CopterSim模拟器
    #  - @anchor sendDynModiParams
    #  @param Bitmask (int): 一个32位整数，决定`InParams`中的哪些参数将被修改。每个位代表一个不同的参数，如果一个位被设置为1，那么对应的参数将被修改。
    #  @param InParams (list): 64维包含double的列表，代表要修改的参数。
    #  @param copterID (int): 发送参数至哪个CopterSim。如果`copterID`小于或等于0，参数将被发送到与`DLLSimCtrlAPI`实例关联的Copter。
    def sendDynModiParams(self, Bitmask=0, InParams=[0]*64, copterID=-1):
        """
        描述:
            该函数作用为向DLL模型FaultParamAPI.DynModiParams接口传入数据，其中，参数InParams为使用者需要传入DLL模型FaultInParams接口的参数，
            数据类型为double；参数Bitmask为64位无符号的位掩码，用来指定传入到DLL模型DynModiParams接口参数生效的位数，比如当Bitmask为0b01时，
            传入到DLL模型的DynModiParams参数仅第1位有效。

            该函数调用的结构体如下，checksum要求为1234567893。函数预留copterID参数传入位，当函数传入的CopterID大于0时，函数采用传入的
            CopterID，否则取对象本身的CopterID。通过30100+(copterID-1)*2 UDP端口号通讯（copterID从1开始计数）。将函数传入的InParams进行
            长度校正后将结构体相关数据按iiQ64d的格式打包，并通过UDP的方式发出，其中 'ii' 表示2个整型数据（校验位），'Q'表示无符号双长整型，'64d' 
            表示64个双精度浮点数。

            struct PX4DynModiParams{
                int checksum;//1234567893
                int CopterID;
                uint64_t Bitmask;
                double InParams[64];
            };    
        
        """
        checkSum = 1234567893  # A fixed checksum value for the data packet
        ID = copterID
        if copterID <= 0:
            ID = self.CopterID
        PortNum = 30100 + (ID - 1) * 2  # Calculate the port number based on the copter ID
        InParams = self.fillList(InParams, 64)  # Ensure that the InParams list has exactly 64 elements
        buf = struct.pack("iiQ64d", checkSum, copterID, Bitmask, *InParams)  # Pack the data into a binary format suitable for sending over UDP
        self.udp_socket.sendto(buf, (self.ip, PortNum))  # Send the data packet to the specified IP and port

    ## @brief 用于处理UE（三维环境）和CopterSim DLL模型之间的通信。
    #  - @anchor sendUE2Coptersim
    #  @param inFromUE (list of float, optional): 32维双精度浮点数列表。RflySim3D发送到CopterSim的数据
    #  @param copterID (int, optional): 将数据发送到的无人机的ID。如果小于或等于零，则发送到初始化时设置的无人机ID。
    def sendUE2Coptersim(self, inFromUE=None, copterID=-1):
        """

        根据无人机ID通过UDP将结构化数据打包并发送到dll模型。
        struct UEToCopterDataD{
            int checksum; //1234567899为校验ID
            int CopterID; //发出本消息的飞机ID
            double inFromUE[32]; //通过蓝图发出的数据
        }
        struct.pack ii32d            
            

        示例:
            dll = DllSimCtrlAPI.DllSimCtrlAPI()
            dll.sendUE2Coptersim([0.1]*32,1)
        """
        if inFromUE is None:
            inFromUE = [0.0] * 32
        checkSum = 1234567899  # 唯一校验标识符
        ID = copterID if copterID > 0 else self.CopterID
        portNum = 30100 + (ID - 1) * 2  # 根据无人机ID计算端口号
        buf = struct.pack("ii32d", checkSum, ID, *inFromUE)  # 使用struct打包数据
        self.udp_socket.sendto(buf, (self.ip, portNum))


    ## @brief 初始化真实数据接收机制
    #  - @anchor InitTrueDataLoop
    #  @param 无
    #  @return 无
    def InitTrueDataLoop(self,TargetCopter=-1):
        """ Initialize UDP True data listen loop from CopterSim through 30100 series ports
        """
        
        if TargetCopter<0:
            TargetCopter = self.CopterID
        port = 30100+TargetCopter*2-1
        self.udp_socketTrue = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.udp_socketTrue.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        self.udp_socketTrue.bind(('0.0.0.0', port))
        self.stopFlagTrueData=False
        self.tTrue = threading.Thread(target=self.getTrueDataMsg, args=())
        self.tTrue.start()

    ## @brief 结束真实数据接收的循环
    #  - @anchor EndTrueDataLoop
    #  @param 无
    #  @return 无
    def EndTrueDataLoop(self):
        """ End the true data mode
        """
        self.stopFlagTrueData=True
        self.udp_socketTrue.close()
        time.sleep(0.5)
        self.tTrue.join()


    # Update Pixhawk states from MAVLink for 100Hz
    ## @brief 从30100串口监听真是数据，更新Pixhawk状态
    #  - @anchor getTrueDataMsg
    #  @param 无
    #  @return 无 
    def getTrueDataMsg(self):
        """ Start loop to listen True data from 30100 serial data
        """

        while True:
            if self.stopFlagTrueData:
                break

            buf, addr = self.udp_socketTrue.recvfrom(65500)
            
            # struct outCopterStruct{
            #     int checksum; //1234567890
            #     int CopterID;
            #     double data[32]; //data
            # } ii32d  4+4+8*32 264
            if len(buf)==264:
                UIV=struct.unpack('ii32d',buf)
                checksum=UIV[0]
                CopterID=UIV[1]
                if checksum==1234567890:
                    for i in range(32):
                        self.outCopterVect[i] = UIV[i+2]

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
            if len(buf)==168:
                UIV=struct.unpack('4i24f7d',buf)
                checksum=UIV[0]
                if checksum==123456789:
                    self.out3Ddata.updateUdp(UIV)


    ## @brief 发送与无人机碰撞相关的浮点数据到CopterSim DLL模型的inFloatsCollision接口。
    #  - @anchor fillList
    #  @param size (float): 无人机的尺寸。
    #  @param velE (list of float): 3维列表，无人机在东北天坐标系中的速度，格式为[x, y, z]。
    #  @param ray (list of float): 6维列表，表示无人机各方向的射线数据，格式为[前, 后, 左, 右, 上, 下]。
    #  @param    posE (list of float): 3维列表，无人机在东北天坐标系中的位置，格式为[x, y, z]。
    #  @param    copterID (int): 指定发送数据的无人机ID。若为非正数，则使用初始化时的无人机ID。
    def sendFloatsColl(self, size=0, velE=[0, 0, 0], ray=[0, 0, 0, 0, 0, 0], posE=[0, 0, 0], copterID=-1):
        """
        

        此方法封装了与无人机的碰撞相关数据，并通过UDP发送到指定的模拟器端口。
        struct Ue4RayTraceDrone {
            int checksum;//校验码1234567890
            int CopterID;
            float size;
            float velE[3];
            float ray[6];//前后左右上下
            float posE[3];
        }struct.pack ii13f        
        
        
        

        功能:
        根据提供的参数，封装数据并通过UDP发送。使用struct.pack格式化数据为'ii13f'，即两个整数和十三个浮点数。
        
        示例：
        dll = DllSimCtrlAPI.DllSimCtrlAPI()
        dll.sendFloatsColl(1,[0,0,0],[0,0,0,0,0,0],[0,0,0])
        """
        checkSum = 1234567890  # 校验码
        ID = copterID if copterID > 0 else self.CopterID
        PortNum = 30100 + (ID - 1) * 2  # 计算UDP端口号
        buf = struct.pack("ii13f", checkSum, ID, size, *velE, *ray, *posE)
        self.udp_socket.sendto(buf, (self.ip, PortNum))


    # //输出到CopterSim DLL模型的inCollision20d接口

    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    # Coll20d是1到20维的numpy或list向量
    # 最终会发给DLL的inCollision20d输入口
    ## @brief 向CopterSim DLL模型的inCollision20d接口发送20维碰撞数据。
    #  - @anchor sendColl20d
    #  @param Coll20d (list or numpy array, optional): 1到20维的碰撞double数据，默认为20个0。
    #  @param copterID (int, optional): 指定发送数据的无人机ID。若为非正数，则使用初始化时的无人机ID。
    def sendColl20d(self,Coll20d=[0]*20,copterID=-1):
        """
        

        此方法封装了无人机的20维碰撞数据，并通过UDP发送到指定的模拟器端口。
        struct Ue4Coll20d {
            int checksum; //校验码1234567880
            int CopterID;
            float inCollision20d[20];
        }
        struct.pack ii20f        
        

        功能:
        根据提供的碰撞数据和无人机ID，封装数据并通过UDP发送。数据格式化为'ii20f'，即两个整数和二十个浮点数。
        
        示例
        dll = DllSimCtrlAPI.DllSimCtrlAPI()
        dll.sendColl20d([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],1)
        """
        checkSum=1234567880 # 这里主要是checksum的区别
        Coll20d=self.fillList(Coll20d,20)
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        buf = struct.pack("ii20f",checkSum,copterID,*Coll20d)
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    # TerrIn15d是1到15维的numpy或list向量
    # 最终会发给DLL的TerrainIn15d输入口
    ## @brief 向CopterSim DLL模型的TerrainIn15d接口发送15维地形数据。
    #  - @anchor sendTerrIn15d
    #  @param TerrIn15d (list or numpy array, optional): 1到15维地形浮点数据，默认为15个0。
    #  @param copterID (int, optional): 指定发送数据的无人机ID。若为非正数，则使用初始化时的无人机ID。
    def sendTerrIn15d(self,TerrIn15d=[0]*15,copterID=-1):
        """
        

        此方法封装了无人机的15维地形数据，并通过UDP发送到指定的模拟器端口。        
        

        功能:
        根据提供的地形数据和无人机ID，封装数据并通过UDP发送。数据格式化为'ii15f'，即两个整数和十五个浮点数。
        
        示例
        dll = DllSimCtrlAPI.DllSimCtrlAPI()
        dll.sendTerrIn15d([0,0,0,0,0,0,0,0,0,0,0,0,0],1)
        """
        checkSum=1234567881 # 这里主要是checksum的区别，相对sendColl20d
        TerrIn15d=self.fillList(TerrIn15d,20)
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        buf = struct.pack("ii20f",checkSum,copterID,*TerrIn15d) # 和sendColl20d同样通道
        self.udp_socket.sendto(buf, (self.ip, PortNum))

## @class RflySimCP
#  @brief RflySim 综合模型控制协议。
#  
#  此类通过UDP协议与无人机模拟器进行通信，控制模拟器的操作。
class RflySimCP:
    """
    RflySim 综合模型控制协议
    """
    ## inSILInts数据对应的起始下标
    ILen = 8
    ICmd = 0
    IOffboard = 1
    # 整数型纬度
    ILat = 6
    # 整数型高度
    ILon = 7

    ## inSILInts[0]指令标志
    CmdEn = 1 # 值为1的时候设置一次状态
    CmdSIL = 1 << 1  # 仿真
    CmdArmed = 1 << 2  # 解锁
    CmdTakeoff = 1 << 8  # 起飞
    CmdPosition = 1 << 9  # 定点
    CmdLand = 1 << 10  # 着陆
    CmdReturn = 1 << 11  # 返航
    CmdOffboard_Pos = 1 << 16  # 盘旋（固定翼）
    CmdOffboard_Att = 1 << 17  # 定高模式
    CmdHor = 1 << 14  # 水平位置控制
    CmdOffboardPos = 1 << 16  # Offboard位置控制
    CmdOffboardAtt = 1 << 17  # Offboard位置姿态
    CmdBase = 3

    ## inSILInts[1] Offboard标志
    HasPos = 1  # 位置
    HasVel = 1 << 1  # 速度
    HasAcc = 1 << 2  # 加速度
    HasYaw = 1 << 3  # 偏航角
    HasYawRate = 1 << 4  # 偏航角速率
    HasAtt = 1 << 8  # 姿态
    HasRollRate = 1 << 9  # 滚转角速率
    HasPitchRate = 1 << 10  # 俯仰角速率
    HasThrust = 1 << 11  # 油门
    FRD = 1 << 12  # 速度FRD
    Local = 1 << 13  # 本地位置
    HasFull = 1 << 15
    NED = 1 << 16  # 位置和速度NED
    Global = 1 << 17  # 位置和速度Global

    # Constants for inSILInts[2] Copter Switch flags
    Quadrotor = 1  # 四旋翼
    Hexacopter = 1 << 1  # 六旋翼
    Octocopter = 1 << 2  # 八旋翼
    FourAxOctocopter = 1 << 3  # 四轴八旋翼

    ## inSILFloats位置、速度、加速度、姿态、角速率、油门数据对应的起始下标
    FLen = 20
    FPos = 0
    FVel = 3
    FAcc = 6
    FAtt = 9
    FAttRate = 12
    FThrust = 15
    
    ## 静态方法
    

    @staticmethod
    def IsBitSet(num, bit):
        """
        检查指定位是否被置位。
        参数:
        num (int): 需要检查的整数。
        bit (int): 指定的位。
        返回:
        bool: 如果指定位被置位，则返回True，否则返回False。
        """
        mask = 1 << bit
        return (num & mask) != 0

    @staticmethod
    def IsPosSet(num):
        """
        检查是否启用了位置控制。
        参数:
        num (int): 需要检查的整数。
        返回:
        bool: 如果启用了位置控制，则返回True，否则返回False。
        """
        return RflySimCP.IsBitSet(num, 0)   
    
    @staticmethod
    def getPosNED(x=0,y=0,z=0,yaw=0):
        inSILInts=[0]*8
        inSILFloats=[0]*20
        inSILInts[RflySimCP.ICmd] = RflySimCP.CmdBase + RflySimCP.CmdArmed + RflySimCP.CmdPosition
        inSILInts[RflySimCP.IOffboard] = RflySimCP.HasPos + RflySimCP.HasYaw
        start = RflySimCP.FPos
        inSILFloats[start:start + 3] = [x, y, z]
        start = RflySimCP.FAtt
        inSILFloats[start:start + 3] = [0, 0, yaw]
        inSILDoub = inSILInts+inSILFloats
        return inSILDoub
    
    @staticmethod
    def getPosLocal(x=0,y=0,z=0,yaw=0):
        inSILInts=[0]*8
        inSILFloats=[0]*20
        inSILInts[RflySimCP.ICmd] = RflySimCP.CmdBase + RflySimCP.CmdArmed + RflySimCP.CmdPosition
        inSILInts[RflySimCP.IOffboard] = RflySimCP.HasPos + RflySimCP.HasYaw + RflySimCP.Local
        start = RflySimCP.FPos
        inSILFloats[start:start + 3] = [x, y, z]
        start = RflySimCP.FAtt
        inSILFloats[start:start + 3] = [0, 0, yaw]
        inSILDoub = inSILInts+inSILFloats
        return inSILDoub
    
    @staticmethod
    def getVelNedNoYaw(northvel=0, eastvel=0, downvel=0):
        inSILInts=[0]*8
        inSILFloats=[0]*20
        inSILInts[RflySimCP.ICmd] = RflySimCP.CmdBase + RflySimCP.CmdArmed
        inSILInts[RflySimCP.IOffboard] = RflySimCP.HasVel
        inSILFloats[RflySimCP.FVel] = northvel
        inSILFloats[RflySimCP.FVel + 1] = eastvel
        inSILFloats[RflySimCP.FVel + 2] = downvel
        inSILDoub = inSILInts+inSILFloats
        return inSILDoub
    
    ## @brief 向飞行控制系统发送一个包含速度和偏航速率的指令
    #  - @anchor getVelNED
    #  @param vx x方向的速度
    #  @param vy y方向的速度
    #  @param vz z方向的速度
    #  @param yawRate 偏航速率
    #  @return 无 
    @staticmethod
    def getVelNED(vx=0, vy=0, vz=0, yawRate=0):
        inSILInts=[0]*8
        inSILFloats=[0]*20
        inSILInts[RflySimCP.ICmd] = RflySimCP.CmdBase + RflySimCP.CmdArmed
        inSILInts[RflySimCP.IOffboard] = RflySimCP.HasVel + RflySimCP.HasYawRate
        start = RflySimCP.FVel
        inSILFloats[start:start + 3] = [vx, vy, vz]
        start = RflySimCP.FAttRate
        inSILFloats[start:start + 3] = [0, 0, yawRate]
        inSILDoub = inSILInts+inSILFloats
        return inSILDoub
    
    
    ## @brief 向飞行控制系统发送一个包含速度和偏航速率的指令，速度坐标系为机体前右下FRD坐标系
    #  - @anchor getVelFRD
    #  @param vx x方向的速度
    #  @param vy y方向的速度
    #  @param vz z方向的速度
    #  @param yawRate 偏航速率
    #  @return 无 
    @staticmethod
    def getVelFRD(vx=0, vy=0, vz=0, yawRate=0):
        inSILInts=[0]*8
        inSILFloats=[0]*20
        inSILInts[RflySimCP.ICmd] = RflySimCP.CmdBase + RflySimCP.CmdArmed
        inSILInts[RflySimCP.IOffboard] = RflySimCP.HasVel + RflySimCP.HasYawRate + RflySimCP.FRD
        start = RflySimCP.FVel
        inSILFloats[start:start + 3] = [vx, vy, vz]
        start = RflySimCP.FAttRate
        inSILFloats[start:start + 3] = [0, 0, yawRate]
        inSILDoub = inSILInts+inSILFloats
        return inSILDoub


    # ## @brief 向飞行控制系统发送一个包含加速度的指令
    # #  - @anchor SendSynAcc
    # #  @param ax x方向的加速度
    # #  @param ay y方向的加速度
    # #  @param az z方向的加速度
    # #  @return 无 
    # def SendSynAcc(self, ax, ay, az):
    #     self.inSILInts[RflySimCP.ICmd] = self.cmdBase
    #     self.inSILInts[RflySimCP.IOffboard] = RflySimCP.HasAcc
    #     start = RflySimCP.FAcc
    #     self.inSILFLoats[start:start + 3] = [ax, ay, az]

    # ## @brief 向飞行控制系统发送一个包含位置，速度，加速度，偏航速率的指令
    # #  - @anchor SendSynFull
    # #  @param x x方向的位置
    # #  @param y y方向的位置
    # #  @param z z方向的位置
    # #  @param vx x方向的速度
    # #  @param vy y方向的速度
    # #  @param vz z方向的速度
    # #  @param ax x方向的加速度
    # #  @param ay y方向的加速度
    # #  @param az z方向的加速度
    # #  @param yawRate 偏航速率
    # #  @return 无 
    # def SendSynFull(self, x, y, z, vx, vy, vz, ax, ay, az, yawRate):
    #     self.inSILInts[RflySimCP.ICmd] = self.cmdBase
    #     self.inSILInts[RflySimCP.IOffboard] = RflySimCP.HasPos + RflySimCP.HasVel + RflySimCP.HasYawRate + \
    #                                           RflySimCP.HasAcc + RflySimCP.HasFull
    #     start = RflySimCP.FPos
    #     self.inSILFLoats[start:start + 3] = [x, y, z]
    #     start = RflySimCP.FVel
    #     self.inSILFLoats[start:start + 3] = [vx, vy, vz]
    #     start = RflySimCP.FAttRate
    #     self.inSILFLoats[start:start + 3] = [0, 0, yawRate]
    #     start = RflySimCP.FAcc
    #     self.inSILFLoats[start:start + 3] = [ax, ay, az]

    # ## @brief 向飞行控制系统发送一个包含目标姿态和推力的指令
    # #  - @anchor SendSynAttThrust
    # #  @param roll 飞行器的滚转角
    # #  @param pitch 飞行器的俯仰角
    # #  @param yaw 飞行器的偏航角
    # #  @param thrust 飞行器的推力值
    # #  @return 无 
    # def SendSynAttThrust(self, roll, pitch, yaw, thrust):
    #     self.inSILInts[RflySimCP.ICmd] = self.cmdBase
    #     self.inSILInts[RflySimCP.IOffboard] = RflySimCP.HasAtt + RflySimCP.HasThrust
    #     start = RflySimCP.FAtt
    #     self.inSILFLoats[start:start + 3] = [roll, pitch, yaw]
    #     start = RflySimCP.FThrust
    #     self.inSILFLoats[start] = thrust

    # ## @brief 向飞行控制系统发送一个包含目标姿态的指令
    # #  - @anchor SendSynAtt
    # #  @param roll 飞行器的滚转角
    # #  @param pitch 飞行器的俯仰角
    # #  @param yaw 飞行器的偏航角
    # #  @return 无 
    # def SendSynAtt(self, roll, pitch, yaw):
    #     self.inSILInts[RflySimCP.ICmd] = self.cmdBase
    #     self.inSILInts[RflySimCP.IOffboard] = RflySimCP.HasAtt
    #     start = RflySimCP.FAtt
    #     self.inSILFLoats[start:start + 3] = [roll, pitch, yaw]

    # ## @brief 向飞行控制系统发送一个同步的起飞命令
    # #  - @anchor TakeoffSyn
    # #  @param height 飞行器的起飞高度
    # #  @return 无 
    # def TakeoffSyn(self, height):
    #     """指令不能有延迟"""
    #     self.ResetSynCmd()
    #     cmd = self.cmdBase + RflySimCP.CmdTakeoff
    #     self.inSILInts[RflySimCP.ICmd] = cmd
    #     self.inSILFLoats[RflySimCP.FPos + 2] = height
    #     # 指令从设计上来说只需发一次，但考虑到丢失问题，连续发送1s
    #     # time.sleep(1)
    #     # self.inSILInts[RflySimCP.ICmd] = self.cmdBase

    # ## @brief 向飞行控制系统发送一个同步返回起始位置的命令
    # #  - @anchor ReturnHomeSyn
    # #  @param height 飞行器的起始高度
    # #  @return 无 
    # def ReturnHomeSyn(self, height):
    #     """指令不能有延迟"""
    #     self.ResetSynCmd()
    #     cmd = self.cmdBase + RflySimCP.CmdReturn
    #     self.inSILInts[RflySimCP.ICmd] = cmd
    #     self.inSILFLoats[RflySimCP.FPos + 2] = height
    #     # time.sleep(1)
    #     # self.inSILInts[RflySimCP.ICmd] = self.cmdBase

    # ## @brief 向飞行控制系统发送一个同步降落的命令
    # #  - @anchor LandSyn
    # #  @param height（默认值为0） 飞行器的降落目标高度
    # #  @return 无 
    # def LandSyn(self, height=0):
    #     """指令不能有延迟"""
    #     self.ResetSynCmd()
    #     cmd = self.cmdBase + RflySimCP.CmdLand
    #     self.inSILInts[RflySimCP.ICmd] = cmd
    #     self.inSILFLoats[RflySimCP.FPos + 2] = height
    #     # time.sleep(1)
    #     # self.inSILInts[RflySimCP.ICmd] = self.cmdBase

    # ## @brief 向固定翼飞机的综合模型同时发送目标位置和速度的指令
    # #  - @anchor SendPosSpeedFWSyn
    # #  @param x x坐标
    # #  @param y y坐标
    # #  @param z z坐标
    # #  @param speed 速度
    # #  @return 无 
    # def SendPosSpeedFWSyn(self, x, y, z, speed):
    #     """
    #     固定翼综合模型同时位置和速率
    #     """
    #     self.inSILInts[RflySimCP.ICmd] = self.cmdBase + RflySimCP.CmdPosition
    #     self.inSILInts[RflySimCP.IOffboard] = RflySimCP.HasPos + RflySimCP.HasVel
    #     start = RflySimCP.FPos
    #     self.inSILFLoats[start:start + 3] = [x, y, z]
    #     start = RflySimCP.FVel
    #     self.inSILFLoats[start:start + 3] = [speed, 0, 0]

    
    
    

##  @brief DLL文件外部加载接口类
#  
#   用于加载和初始化一个指定的 DLL 文件（动态链接库）
class ModelLoad:
    ## @brief 这是类的构造函数，用于初始化 ModelLoad 类的实例。参数的详细含义参考 @ref md_ctrl_2md_2BatScripts
    #  - @anchor __init__
    #  @param dll_name 要加载的 DLL 文件的名称。
    #  @param CopterID 飞机ID
    #  @param ClassID 载具三维样式
    #  @param MapName 仿真三维地图名称
    #  @param ip ip地址，默认为本机
    #  @param LocX 载具初始X位置，单位m
    #  @param LocY 载具初始Y位置，单位m
    #  @param Yaw 载具初始yaw角度，设置地图中的初始航向角，单位是度数（degree）。航向角是指在水平面上，相对于北方的方向角。
    #  @param UdpMode 选择不同的通信模式（默认为Full）
    #  @param useGPS 是否启用GPS模式(默认模式选择直角坐标系)
    #  @param GpsOrin GPS原点坐标，三维数列依次为经度、纬度、海拔高度    
    def __init__(self, dll_name,CopterID=1,ClassID=-1,MapName='Grasslands',ip='127.0.0.1',LocX=0,LocY=0,Yaw=0,UdpMode=0,useGPS=False,GpsOrin=[0,0,0]):

        self.udp_socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )  # Create socket
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        self.isSendExtData=False
        
        self.out3Ddata=Data3D()
        self.out3Dvect=[0]*60
        self.outHilData = DataHIL()
        self.outCopterVect=[0]*32
        self.simSpeed = 1
        
        # 获取指定的环境变量 PSP_PATH  
        psp_path = os.environ.get('PSP_PATH')
        if psp_path is None:  # 如果没有成功获取路径，就使用PX4PSP默认路径
            psp_path=r'C:\PX4PSP'
        PSPDllPath=psp_path+ r"\CopterSim\external\model"
        
        dll_full_name=''
        if os.path.isabs(dll_name):
            if os.path.exists(dll_name):
                dll_full_name = dll_name
            else:
                print("没有找到对应的dll模型。")  
                sys.exit(0)
        else:  
            if os.path.exists(sys.path[0]+'/'+dll_name):
                dll_full_name=sys.path[0]+'/'+dll_name
            else:
                if os.path.exists(PSPDllPath+'/'+dll_name):
                    dll_full_name=PSPDllPath+'/'+dll_name
                else:
                    print("没有找到对应的dll模型。")  
                    sys.exit(0)

        if CopterID>1:
            strLen=len(dll_full_name)
            dll_full_name0=dll_full_name
            dll_full_name = dll_full_name[0:strLen-4]+'_'+ str(CopterID)+dll_full_name[strLen-4:strLen]
            shutil.copy2(dll_full_name0,dll_full_name)
            time.sleep(0.5)

        self.dll = ctypes.CDLL(dll_full_name)

        # 添加 isBroadCast 属性
        self.isBroadCast = False  # 默认为 False

        self.isAutoUpdate=False

        # 定义DllInputSILs函数的参数类型
        if hasattr(self.dll, "DllInputSILs"):
            self.dll.DllInputSILs.argtypes = [
                (ctypes.c_int * 8),
                (ctypes.c_float * 20),
            ]

        # 定义DllInputColls函数的参数类型
        if hasattr(self.dll, "DllInputColls"):
            self.dll.DllInputColls.argtypes = [
                (ctypes.c_double * 20),
            ]

        # 定义DllInputTerrain函数的参数类型
        if hasattr(self.dll, "DllInitGpsPos"):
            self.dll.DllInitGpsPos.argtypes = [
                (ctypes.c_double * 3),
            ]

        # 定义DllInitPosAngState函数的参数类型
        if hasattr(self.dll, "DllInitPosAngStat"):
            self.dll.DllInitPosAngState.argtypes = [
                (ctypes.c_double * 3),
                (ctypes.c_double * 3),
            ]

        # 定义DllInitPosAngState函数的参数类型
        if hasattr(self.dll, "DllInitPosAngStat"):
            self.dll.DllInitPosAngState.argtypes = [
                (ctypes.c_double * 3),
                (ctypes.c_double * 3),
            ]

        # 定义DllInputDoubCtrls函数的参数类型
        if hasattr(self.dll, "DllInputDoubCtrls"):
            self.dll.DllInputDoubCtrls.argtypes = [
                (ctypes.c_double * 28),
            ]

        # 定义DllinSIL28d函数的参数类型
        if hasattr(self.dll, "DllinSIL28d"):
            self.dll.DllinSIL28d.argtypes = [
                (ctypes.c_double * 28),
            ]

        # 定义DllTerrainIn15d函数的参数类型
        if hasattr(self.dll, "DllTerrainIn15d"):
            self.dll.DllTerrainIn15d.argtypes = [
                (ctypes.c_double * 15),
            ]

        # 定义DllInCtrlExt函数的参数类型
        if hasattr(self.dll, "DllInCtrlExt"):
            self.dll.DllInCtrlExt.argtypes = [
                (ctypes.c_double * 140),
            ]

        # 定义DllInFromUE函数的参数类型
        if hasattr(self.dll, "DllInFromUE"):
            self.dll.DllInFromUE.argtypes = [
                (ctypes.c_double * 32),
            ]


        # DllInputColls
        if hasattr(self.dll, "DllInputColls"):
            self.dll.DllInputColls.argtypes = [
                (ctypes.c_float * 20),
            ]
            
        # 设置DllGetStep0函数的返回类型为c_double
        if hasattr(self.dll, "DllGetStep0"):
            self.dll.DllGetStep0.restype = ctypes.c_double

        # 设置DllReInitModel和Dllstep的返回类型为c_int
        if hasattr(self.dll, "DllReInitModel"):
            self.dll.DllReInitModel.restype = ctypes.c_int

        if hasattr(self.dll, "Dllstep"):
            self.dll.Dllstep.restype = ctypes.c_int

        # 设置DllOutCopterData的返回类型
        if hasattr(self.dll, "DllOutCopterData"):
            self.dll.DllOutCopterData.argtypes = [
                (ctypes.c_double * 32)
            ]  # 设置参数类型
            self.dll.DllOutCopterData.restype = None  # 设置返回类型

        if hasattr(self.dll, "DlloutVehileInfo60d"):
            self.dll.DlloutVehileInfo60d.argtypes = [
                (ctypes.c_double * 60),
                ctypes.c_int,
            ]  # 设置参数类型
            self.dll.DlloutVehileInfo60d.restype = None  # 设置返回类型

        # 定义 DllDestroyModel 函数指针类型
        if hasattr(self.dll, "DllDestroyModel"):
            self.dll.DllDestroyModel.restype = None
            
        self.CreateVehicle(CopterID,ClassID,MapName,ip,LocX,LocY,Yaw,UdpMode,useGPS,GpsOrin)

    ## @brief 初始化并配置载具模型。参数的详细含义参考 @ref md_ctrl_2md_2BatScripts
    #  - @anchor CreateVehicle
    #  @param dll_name 要加载的 DLL 文件的名称。
    #  @param CopterID 飞机ID
    #  @param ClassID 载具三维样式
    #  @param MapName 仿真三维地图名称
    #  @param ip ip地址，默认为本机
    #  @param LocX 载具初始X位置，单位m
    #  @param LocY 载具初始Y位置，单位m
    #  @param Yaw 载具初始yaw角度，设置地图中的初始航向角，单位是度数（degree）。航向角是指在水平面上，相对于北方的方向角。
    #  @param UdpMode 选择不同的通信模式（默认为Full）
    #  @param useGPS 是否启用GPS模式(默认模式选择直角坐标系)
    #  @param GpsOrin GPS原点坐标，三维数列依次为经度、纬度、海拔高度，启用GPS模式后起作用
    def CreateVehicle(self,CopterID=1,ClassID=-1,MapName='Grasslands',ip='127.0.0.1',LocX=0,LocY=0,Yaw=0,UdpMode=0,useGPS=False,GpsOrin=[0,0,0]):
        self.CopterID=CopterID
        self.ClassID = ClassID
        self.MapName= MapName
        self.LocX = LocX
        self.LocY = LocY
        self.Yaw = Yaw
        self.UdpMode = UdpMode
        self.useGPS = useGPS
        self.GpsOrin = [40.1540302,116.2593683,50.0]
        
        # 创建UEMapServe实例
        self.map = UEMapServe.UEMapServe(MapName)
        self.ue = UE4CtrlAPI.UE4CtrlAPI(ip)
        self.geo = EarthModel.EarthModel()
        self.ip = ip
        
        self.port = 20100 + CopterID*2 -1 # 20100系列端口，转发飞控模拟数据
        self.port2 = 30100 + CopterID*2 -1 # 30100系列端口，转发模型真值
        
        # self.useGPS 判断初始值是否为GPS坐标，如果是，还要做GPS坐标转换
        # 获取对应位置的地形高度
        terrain_height = self.map.getTerrainAltData(self.LocX, self.LocY)
        self.InitPos = [self.LocX,self.LocY,terrain_height]
        self.InitAng = [0,0,self.Yaw/180*math.pi]
        
        self.DllTerrainIn15d(terrain_height)
        
        # 如果输入有效的GPS坐标，则以它为原点
        if abs(GpsOrin[0])>0.1 and abs(GpsOrin[1])>0.1:
            self.GpsOrin = GpsOrin
            self.DllInitGpsPos(GpsOrin)
        self.DllInitPosAngState(self.InitPos,self.InitAng) # 重新设置飞机位置
        self.DllReInitModel() # 重新初始化

        
        self.GpsOrinInit=[0,0,0]     
        self.GpsOrinInit[0] = int(self.GpsOrin[0]*10000000)
        self.GpsOrinInit[1] = int(self.GpsOrin[1]*10000000)
        self.GpsOrinInit[2] = int(self.GpsOrin[2]*1000)
        
        self.SendUpdate3DMap()

        # 更新一步仿真
        self.Dllstep()
        # 发送数据并在RflySim3D创建飞机
        self.Updata3Doutput()

    ## @brief 用于更新载具模型仿真输出，读取车辆当前的运动状态数据并更新到 out3Ddata 和 outHilData，并与地形数据进行交互。
    #  - @anchor Updata3Doutput
    def Updata3Doutput(self):
        
        data3d = self.DlloutVehileInfo60d()
        for i in range(len(data3d)):
            self.out3Dvect[i]=data3d[i]
        self.out3Ddata.update(self.out3Dvect,self.CopterID,self.ClassID)
        
        # 获取当前位置
        east_pos = self.out3Ddata.PosE[0]
        north_pos = self.out3Ddata.PosE[1]
        
        # 获取对应位置的地形高度
        terrain_height = self.map.getTerrainAltData(east_pos, north_pos)
        self.DllTerrainIn15d(terrain_height)
        
        if hasattr(self.dll, "DllOutCopterData"):
            outData=self.DllOutCopterData()
            for i in range(len(outData)):
                self.outCopterVect[i]=outData[i]
        
        
        for i in range(3):
            self.outHilData.gpsHome[i] = self.GpsOrinInit[i]
            self.outHilData.localPos[i] = self.out3Ddata.PosE[i]-self.InitPos[i] # 获取本地位置
            self.outHilData.AngEular[i] = self.out3Ddata.AngEuler[i]
            self.outHilData.localVel[i] = self.out3Ddata.VelE[i]
        
        
        return self.out3Ddata
    
    ## @brief 用于向 RflySim3D发送命令，更新 3D 地图或设置RflySim3D仿真速度。
    #  - @anchor SendUpdate3DMap
    #  @param isSetSpeed 是否启用加速仿真。
    def SendUpdate3DMap(self,isSetSpeed=False):
        
        # 发送地图切换命令
        strCmd = f"RflyChangeMapbyName {self.MapName}"
        self.ue.sendUE4Cmd(strCmd.encode())  
        
        if isSetSpeed:
            # 设置仿真速度
            strCmd = f"RflySetSimSpeed {str(self.simSpeed)}"
            self.ue.sendUE4Cmd(strCmd.encode())  
        
    ## @brief 提取载具模型的 3D 仿真数据out3Dvect并将其打包，以便通过网络转发给RflySim3D
    #  - @anchor SendUpdate3DMap
    def get3DDataBuf(self):
        # 提取三维仿真数据，并转发给RflySim3D
        
        copter_data = self.out3Dvect

        copterID = int(self.CopterID)
        vehicleType = int(copter_data[1])
        if self.ClassID > 0:
            vehicleType=self.ClassID
        
        runnedTime = copter_data[2]
        VelE = copter_data[3:6]
        PosE = copter_data[6:9]
        AngEuler = copter_data[9:12]
        AngQuatern = copter_data[12:16]
        MotorRPMS = copter_data[16:24]
        AccB = copter_data[24:27]
        RateB = copter_data[27:30]
        PosGPS = copter_data[30:33]
        reserved = copter_data[33:60]
        
        checkSum = 123456789
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
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack(
            "4i24f7d",
            checkSum,
            copterID,
            vehicleType,
            0,
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
        return buf   
        
    # 发送消息给RflySim3D，便于
    ## @brief 将打包的 3D 数据通过 UDP 发送到端口 20010，用于与 RflySim3D 进行通信 
    #  - @anchor Send3DData
    def Send3DData(self):
        buf = self.get3DDataBuf()
        self.udp_socket.sendto(buf, (self.ip, 20010)) # 发送到20010系列端口

    ## @brief 将与 Send3DData 同样的 3D 数据发送到端口 30100，用于仿真数据的传输 
    #  - @anchor SendSimData
    def SendSimData(self):
        buf = self.get3DDataBuf()
        self.udp_socket.sendto(buf, (self.ip, self.port2)) # 发送到30100系列端口
        
    ## @brief 发送硬件在环（HIL）仿真数据到指定的端口，通常用于与飞控系统交互 
    #  - @anchor SendHILData
    def SendHILData(self):

        
        CheckSum=1234567890
        #struct outHILStateShort{
        #    int checksum;
        #    int32_t gpsHome[3];     //GPS原始数据，其中经纬度(度*1e7)，高度向上为正(单位m*1e3->mm)
        #    float AngEular[3];    //滤波后的飞机欧拉角，单位rad
        #    float localPos[3];    //滤波后的本地位置，单位m
        #    float localVel[3];    //滤波后的本地位置，单位m/s
        #} 4i9f
        
        buf = struct.pack(
            "4i9f",
            CheckSum,
            *self.outHilData.gpsHome,
            *self.outHilData.AngEular,
            *self.outHilData.localPos,
            *self.outHilData.localVel
        )
        self.udp_socket.sendto(buf, (self.ip, self.port)) # 发送到20100系列端口
        
    ## @brief 发送仿真数据到30100端口，可用于日志记录
    #  - @anchor SendOutCopter
    def SendOutCopter(self):
        # struct outCopterStruct{
        #     int checksum; //1234567890
        #     int CopterID;
        #     double data[32]; //data
        # } ii32d  4+4+8*32 264

        if not hasattr(self.dll, "DllOutCopterData"):
            return False
        
        #self.outCopterVect
        checkSum=1234567890
        CopterID = self.CopterID
        buf = struct.pack(
            "ii32d",
            checkSum,
            CopterID,
            *self.outCopterVect
        )
        #print(self.outCopterVect)
        self.udp_socket.sendto(buf, (self.ip, self.port2)) # 发送到30100系列端口
        return True


    ## @brief 填充或裁剪一个列表
    # - @anchor fillList
    #  @param data 数据
    #  @param inLen 指定数据长度
    #  @param fill（默认值为0） 填充值
    #  @return 裁剪或者填充到长度为inLen的data
    def fillList(self,data,inLen,fill=0):
        if isinstance(data, np.ndarray):
            data = data.tolist()
            
        if isinstance(data, list) and len(data)==inLen:
            return data
        else:
            if isinstance(data, list):
                datLen = len(data)
                if datLen<inLen:
                    data = data + [fill]* (inLen-datLen)
                    
                if datLen>inLen:
                    data = data[0:inLen]
            else:
                data = [data] + [fill]* (inLen-1)
        return data

    # 每秒更新次数，以及仿真速率
    ## @brief 用于启动自动更新循环，以指定的频率更新仿真状态。
    #  - @anchor StartAutUpdate
    #  @param UpdateFreq 更新频率，默认为30hz
    #  @param speed 仿真速度倍数，默认为-1，使用默认仿真速度
    def StartAutUpdate(self,UpdateFreq=30,speed=-1):
        self.isAutoUpdate=True
        self.t1 = threading.Thread(target=self.AutUpdateLoop, args=())
        self.Interval = 1/UpdateFreq
        if speed<0: # 如果设置了速度，就需要发送改变速度的消息
            self.simSpeed=1
            self.SendUpdate3DMap(False)
        else:
            self.simSpeed = speed
            self.SendUpdate3DMap(True)
        self.t1.start()
        
        #RflySetSimSpeed

    # 每秒更新次数，以及仿真速率
    ## @brief 用于停止自动更新循环
    #  - @anchor StopAutUpdate
    def StopAutUpdate(self):
        self.isAutoUpdate = False
        if self.t1 is not None:
            self.t1.join()

    ## @brief 自动更新循环的核心逻辑，实现持续的仿真更新、数据发送等功能。
    #  - @anchor AutUpdateLoop
    def AutUpdateLoop(self):

        # 获取步长
        step_size = self.DllGetStep0()
        print(f"Step size: {step_size}")

        # 获取初始时间
        start_time = time.time()
        self.nextTime = time.time()
        self.lastTime = time.time()
        
        while True:
            if not self.isAutoUpdate:
                break
            self.nextTime = self.nextTime + self.Interval
            sleepTime = self.nextTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.nextTime = time.time()

            dt=time.time()-self.lastTime
            self.lastTime = time.time()
            # 计算更新时间，然后更新平台
            self.ModelUpdate(dt*self.simSpeed) # 实际更新的频率由速度决定
            
            # 发送三维消息
            self.Send3DData()
            
            if self.isSendExtData:
            
                # 发送仿真真值
                self.SendSimData()
                
                # 发送仿真消息
                self.SendHILData()
                
                # 发送OutCopter数值
                self.SendOutCopter()
        
    ## @brief 用于启动外部控制功能，创建两个用于接收外部控制数据的UDP套接字，并开启两个线程来监听数据。
    #  - @anchor StartExtCtrl
    def StartExtCtrl(self):
        self.isSendExtData=True
        self.udp_20100 = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )  # Create socket
        self.udp_20100.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_20100.bind(('0.0.0.0', self.port-1))
        #print('Bind',self.port-1)
        self.isRec20100=True
        self.t2 = threading.Thread(target=self.Recv20100Loop, args=())
        self.t2.start()
        
        self.udp_30100 = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )  # Create socket
        self.udp_30100.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_30100.bind(('0.0.0.0', self.port2-1))
        #print('Bind',self.port2-1)
        self.isRec30100=True
        self.t3 = threading.Thread(target=self.Recv30100Loop, args=())
        self.t3.start()

    ## @brief 停止外部控制，关闭UDP套接字并终止接收线程。
    #  - @anchor StopExtCtrl 
    def StopExtCtrl(self):
        self.isSendExtData=False
        self.udp_20100.close()
        self.udp_30100.close()
        self.isRec20100=False
        self.isRec30100=False
        time.sleep(0.5)
        if self.t2 is not None:
            self.t2.join()
        if self.t3 is not None:
            self.t3.join()
            
    ## @brief 线程的主循环，用于接收20100端口的数据并根据控制模式执行相应的控制命令。
    #  - @anchor Recv20100Loop
    def Recv20100Loop(self):
        while True:
            if not self.isRec20100:
                break

            buf,addr = self.udp_20100.recvfrom(65500)
        
            if len(buf) == 24:
                # struct inOffboardShortData{
                #     int checksum;
                #     int ctrlMode;
                #     float controls[4];
                # } 2i4f
                data = struct.unpack('2i4f',buf)
                checksum = data[0]
                if checksum == 1234567890:
                    ctrlMode=data[1]
                    controls=data[2:6]
                    if ctrlMode==0: # 地球速度控制
                        self.SendVelNED(controls[0],controls[1],controls[2],controls[3])
                    elif ctrlMode==1: # 机体速度控制
                        self.SendVelFRD(controls[0],controls[1],controls[2],controls[3])
                    elif ctrlMode==2: # 地球位置控制
                        self.SendPosNED(controls[0],controls[1],controls[2],controls[3])
                    continue


    ## @brief 线程的主循环，用于接收30100端口的数据并根据不同的数据类型执行相应处理。
    #  - @anchor Recv30100Loop
    def Recv30100Loop(self):
        while True:
            if not self.isRec30100:
                break

            buf,addr = self.udp_30100.recvfrom(65500)
            
            if len(buf) == 120:    
                # struct PX4SILIntFloat{
                #     int checksum;//1234567897
                #     int CopterID;
                #     int inSILInts[8];
                #     float inSILFLoats[16];
                # }; 10i20f
                data = struct.unpack('10i20f',buf)
                checksum=data[0]
                if checksum==1234567897:
                    CopterID=data[1]
                    inSILInts=list(data[2:10])
                    inSILFLoats=list(data[10:30])
                    self.DllInputSILs(inSILInts,inSILFLoats)
                    self.DllInputDoubCtrls(inSILInts+inSILFLoats)
                    self.DllinSIL28d(inSILInts+inSILFLoats)
                    continue
            
            if len(buf) == 232:
                # struct DllInDoubCtrls{
                #     int checksum;//校验码1234567897
                #     int CopterID; // 飞机的ID
                #     double inDoubCtrls[28];//28维的double型输入
                # }; 2i28d
                data = struct.unpack('2i28d',buf)
                checksum=data[0]
                if checksum==1234567897:
                    CopterID=data[1]
                    inSILDoub=list(data[2:30])
                    self.DllInputSILs(inSILDoub[0:8],inSILDoub[8:28])
                    self.DllInputDoubCtrls(inSILDoub)
                    self.DllinSIL28d(inSILDoub)
                    continue
        
            # 响应碰撞结构体
            if len(buf) == 60:
                # 此方法封装了与无人机的碰撞相关数据，并通过UDP发送到指定的模拟器端口。
                # struct Ue4RayTraceDrone {
                #     int checksum;//校验码1234567890
                #     int CopterID;
                #     float size;
                #     float velE[3];
                #     float ray[6];//前后左右上下
                #     float posE[3];
                # }struct.pack ii13f   
                data = struct.unpack('ii13f',buf)
                checksum=data[0]
                if checksum==1234567890:
                    inSILFLoats=[0]*20
                    inSILFLoats[0]=12345
                    inSILFLoats[1]=data[1]
                    inSILFLoats[2]=data[2]
                    inSILFLoats[3]=data[12]
                    inSILFLoats[4]=data[13]
                    inSILFLoats[5]=data[14]
                    inSILFLoats[6]=data[3]
                    inSILFLoats[7]=data[4]
                    inSILFLoats[8]=data[5]
                    for i in range(6):
                        inSILFLoats[9+i]=data[6+i]
                    
                    self.DllInputColls(inSILFLoats)
        
    
    ## @brief 销毁当前加载的仿真模型
    #  - @anchor DllDestroyModel
    def DllDestroyModel(self):
        """
        销毁模型
        """
        if hasattr(self.dll, "DllDestroyModel"):
            self.dll.DllDestroyModel()


    ## @brief 重新初始化仿真模型。
    #  - @anchor DllReInitModel
    def DllReInitModel(self):
        """
        初始化模型
        """
        if hasattr(self.dll, "DllReInitModel"):
            self.dll.DllReInitModel()


    ## @brief 将综合模型系统仿真输入数据传递给 DLL，参见 @ref sendSILIntFloat  
    #  - @anchor DllInputSILs
    #  @param in_ints 最高8维整型数组，主要用于传递系统状态
    #  @param in_floats 最高20维浮点型数组，主要用于传递系统状态
    #  @callgraph 
    #  @callergraph
    def DllInputSILs(self, in_ints=[0]*8, in_floats=[0]*20):
        """
        输入SIL数据

        """
        # in_ints允许是列表或nparray
        in_ints=self.fillList(in_ints,8,0) # 转换为8维列表，默认填充0
        in_ints_1=[0]*8
        for i in range(8):
            in_ints_1[i]=int(in_ints[i])
            
        # print(in_ints_1)
        in_ints=(ctypes.c_int * len(in_ints_1))(*in_ints_1)
        in_floats=self.fillList(in_floats,20,0)# 转换为20维列表，默认填充0
        # print(in_floats)
        in_floats=(ctypes.c_float * len(in_floats))(*in_floats)
        if hasattr(self.dll, "DllInputSILs"):
            self.dll.DllInputSILs(in_ints, in_floats)


    ## @brief 将碰撞数据传递给 DLL，用于仿真中处理碰撞情况，参见 @ref sendColl20d  
    #  - @anchor DllInputColls
    #  @param in_floats_collision 最多20 维的浮点数组，传输碰撞数据
    #  @callgraph 
    #  @callergraph  
    def DllInputColls(self, in_floats_collision=[0]*20):
        """
        输入Collision数据
        """
        in_floats_collision=self.fillList(in_floats_collision,20,0) # 转换为20维列表，默认填充0
        in_floats_collision=(ctypes.c_float * len(in_floats_collision))(*in_floats_collision)
        if hasattr(self.dll, "DllInputColls"):
            self.dll.DllInputColls(in_floats_collision)


    ## @brief 输入三维场景的地形高度数据给DLL，用于仿真环境的地形建模，参见 @ref sendTerrIn15d 
    #  - @anchor DllTerrainIn15d
    #  @param terrain15d 最多15 维double型数据，地形高度数据
    #  @callgraph 
    #  @callergraph
    def DllTerrainIn15d(self, terrain15d=[0]*15):
        """
        输入地形高度数据
        """
        terrain15d=self.fillList(terrain15d,15,0) # 转换为15维列表，默认填充0
        terrain15d=(ctypes.c_double * len(terrain15d))(*terrain15d)
        if hasattr(self.dll, "DllTerrainIn15d"):
            self.dll.DllTerrainIn15d(terrain15d)


    ## @brief 初始化仿真的 GPS 坐标原点，用于地理位置相关的仿真，参见 @ref sendReGPSOrin 	
    #  - @anchor DllInitGpsPos
    #  @param gps 三维浮点数列,依次为经度、纬度、海拔高度
    def DllInitGpsPos(self, gps=[0]*3):
        """
        输入GPS坐标的原点值
        """
        gps=self.fillList(gps,3,0) # 转换为3维列表，默认填充0
        gps=(ctypes.c_double * len(gps))(*gps)
        if hasattr(self.dll, "DllInitGpsPos"):
            self.dll.DllInitGpsPos(gps)


    ## @brief 初始化位置和姿态状态，用于设置仿真对象的初始位置信息和朝向，参见 @ref sendReSimXyzRPYaw 
    #  - @anchor DllInitPosAngState
    #  @param init_pos_offset 3维浮点列表，初始化位置状态
    #  @param init_ang_offset 3维浮点列表，初始化角度状态
    def DllInitPosAngState(self, init_pos_offset=[0,0,0], init_ang_offset=[0,0,0]):
        """
        初始化位置和角度状态
        """
        init_pos_offset=self.fillList(init_pos_offset,3,0) # 转换为3维列表，默认填充0
        init_pos_offset=(ctypes.c_double * len(init_pos_offset))(*init_pos_offset)
        init_ang_offset=self.fillList(init_ang_offset,3,0)# 转换为3维列表，默认填充0
        init_ang_offset=(ctypes.c_double * len(init_ang_offset))(*init_ang_offset)
        
        if hasattr(self.dll, "DllInitPosAngState"):
            self.dll.DllInitPosAngState(init_pos_offset, init_ang_offset)


    ## @brief 综合模型外部控制指令输入接口，用于传递 28 维的双精度控制指令数据，参见 @ref sendInDoubCtrls 
    #  - @anchor DllInputDoubCtrls
    #  @param in_doub_ctrls 最高28 维的double控制指令数据
    def DllInputDoubCtrls(self, in_doub_ctrls=[0]*28):
        """
        输入控制指令数据
        """
        in_doub_ctrls=self.fillList(in_doub_ctrls,28,0) # 转换为28维列表，默认填充0
        in_doub_ctrls=(ctypes.c_double * len(in_doub_ctrls))(*in_doub_ctrls)
        
        if hasattr(self.dll, "DllInputDoubCtrls"):
            self.dll.DllInputDoubCtrls(in_doub_ctrls)


    ## @brief 综合模型外部控制指令输入接口，用于传递 28 维的双精度控制指令数据，参见 @ref sendInDoubCtrls 
    #  @param in_doub_ctrls 最高28 维的double控制指令数据
    def DllinSIL28d(self, in_doub_ctrls=[0]*28):
        """
        输入控制指令数据
        """
        in_doub_ctrls=self.fillList(in_doub_ctrls,28,0) # 转换为28维列表，默认填充0
        in_doub_ctrls=(ctypes.c_double * len(in_doub_ctrls))(*in_doub_ctrls)
        
        if hasattr(self.dll, "DllinSIL28d"):
            self.dll.DllinSIL28d(in_doub_ctrls)

    ## @brief 扩展控制数据，用于支持更高级的控制仿真功能，参见 @ref sendInCtrlExtAll 
    #  - @anchor DllInCtrlExt
    #  @param in_ctrl_ext 最高140 维浮点型的扩展控制数据
    def DllInCtrlExt(self, in_ctrl_ext=[0]*140):
        """
        输入故障注入数据
        """
        in_ctrl_ext=self.fillList(in_ctrl_ext,140,0) # 转换为140维列表，默认填充0
        in_ctrl_ext=(ctypes.c_double * len(in_ctrl_ext))(*in_ctrl_ext)
        
        if hasattr(self.dll, "DllInCtrlExt"):
            self.dll.DllInCtrlExt(in_ctrl_ext)


    ## @brief 从RflySim3D输入自定义数据到 DLL，用于与外部仿真环境的集成。
    #  - @anchor DllInFromUE
    #  @param in_from_ue 最高32 维double型列表，来自RflySim3D的数据
    def DllInFromUE(self, in_from_ue=[0]*32):
        """
        输入UE数据给DLL
        """
        in_from_ue=self.fillList(in_from_ue,32,0) # 转换为32维列表，默认填充0
        in_from_ue=(ctypes.c_double * len(in_from_ue))(*in_from_ue)
        if hasattr(self.dll, "DllInFromUE"):
            self.dll.DllInFromUE(in_from_ue)

    ## @brief 传递 20 维float数组的碰撞数据的接口，用于处理仿真对象之间的碰撞检测。
    #  - @anchor DllInputColls
    #  @param inCll 最高20 维float数组的碰撞数据
    def DllInputColls(self, inCll=[0]*20):
        """
        输入碰撞数据给DLL
        """
        inCll=self.fillList(inCll,20,0) # 转换为32维列表，默认填充0
        inCll_c=(ctypes.c_float * len(inCll))(*inCll)
        if hasattr(self.dll, "DllInputColls"):
            self.dll.DllInputColls(inCll_c)
            #print('Crash Data',inCll)


    ## @brief 获取仿真的步长，用于确定仿真时间步的大小。如果 DLL 中没有对应的函数，默认返回 0.001
    #  - @anchor DllGetStep0
    def DllGetStep0(self):
        """
        获取步长
        """
        if hasattr(self.dll, "DllGetStep0"):
            return self.dll.DllGetStep0()
        else:
            return 0.001

    ## @brief 执行一次仿真步骤。这是对 DLL 中仿真时间推进的基本调用。
    #  - @anchor Dllstep
    def Dllstep(self):
        """
        执行仿真步骤
        """
        if hasattr(self.dll, "Dllstep"):
            self.dll.Dllstep()

    ## @brief 手动更新仿真状态的接口
    #  - @anchor ModelUpdate
    #  @param step_time 执行仿真步骤以推进仿真 step_time 秒
    #  @details 首先获取仿真步长（ @ref DllGetStep0 ），然后根据给定的时间 step_time 和步长计算需要执行的步骤数 num_steps，并调用 @ref Dllstep 完成这些步骤。最后，调用 @ref Updata3Doutput 更新输出信息。
    def ModelUpdate(self, step_time):
        """
        执行仿真步骤，仿真step_time秒
        """
        if hasattr(self.dll, "Dllstep"):
            step_size = self.DllGetStep0()
            if step_size is not None:
                num_steps = int(step_time / step_size)
                for _ in range(num_steps):
                    self.dll.Dllstep()
        # 更新输出信息
        self.Updata3Doutput()
                    
    ## @brief 从 DLL 获取 60 维的载具三维信息数据。包括各种飞行器或车辆的状态数据。
    #  - @anchor DlloutVehileInfo60d
    #  @return 返回的数据是一个 60 维Python 列表。
    def DlloutVehileInfo60d(self):
        if hasattr(self.dll, "DlloutVehileInfo60d"):
            out_data = [0]*60
            out_data=(ctypes.c_double * 60)(*out_data)
            length=60
            self.dll.DlloutVehileInfo60d(out_data, length)
            # 将 ctypes 数组转换为 Python 列表
            data_list = list(out_data)
            # 检查数据是否为空
            if data_list:
                # print("Data received:", data_list)
                return data_list
        return [0]*60


    ## @brief 从 DLL 获取 32 维数据，包括飞行器的传感器读数、状态信息等
    #  - @anchor DllOutCopterData
    #  @return 返回的数据是一个 32 元素的 Python 列表
    def DllOutCopterData(self):
        if hasattr(self.dll, "DllOutCopterData"):
            out_data = [0]*32
            out_data=(ctypes.c_double * len(out_data))(*out_data)
            self.dll.DllOutCopterData(out_data)
            data_list = list(out_data)
            # 检查数据是否为空
            if data_list:
                return data_list
        return [0]*32

    ## @brief 发送NED（北东地）地球参考系下的位置控制命令。
    #  - @anchor SendPosNED
    #  @param x NED坐标系下的x坐标偏移值
    #  @param y NED坐标系下的y坐标偏移值
    #  @param z NED坐标系下的z坐标偏移值
    #  @param yaw 偏航角度
    def SendPosNED(self, x=0,y=0,z=0,yaw=0):
        # 获取位置控制编码，并注入DLL函数
        # SendPosNED是相对起飞点坐标系的位置
        ctrls = RflySimCP.getPosNED(self.InitPos[0]+x,self.InitPos[1]+y,self.InitPos[2]+z,yaw)
        #print(ctrls)
        self.DllInputDoubCtrls(ctrls)
        self.DllinSIL28d(ctrls)
        self.DllInputSILs(ctrls[0:8],ctrls[8:28])

    
    def SendPosLocal(self, x=0,y=0,z=0,yaw=0):
        # 获取位置控制编码，并注入DLL函数
        # SendPosNED是相对起飞点坐标系的位置
        ctrls = RflySimCP.getPosLocal(x,y,z,yaw)
        #print(ctrls)
        self.DllInputDoubCtrls(ctrls)
        self.DllinSIL28d(ctrls)
        self.DllInputSILs(ctrls[0:8],ctrls[8:28])

        
    ## @brief 发送NED（北东地）地球参考系下的速度及偏航控制命令。
    #  - @anchor SendVelNED
    #  @param vx NED坐标系下的x方向速度
    #  @param vy NED坐标系下的y方向速度
    #  @param vz NED坐标系下的z方向速度
    #  @param yawRate 偏航角速度
    def SendVelNED(self, vx=0, vy=0, vz=0, yawRate=0):
        # 获取地球速度控制编码，并注入DLL函数
        ctrls = RflySimCP.getVelNED(vx,vy,vz,yawRate)
        self.DllInputDoubCtrls(ctrls)
        self.DllinSIL28d(ctrls)
        self.DllInputSILs(ctrls[0:8],ctrls[8:28])
        
    ## @brief 发送FRD(前右下)机体参考系下的速度控制命令。
    #  - @anchor SendVelFRD
    #  @param vx FRD坐标系下的x方向速度
    #  @param vy FRD坐标系下的y方向速度
    #  @param vz FRD坐标系下的z方向速度
    #  @param yawRate 偏航角速度
    def SendVelFRD(self, vx=0, vy=0, vz=0, yawRate=0):
        # 获取机体速度控制编码，并注入DLL函数
        ctrls = RflySimCP.getVelFRD(vx,vy,vz,yawRate)
        self.DllInputDoubCtrls(ctrls)
        self.DllinSIL28d(ctrls)
        self.DllInputSILs(ctrls[0:8],ctrls[8:28])
        
    ## @brief 发送NED（北东地）地球参考系下的速度及偏航控制命令。
    #  - @anchor SendVelNED
    #  @param northvel NED坐标系下的x方向速度
    #  @param eastvel NED坐标系下的y方向速度
    #  @param downvel NED坐标系下的z方向速度
    def SendVelNEDNoYaw(self, northvel=0, eastvel=0, downvel=0):
        ctrls = RflySimCP.getVelNedNoYaw(northvel, eastvel, downvel)
        self.DllInputDoubCtrls(ctrls)
        self.DllinSIL28d(ctrls)
        self.DllInputSILs(ctrls[0:8],ctrls[8:28])
        

class MultiCopterDll(ModelLoad):

    def __init__(self,CopterID=1,ClassID=-1,MapName='Grasslands',ip='127.0.0.1',LocX=0,LocY=0,Yaw=0,UdpMode=0,useGPS=False,GpsOrin=[0,0,0]):
        super().__init__('MulticopterNOpx4.dll',CopterID,ClassID,MapName,ip,LocX,LocY,Yaw,UdpMode,useGPS,GpsOrin)

    # TODO 一些公用的接口函数放到ModelLoad，一些多旋翼专用的接口函数放这里面

# TODO 这里要创建固定翼、垂起等其他飞机的模型，根据需求继承ModelLoad，并扩展自己的控制接口
