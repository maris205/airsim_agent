import socket  
import struct  
import binascii
import copy
import time
import threading
from datetime import datetime
from typing import overload
## @file
#
#  @anchor distSimCtrlAPI接口库文件
#  @brief 本文件包含了distSimCtrlAPI类和reqVeCrashData类的定义和使用，用于组网通信、指令发送与接收。


# 定义组播地址和端口  
MCAST_GADDR = '224.0.0.10'  
MCAST_PORT = 9000
ANY = "0.0.0.0"

## @class reqVeCrashData
#  @brief 用于存储从UDP组播接收的节点信息数据结构
#  
#  数据结构格式：
#    int checksum; //校验码，需要设置为12345678
#    int NodeID=1; //局域网内唯一识别ID
#    uint16_t ClassId; //节点类别ID
#    uint16_t SeqID; //节点序号
#    quint8 WinOrLinux; //系统属性，1表示Linux，0表示Windows
#    quint8 CmdExeState; //上条命令执行结果
#    quint8 Recv[6]; //保留位
#    int StartUavIdx; //本机飞机列表起始ID
#    quint64 UavListMask; //基于起始ID的mask标志位
#    quint64 timeStmp; //电脑时间戳
#    char hostname[32]; //主机名字，最多31个字符，最后字符为0
class reqVeCrashData:
    ''''
        int checksum; //校验码，需要设置为12345678
        int NodeID=1;// 局域网内唯一识别ID
        uint16_t ClassId; //节点类别ID
        uint16_t SeqID; //节点序号
        quint8 WinOrLinux; //系统属性 1表示Linux，0表示Windows
        quint8 CmdExeState; //上条命令执行结果
        quint8 Recv[6];//保留位
        int StartUavIdx; //本机飞机列表起始ID
        quint64 UavListMask; //基于起始ID的mask标志位，最多表示起始ID后的64个飞机ID
        quint64 timeStmp; //电脑时间戳
        char hostname[32];//主机名字，最多31个字符，32字符为0
 
        '''''
    ## @brief 构造函数，初始化reqVeCrashData类成员
    #  - @anchor __init__
    #  @param iv 可选参数，用于直接从解包的数据中初始化类成员        
    def __init__(self, iv=None):        
        if iv != None:
            self.CopyData(iv)
            return
        self.Checksum = 12345678
        self.NodeID = 0
        self.ClassID = 0
        self.SeqID = 0
        self.WinOrLinux = 0
        self.CmdExeState = 0
        self.Recv = 0
        self.StartUavIdx = 0
        self.UavListMask = 0
        self.timeStmp = 0
        self.hostname = ""
        self.hasUpdate = True
        
    ## @brief 从解包的数据元组iv中拷贝数据并赋给当前对象的成员变量
    #  - @anchor CopyData
    #  @param iv 解包后的数据元组
    def CopyData(self, iv):
        self.Checksum = iv[0]
        self.NodeID = iv[1]
        self.ClassID = iv[2]
        self.SeqID = iv[3]
        self.WinOrLinux = iv[4]
        self.CmdExeState = iv[5]
        self.Recv = iv[6]
        self.StartUavIdx = iv[7]
        self.UavListMask = iv[8]
        self.timeStmp = iv[9]
        self.hostname = iv[10].decode("UTF-8")
        self.hostname = self.hostname.strip(b"\x00".decode())
        self.hasUpdate = True
        
## @class distSimCtrlAPI
#  @brief 用于通过组播方式扫描局域网中的节点信息并向节点发送指令
class distSimCtrlAPI: 
    ## @brief 构造函数，初始化UDP套接字和相关成员变量
    #  - @anchor __init__
    #  @param ip(str): 本机IP地址，默认为"127.0.0.1"     
    def __init__(self, ip="127.0.0.1"):
        self.ip = ip
        # self.multicast_ip = multicast_ip
        # self.multicast_port = multicast_port
        self.startTime = time.time()  # 添加这行以初始化 startTime 属性

        self.udp_socketDsitSim = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM,socket.IPPROTO_UDP
        )  # Create socket
        
        #self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.inSilVect = []
        self.inReqVect = []
        self.inReqUpdateVect = []

        self.hasMsgEvent = threading.Event()
        self.trueMsgEvent = threading.Event()

    ## @brief 扫描局域网内的组播数据，解析并存储节点信息
    #  - @anchor scan_udp
    #  @return 返回包含所有已解析reqVeCrashData对象的列表
    def scan_udp(self):
        #checksum = 123456789
        # 创建 UDP 套接字  
        #self.udp_socketDsitSim = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  

        # 允许地址重用  
        self.udp_socketDsitSim.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  

        # 绑定套接字到端口  
        self.udp_socketDsitSim.bind(('', MCAST_PORT))
        # 加入组播组  
        mreq = struct.pack("4sl", socket.inet_aton(MCAST_GADDR), socket.INADDR_ANY)  
        
        self.udp_socketDsitSim.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GADDR) + socket.inet_aton(ANY))  

        print(f"Listening for UDP multicast on {MCAST_GADDR}:{MCAST_PORT}")  
        start_time = time.time()

        # 循环接收数据  
        while True:  
            try:
                current_time = time.time()
                elapsed_time =  current_time - start_time
                buf, addr = self.udp_socketDsitSim.recvfrom(1024)
                if elapsed_time >= 5:
                    print("检测局域网完成!")
                    break
                if len(buf) == 72:
                    isCopterExist = False
                    iValue = struct.unpack("2i2H2B6s1i2Q32s", buf[0:72]) #10i32c
                    if iValue[0] == 12345678:
                        #print(f"Received message: {iValue} from {addr}")
                        for i in range(len(self.inReqVect)):  # 遍历数据列表，飞机ID有没有出现过
                            if self.inReqVect[i].NodeID == iValue[1]:  # 如果出现过，就直接更新数据
                               isCopterExist = True
                               self.inReqVect[i].CopyData(iValue)  # =copy.deepcopy(vsr)
                               self.inReqUpdateVect[i] = True
                               #print(f"Received message: {self.inReqVect[i].NodeID} from {addr}")
                               break
                        if not isCopterExist:  # 如果没有出现过，就创建一个结构体
                           vsr = reqVeCrashData(iValue)
                           self.inReqVect = self.inReqVect + [copy.deepcopy(vsr)]  # 扩充列表，增加一个元素
                           self.inReqUpdateVect = self.inReqUpdateVect + [True]
            except KeyboardInterrupt:  
                print("Exiting...")  
                break  
            except Exception as e:  
                print(f"Error: {e}") 
        return self.inReqVect
    
    '''
    group ,多机组网时，使用分组进行指令执行；
    num , DistSim节点，累加于端口号；
    ports ,端口号，发送给所有组网设备使用9000，指定发送给某一台组网设备使用9000+num
    command，发送指令，需要区分接收端系统类型；
    execute_bat_script(命令) 广播命令   
    execute_bat_script(命令，5) 特定发给5号NodeID（对应9005端口）     
     execute_bat_script(命令, -1, 1,2)特定发给1组的2号飞机。 原理：通过9000端口广播给所有飞机，但是只有1组2号飞机响应。
     第一个参数是bat命令，第二个参数是TargetNodeID(默认参数是0，表示发给9000就是广播)，第三个参数是TargetClass(默认是-1)，第四个参数是TargetSeq(默认是-1)。
     TargetNodeID如果是-1，ClassID不是1的话就按 分组和次序来控制。
    '''
    ## @brief 发送指令给指定的节点或广播给所有节点
    #  - @anchor execute_bat_script
    #  
    #  @param command(str): 要执行的命令
    #  @param TargetNodeID(int): 目标节点ID，0表示广播给9000端口
    #  @param TargetClass(int): 目标节点类别（默认为-1，不区分）
    #  @param TargetSeq(int): 目标节点序号（默认为1）
    #
    #  使用示例：
    #    execute_bat_script("echo Hello", 0)  # 广播消息到所有节点(9000端口)
    #    execute_bat_script("command", 5)    # 只发送给NodeID=5节点(9005端口)
    #    execute_bat_script("command", -1, 1, 2) # 按组播概念分组、序号发送    
    def execute_bat_script(self, command , TargetNodeID = 0 , TargetClass = -1 , TargetSeq = 1 ):
        self.udp_socketDsitSim = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  
        ttl = struct.pack('b',1)
        self.udp_socketDsitSim.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_TTL,ttl)
        checksum = 20240719
        nodeIdMask = 0
        mask = 1 << 1
        #print(f"Result:{nodeIdMask:032b}")
        cmdResultData = command.encode('utf-8')

        packed_data = struct.pack('iid256s', checksum , nodeIdMask , datetime.now().timestamp() , cmdResultData)
                
        self.udp_socketDsitSim.sendto(packed_data,(MCAST_GADDR,MCAST_PORT + int(TargetNodeID)))
  
        #print("Message sent to multicast address")
        # 关闭套接字  
        self.udp_socketDsitSim.close()
    
    ## @brief 关闭UDP套接字连接
    #  - @anchor close_udp
    def close_udp(self):
        self.udp_socketDsitSim.close();

if __name__ == "__main__":
    cmdResultData = "start cmd /k start C:\\PX4PSP\\RflySimAPIs\\10.RflySimSwarm\\1.BasicExps\\e1_RflyUdpSwarmExp\\2.RflyUdpFullFour_Mat\\RflyUdpFullFour.bat"
    cmdResultData0 = "echo  Hello world!"
    cmdResultData1 = "start cmd /k start C:\\PX4PSP\\RflySimAPIs\\10.RflySimSwarm\\1.BasicExps\\e1_RflyUdpSwarmExp\\2.RflyUdpFullFour_Mat\\RflyUdpFullFour.exe"
    cmdResultData2 = "taskkill /im CopterSim.exe & taskkill /f /im RflySim3D.exe & taskkill /f /im QGroundControl.exe & taskkill /f /im RflyUdpFullFour.exe"   

    cmdResultData3 = "sudo root"
    cmdResultData4 = 'sudo bash start.sh'
    cmdResultData5 = "start cmd /k start C:\\Users\\uavcs\\Desktop\\FWPosCtrlAPI240912\\AircraftMathworksMavlinkHITLRun.bat"
        
    #获取组网列表
    scan_Networking_list = distSimCtrlAPI().scan_udp();
    for i in range(len(scan_Networking_list)):
        if scan_Networking_list[i].WinOrLinux != 0:
            print(f"序号：{i+1}, 主机ID：{scan_Networking_list[i].NodeID}, 系统类型：Linux , 主机名称：{scan_Networking_list[i].hostname}")
        else:
            print(f"序号：{i+1},主机ID：{scan_Networking_list[i].NodeID}, 系统类型：Win , 主机名称：{scan_Networking_list[i].hostname}")
    #发送给所有测试主机
    #区分组网列中中win和Ubuntu，根据系统执行不同的指令
    #注意发送时9000端口为发送给所有设备

    #发送指令
    startComputer_id_list = input("1.发送所有测试指令")
    distSimCtrlAPI().execute_bat_script(cmdResultData0);
    
    #启动平台
    #SendMassage(cmdResultData,9001)
    #SendMassage(cmdResultData5,9002)
    startComputer_id_list = input("2.启动平台程序")
    distSimCtrlAPI().execute_bat_script(cmdResultData5,6);
    for i in range(5):
        #distSimCtrlAPI().execute_bat_script("start cmd /k start C:\\PX4PSP\\RflySimAPIs\\10.RflySimSwarm\\3.CustExps\\e4_50_drones_mat\\e4.2_100_drones_SIL_mat\\controlled_"+str(i+1)+"\\RflyUdpUltraSimple.bat",i+1);
        distSimCtrlAPI().execute_bat_script(cmdResultData,i+1);
        distSimCtrlAPI().execute_bat_script(cmdResultData1,i+1);
    #distSimCtrlAPI().execute_bat_script(cmdResultData,1);
    
    
    #视觉盒子启动脚本
    #SendMassage(cmdResultData1,9001)
    #SendMassage(cmdResultData3,9003)
    #SendMassage(cmdResultData4,9003)
    startComputer_id_list = input("3.启动智能仿真单元程序及旋翼机控制程序：")
    distSimCtrlAPI().execute_bat_script(cmdResultData3,7);
    distSimCtrlAPI().execute_bat_script(cmdResultData1,7);
    distSimCtrlAPI().execute_bat_script(cmdResultData4,7);

    '''''
    startComputer_id_list = input("请输入上述列表中需要RflySimTool 工具链启动平台的主机ID,用逗号分隔：")
    startComputer_id_list = startComputer_id_list.split("，")

    for i in range(len(startComputer_id_list)):
        if int(startComputer_id_list[i]) == scan_Networking_list[i].NodeID:
            if scan_Networking_list[i].WinOrLinux == 0:
                distSimCtrlAPI().execute_bat_script(-1 , -1 , 9000 + scan_Networking_list[i].NodeID, cmdResultData);
                if scan_Networking_list[i].NodeID == 2:                
                    distSimCtrlAPI().execute_bat_script(-1 , -1 , 9000 + scan_Networking_list[i].NodeID , cmdResultData5);
            else:
                distSimCtrlAPI().execute_bat_script(-1 , -1 , 9000 + scan_Networking_list[i].NodeID , cmdResultData3);
                distSimCtrlAPI().execute_bat_script(-1 , -1 , 9000 + scan_Networking_list[i].NodeID , cmdResultData4);
                
        else:
            print("当前输入主机ID不存在，请检查或重新检索！")
    #distSimCtrlAPI().execute_bat_script(-1 , -1 , 9000 , cmdResultData0);
    '''''