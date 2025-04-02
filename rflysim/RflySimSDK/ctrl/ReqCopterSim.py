import socket
import time
import struct
import copy
import re
import os
import psutil

## @file
#  @brief 这是一个用于初始化指定CopterSim的模块。
#  @anchor ReqCopterSim接口库文件
#  对应例程链接见
#  @ref md_ctrl_2md_2ReqCopterSim 
#
#  用户可以通过这个类向指定CopterSim发送消息，请求初始化CopterSim。
#  @note 用户需要确保CopterSim与Python脚本运行在同一个局域网内。




isWSL = False
if 'WSL_DISTRO_NAME' in os.environ:
    isWSL = True

## @var tarIpAddr
#  Windows电脑的IP地址。
#
#  这个变量用于存储目标IP地址。
#  @note tarIpAddr='192.168.1.1' 这个格式，程序会自动广播到192.168.1.*的网段。
#  @note tarIpAddr='192.168.1.5' 这个格式，程序会发送消息到192.168.1.5的IP地址。
tarIpAddr=''
# 注：tarIpAddr=192.168.1.1这个格式，程序会自动广播到192.168.1.*的网段
# 注：tarIpAddr=192.168.1.5这个格式，程序会发送消息到192.168.1.5的Ip地址




## @brief 这个类表示CopterSim模拟器的时间戳结构。 
#
#  该类设计用于处理和更新CopterSim模拟器中使用的时间戳
class RflyTimeStmp:
    '''
    注意:本条消息会发送给指定远端电脑的端口20005
    struct RflyTimeStmp{
        int checksum; //校验位，取123456789
        int copterID; //当前飞机的ID号
        long long SysStartTime; //开始仿真时的时间戳（单位毫秒，格林尼治标准起点）
        long long SysCurrentTime;//当前时间戳（单位毫秒，格林尼治标准起点）
        long long HeartCount; //心跳包的计数器
    } 2i3q
    '''
    ## @brief RflyTimeStmp类的构造函数。 
    #
    #  用默认值初始化RflyTimeStmp实例。
    def __init__(self):
        ## @var checksum
        #  校验值，应该设置为123456789。
        self.checksum = 1234567897
        ## @var copterID
        #  当前飞机的ID号。
        self.copterID = 0
        ## @var SysStartTime
        #  开始仿真时的时间戳（单位毫秒，格林尼治标准时间）。
        self.SysStartTime = 0  # CopterSim开始仿真时，电脑的时间戳（单位秒）
        ## @var SysCurrentTime
        #  当前时间戳（单位毫秒，格林尼治标准时间）。
        self.SysCurrentTime = 0  # CopterSim运行电脑当前的时间戳（单位秒）
        ## @var HeartCount
        #  心跳包的计数器。
        self.HeartCount = 0

        ## @var isCopterSimOnPC
        #  布尔值，用于确定CopterSim是否与此脚本在同一台电脑上。
        # Python端处理的时间戳。
        # 注意：如果CopterSim和本Python脚本在一台电脑，SysCurrentTime和time.time()的数值应该相差很小（最多延迟10ms）
        # 以此差值来判断，CopterSim和本Python脚本是否在一台电脑上
        self.isCopterSimOnPC = False
        ## @var tarIP
        #  目标IP地址
        self.tarIP ='127.0.0.1'

    ## @brief RflyTimeStmp类的重载构造函数。
    #
    #  使用提供的值初始化RflyTimeStmp实例。
    #  @param iv 初始化值列表 [checksum, copterID, SysStartTime, SysCurrentTime, HeartCount]。
    def __init__(self, iv):
        self.checksum = iv[0]
        self.copterID = iv[1]
        self.SysStartTime = iv[2] / 1000.0
        self.SysCurrentTime = iv[3] / 1000.0
        self.HeartCount = iv[4]
    ## @brief 使用新值更新RflyTimeStmp实例。
    #
    #  用提供的值更新实例变量。
    #  @param iv 新值列表 [checksum, copterID, SysStartTime, SysCurrentTime, HeartCount]。
    def Update(self, iv):
        self.checksum = iv[0]
        self.copterID = iv[1]
        self.SysStartTime = iv[2] / 1000.0
        self.SysCurrentTime = iv[3] / 1000.0
        self.HeartCount = iv[4]


## @brief 类用于初始化指定CopterSim
class ReqCopterSim:
    ## @brief 构造函数，初始化ReqCopterSim类的实例。
    #  开始监听20005端口以获取CopterID的 @ref RflyTimeStmp
    def __init__(self):
        """Start to listen to 20005 port to get RflyTimeStmp of CopterID"""
        
        ## 创建UDP套接字并设置选项。
        self.udp_time = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.udp_time.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_time.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_time.bind(("0.0.0.0", 20005))

        ## 存储接收到的RflyTimeStmp数据的向量。
        self.RflyTimeVect = []
        
        ##  @var VisionCaptureApi.RflyLocalIPVect
        # @brief 存储本电脑的所有本地IP地址，用于判断数据是否来着本机
        self.RflyLocalIPVect = self.get_all_ip()
        
        ## 获取并存储本地主机的IP地址。
        self.hostIp=self.getLocalIp()
        print('HostIP is '+self.hostIp)
        ## 表示是否为错误的多播状态。
        self.isFalseMulti=False
        
        try:
            ## 增加对组播端口的支持
            status = self.udp_time.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                socket.inet_aton("224.0.0.10") + socket.inet_aton("0.0.0.0"),
            )
        except:
            print('Failed to Init multicast!')
            self.isFalseMulti=True
        
        ## 开始更新模拟消息。
        self.updateSimMsg()

   
    ## @brief 获取本地IP地址。 
    #  - @anchor getLocalIp 
    #  @return 返回本地IP地址字符串。
    def getLocalIp(self):

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('10.254.254.254', 1))
            ip = s.getsockname()[0]
        except:
            ip=''
        finally:
            s.close()
            
        if ip == '':
            ip=socket.gethostbyname(socket.gethostname())
            
        if ip == '127.0.1.1' or ip == '':
            ip='127.0.0.1'
            
        return ip

    ## @brief 检查给定的IP地址是否有效。 
    #  - @anchor isValidIp
    #  @param address 要检查的IP地址字符串。
    #  @return 如果IP地址有效，则返回True，否则返回False。
    def isValidIp(self,address):
        pattern = r'^((25[0-5]|2[0-4]\d|1\d{2}|[1-9]\d|\d)\.){3}(25[0-5]|2[0-4]\d|1\d{2}|[1-9]\d|\d)$'
        
        if re.match(pattern, address):
            return True
        else:
            return False
    

    ## @brief 获取本机所有 IP 地址 
    #  - @anchor get_all_ip 
    #  @return 返回本地IP地址列表。
    # 
    def get_all_ip(self):
        ip_list = []
        try:
            dic = psutil.net_if_addrs()
            for adapter in dic:
                snicList = dic[adapter]
                for snic in snicList:
                    if snic.family.name != 'AF_INET':
                        continue         
                    ip = snic.address
                    if ip.startswith('169'):
                        continue
                    ip_list.append(ip)  

            # hostname = socket.gethostname()
            # # 获取IP地址信息
            # addr_infos = socket.getaddrinfo(hostname, None)
            # print(hostname)
            # print(addr_infos)
            # for addr in addr_infos:
            #     ip_list.append(addr[4][0])  
                
            # interfaces = socket.if_nameindex()
            # for interface in interfaces:
            #     print(f"Interface {interface[0]}: {interface[1]}")
                     
        except:
            ip_list.append('127.0.0.1')
        if '127.0.0.1' not in ip_list:
            ip_list.append('127.0.0.1')
        return ip_list

    ## @brief 判断IP地址是否为本地IP 
    #  - @anchor isIpLocal 
    #  @return 返回本地IP地址列表。
    # 
    def isIpLocal(self,IP):
        if IP in self.RflyLocalIPVect: # 为本机
            return True
        else:
            return False

    ## @brief 更新模拟消息。 
    #  - @anchor updateSimMsg 
    #  设置UDP套接字的超时时间，并监听CopterSim的心跳消息。
    #  将接收到的有效消息存储在 @ref RflyTimeVect 向量中。
    def updateSimMsg(self):
        if isWSL:
            return

        self.udp_time.settimeout(3)

        print('Start listening CopterSim heartbeat Msg ...')
        # 开始监听三秒钟，得到局域网内所有CopterSim的状态信息
        startTime=time.time()
        while True:
            try:
                buf, AddrPort = self.udp_time.recvfrom(65500)
                addr = AddrPort[0]
                # print(AddrPort)
                # if addr==self.hostIp:
                #     addr='127.0.0.1'

                if len(buf) == 32:
                    # print(len(buf[0:12]))
                    TimeData = struct.unpack("2i3q", buf)
                    if TimeData[0] == 123456789:
                        cpIDTmp = TimeData[1]
                        isTimeExist = False
                        
                        # 确认是否已经接收过消息
                        for tStmp in self.RflyTimeVect:
                            if tStmp.copterID == cpIDTmp:
                                isTimeExist = True
                                
                                if self.isIpLocal(addr): # 如果是本机消息，则只接收本机消息
                                    tStmp.Update(TimeData)
                                    if not tStmp.isCopterSimOnPC: # 如果之前没收到过本地包，则重新接收
                                        self.RflyTimeVect.remove(tStmp) 
                                        isTimeExist=False
                                else:
                                    if not tStmp.isCopterSimOnPC: # 如果未收到本机消息，就使用局域网消息
                                        tStmp.Update(TimeData)
                                break
                            
                        if not isTimeExist:
                            tStmp = RflyTimeStmp(TimeData)
                            #print("Got time msg from CopterSim #", tStmp.copterID)
                            if (self.isIpLocal(addr)):
                                tStmp.isCopterSimOnPC = True  # 说明Python和CopterSim在一台电脑上
                                tStmp.tarIP = '127.0.0.1'
                                print("Got time msg from CopterSim #",tStmp.copterID,", running on this PC")
                            else:  # 说明本Python脚本和CopterSim不在一台电脑上
                                tStmp.isCopterSimOnPC = False
                                tStmp.tarIP = addr
                                print("Got time msg from CopterSim #",tStmp.copterID,", not on this PC")
                                #print(tStmp.SysCurrentTime)
                                #print(CurPyTime)

                            self.RflyTimeVect = self.RflyTimeVect + [
                                copy.deepcopy(tStmp)
                            ]  # 扩充列表，增加一个元素
                        
            except:
                print("No Time Msg!")
                break # 跳出循环，不再继续监听

            if time.time()-startTime>3.1:
                break # 跳出循环，不再继续监听
        print('End listening CopterSim heartbeat.')

        ## CopterSim的IP列表
        self.Iplist=[]
        for i in range(len(self.RflyTimeVect)):
            self.Iplist.append((self.RflyTimeVect[i].copterID,self.RflyTimeVect[i].tarIP))
        print('Got '+str(len(self.RflyTimeVect))+' CopterSim on the LAN.')

    ## @brief 获取CopterSim IP列表。
    #  - @anchor getSimIpList
    #  返回包含所有检测到的CopterSim的ID和IP地址的列表。
    #  @return CopterSim的ID和IP地址的列表。
    def getSimIpList(self):
        return self.Iplist
    
    ## @brief 根据CopterID获取其IP地址。
    #  - @anchor getSimIpID
    #  @param CopterID 要获取IP地址的CopterSim的ID。
    #  @return 对应CopterSim的IP地址，如果未找到则为空字符串。
    def getSimIpID(self,CopterID=1):
        global tarIpAddr

        if isWSL:
            return '127.0.0.1'

        IP=''
        for i in range(len(self.RflyTimeVect)):
            if CopterID==self.RflyTimeVect[i].copterID:
                IP = self.RflyTimeVect[i].tarIP
                break
        
        if IP == '':
            # 如果没有收到组播的包，就请求一下广播IP
            self.isFalseMulti=True
            # struct Ue4Req {
            #     int checksum;
            #     int reqIndex ;
            # }
            
            buf = struct.pack("2i", 123450, 90) # 两个暗号，通知CopterSim将心跳包返回本电脑
            
            # 在WIFI模式下，广播消息可能禁用
            self.udp_time.sendto(buf, ('255.255.255.255', 30100 + CopterID*2 -2))
            
            cList = self.hostIp.split(".")
            CurIp=[0,0,0,0]
            if len(cList) == 4 and cList[0]!=127:
                CurIp[0] = int(cList[0])
                CurIp[1] = int(cList[1])
                CurIp[2] = int(cList[2])
                CurIp[3] = int(cList[3])            
            
            # 从*.*.*.2到*.*.*.254
            for i in range(254):
                SendIp=str(CurIp[0])+'.'+str(CurIp[1])+'.'+str(CurIp[2])+'.'+str(i+2)
                try:
                    self.udp_time.sendto(buf, (SendIp, 30100 + CopterID*2 -2))
                except:
                    pass


            if tarIpAddr!='' and self.isValidIp(tarIpAddr):    

                cList1 = tarIpAddr.split(".")
                CurIp1=[0,0,0,0]
                if len(cList1) == 4:
                    CurIp1[0] = int(cList1[0])
                    CurIp1[1] = int(cList1[1])
                    CurIp1[2] = int(cList1[2])
                    CurIp1[3] = int(cList1[3])                  

                # 如果和本机的网段不一样，则需要单独指定IP发放
                if CurIp1[0] != CurIp[0] or CurIp1[1] != CurIp[1] or CurIp1[2] != CurIp[2]:
                    buf = struct.pack("2i", 123450, 91) # 发送暗号，将心跳包回传本电脑
                    if CurIp1[3] == 1:
                        # 从*.*.*.2到*.*.*.254，这里是为了防止广播地址被屏蔽
                        for i in range(254):
                            SendIp=str(CurIp[0])+'.'+str(CurIp[1])+'.'+str(CurIp[2])+'.'+str(i+2)
                            try:
                                self.udp_time.sendto(buf, (SendIp, 30100 + CopterID*2 -2))
                            except:
                                pass    
                    else:
                        self.udp_time.sendto(buf, (tarIpAddr, 30100 + CopterID*2 -2))                
                else:
                    self.udp_time.sendto(buf, (tarIpAddr, 30100 + CopterID*2 -2))

            self.updateSimMsg()
            time.sleep(0.5)
            for i in range(len(self.RflyTimeVect)):
                if CopterID==self.RflyTimeVect[i].copterID:
                    IP = self.RflyTimeVect[i].tarIP
                    break

        if IP=='' and tarIpAddr!='' and self.isValidIp(tarIpAddr):
            IP=tarIpAddr
            
        if IP=='':
            IP='127.0.0.1'
            
        return IP
    
    ## @brief 获取主机IP地址。
    #  - @anchor getHostIP
    #  @return 返回主机IP地址。
    def getHostIP(self):
        return self.hostIp

    ##
    # - @anchor sendReCopterSim
    # @brief 请求初始化指定的CopterSim。
    #
    # @param CopterID 要发送命令的Copter的ID。默认值为1。
    # @param isReqIP 表示是否进行IP请求。小于等于0表示不发响应，大于0表示将进行请求。默认值为1。
    # @param UDP_mode 设置UDP模式。小于0表示不发响应，大于等于0将UDP模式设置为指定值。默认值为-1。
    # @param isXyYaw 表示是否使用XY和Yaw值。小于等于0表示不发响应，大于0表示将使用这些值。默认值为0。
    # @param xyYaw 一个列表，包含XY坐标和Yaw角度。默认值为[0, 0, 0]。
    # @param isZRP 表示是否使用Z、Roll和Pitch值。小于等于0表示不发响应，大于0表示将使用这些值。默认值为0。
    # @param zRollPitch 一个列表，包含Z、Roll和Pitch值。默认值为[0, 0, 0]。
    # @param otherParams 一个列表，用于保留未来使用的其他参数。默认值为[0, 0, 0, 0]。
    #
    # 此方法根据指定的参数构建一条消息，并将其发送到指定的Copter。
    #
    # 如果`isReqIP`大于0且Copter的IP地址为`127.0.0.1`，则将`isReqIP`的值设置为0。
    # 消息然后发送到组播地址`224.0.0.10`上的端口`20002`。如果`isFalseMulti`为真且Copter的IP地址不为空，
    # 则消息还将直接发送到Copter的IP地址上的端口`20002`。
    #
    def sendReCopterSim(self,CopterID=1,isReqIP=1,UDP_mode=-1,isXyYaw=0,xyYaw=[0,0,0],isZRP=0,zRollPitch=[0,0,0],otherParams=[0,0,0,0]):
        '''
        struct reCopterSimMsg{
            int checksum; //数据校验位1234567，用于校验数据是否正确
            int CopterID;// 请求的CopterID，用于校验请求是否正确。
            int8_t isReqIP; //如果<=0则不响应，如果>0则勾选联机勾选，且将飞控数据发给本电脑IP（发送本请求消息的电脑）
            int8_t isXyYaw; //如果<=0则不响应，如果>0，则使用后面xyYaw的值，重新部署飞机位置
            int8_t isZRP; // 如果<=0则不响应（默认z会贴合地形，rollPitch会变为0），如果>0，则使用zRollPitch的值部署飞机位姿。本接口可以让飞机初始化在空中
            int8_t UDP_mode; //如果<0，则不响应。如果>=0，则修改UDP模式为指定值。
            uint8_t otherParams[4]; //保留位其他参数，留给将来使用。
            float zRollPitch[3]; //zRollPitch的初始值，单位米和度，z向下为正
            float yaw;// 初始偏航角，单位度
            double xy[2]; // 初始xy的值，支持双精度大地图，单位米，北东地
        }; //2i4b4B4f2d
        if isXyYaw==2: configure GPS Origin through lat: xy[0] lon: xy[1] alt: yaw
        if isXyYaw==3: configure GPS pos through lat: xy[0] lon: xy[1] yaw
        '''
        if isReqIP==0:
            isReqIP=-1

        myIp = self.getSimIpID(CopterID)
        if isReqIP>0:
            if myIp == '127.0.0.1': # 说明是向本机IP请求数据，那么标注为0模式
                isReqIP=0
        
        checksum=1234567
        yaw=xyYaw[2]
        xy=[xyYaw[0],xyYaw[1]]
        intFlag = [isReqIP,isXyYaw,isZRP,UDP_mode]
        buf = struct.pack("2i4b4B4f2d", checksum,CopterID,*intFlag,*otherParams,*zRollPitch,yaw,*xy)
        
        if not self.isFalseMulti:
            self.udp_time.sendto(buf, ('224.0.0.10',20002)) # 广播给一个所有CopterSim都会监听的端口

        if self.isFalseMulti and myIp!='':
            self.udp_time.sendto(buf, (myIp,20002)) # 广播给一个所有CopterSim都会监听的端口
            self.udp_time.sendto(buf, (myIp,30100+CopterID*2-2)) # # 指定发送给期望的CopterSim


    ##
    # @brief 请求更改CopterSim的DLL模型和地图。
    #  - @anchor sendReDllMap
    #
    # @param CopterID 要发送命令的Copter的ID。如果为0，则不给出任何响应。如果为-1，则命令会广播到所有的Copter。如果大于0，则命令会发送到指定的Copter。
    # @param dllOrMap 切换选项：小于等于0表示不发响应，1表示更改DLL模型，2表示更改地图。
    # @param index 选项ID号；如果小于0，则使用名称确定选项。如果大于等于0，则使用索引替代名称。
    # @param name DLL模型或地图的名称。默认为空字符串。
    #
    # 此方法根据指定的参数构建一条消息，并将其发送到CopterSim。
    # 消息发送到组播地址`224.0.0.10`上的端口`20002`。如果`isFalseMulti`为真且Copter的IP地址不为空，
    # 则消息还将直接发送到Copter的IP地址上的端口`20002`。
    #
    def sendReDllMap(self,CopterID=0,dllOrMap=-1,index=-1,name=''):
        '''
        请求改变CopterSim的DLL模型和地图
        struct CptSimCMD{
            int checksum;//校验码，用于确认数据正确性，这里取值 1234567895
            int CopterID; //目标飞机ID，如果为0则不响应；如果为-1，则广播给所有飞机；如果>0则给指定飞机。
            int flag; //切换选项：如果<=0则不响应；如果=1，则修改DLL模型；如果=2，则修改地图；
            int index; // 选项ID序号；如果<0则不响应，使用name来确定选项；如果>=0，则用序号而不是name来确定选项。
            char name[48]; //DLL模型或地图的名字
        }//长度64，4i48s
        '''
        checksum=1234567895
        # 如果是str类型，则转换为bytes类型
        if isinstance(name, str):
            name = name.encode()
        buf = struct.pack("4i48s", checksum,CopterID,dllOrMap,index,name)
        
        if not self.isFalseMulti:
            self.udp_time.sendto(buf, ('224.0.0.10',20002)) # 广播给一个所有CopterSim都会监听的端口

        myIp = self.getSimIpID(CopterID)
        if self.isFalseMulti and myIp!='':
            self.udp_time.sendto(buf, (myIp,20002)) # 广播给一个所有CopterSim都会监听的端口
            self.udp_time.sendto(buf, (myIp,30100+CopterID*2-2)) # 指定发送给期望的CopterSim
            

    ##
    # @brief 请求通过名称更改CopterSim的DLL模型。
    #  - @anchor sendReSimDllName
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    # @param name DLL模型的名称。默认为空字符串。
    #
    def sendReSimDllName(self,CopterID=1,name=''):
        self.sendReDllMap(CopterID,1,-1,name)

    ##
    # @brief 请求通过索引更改CopterSim的DLL模型。
    #  - @anchor sendReSimDllIdx
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    # @param index DLL模型的索引。默认为-1。
    #
    def sendReSimDllIdx(self,CopterID=1,index=-1):
        self.sendReDllMap(CopterID,1,index)

    ##
    # @brief 请求通过名称更改CopterSim的地图。
    #  - @anchor sendReSimMapName
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    # @param name 地图的名称。默认为空字符串。
    #
    def sendReSimMapName(self,CopterID=1,name=''):
        self.sendReDllMap(CopterID,2,-1,name)

    ##
    # @brief 请求通过索引更改CopterSim的地图。
    #  - @anchor sendReSimMapIdx
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    # @param index 地图的索引。默认为-1。
    #
    def sendReSimMapIdx(self,CopterID=1,index=-1):
        self.sendReDllMap(CopterID,2,index)

    ##
    # @brief 请求指定的CopterSim向主机计算机发送数据。
    #  - @anchor sendReSimIP
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    #
    def sendReSimIP(self,CopterID=1):
        # 请求指定CopterSim将数据回传到本电脑
        self.sendReCopterSim(CopterID)

    ##
    # @brief 请求指定的CopterSim将UDP模式更改为指定的值。
    #  - @anchor sendReSimUdpMode
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    # @param UDP_mode 所需的UDP模式。默认为-1。
    #
    def sendReSimUdpMode(self,CopterID=1,UDP_mode=-1):
        # 请求指定CopterSim更换UDP_Mode为指定值
        self.sendReCopterSim(CopterID,-1,UDP_mode)

    ##
    # @brief 请求指定的CopterSim将xyYaw更改为指定的值。
    #  - @anchor sendReSimXYyaw
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    # @param xyYaw 所需的xyYaw值。默认为[0, 0, 0]。
    #
    def sendReSimXYyaw(self,CopterID=1,xyYaw=[0,0,0]):
        # 请求指定CopterSim更换xyYaw为指定值，xy单位米，北东地，yaw单位度
        self.sendReCopterSim(CopterID,-1,-1,1,xyYaw)

    ##
    # @brief 请求指定的CopterSim将GPS原点更改为指定的值。
    #  - @anchor sendReGPSOrin
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    # @param LLA 所需的GPS原点值（纬度、经度、高度）。默认为[0, 0, 0]。
    #
    def sendReGPSOrin(self,CopterID=1,LLA=[0,0,0]):
        # 请求指定CopterSim更换GPS原点为指定值，LLA -> lat (degree), lon (degree), alt (m, 向上为正)
        self.sendReCopterSim(CopterID,-1,-1,2,LLA)

    ##
    # @brief 请求指定的CopterSim将位置更改为指定的GPS值。
    #  - @anchor sendReGPSPos
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    # @param lat 所需的纬度值。默认为0。
    # @param lon 所需的经度值。默认为0。
    # @param yaw 所需的偏航值。默认为0。
    #
    def sendReGPSPos(self,CopterID=1,lat=0,lon=0,yaw=0):
        # 请求指定CopterSim更换坐标为指定GPS值，LLA -> lat (degree), lon (degree), alt (m, 向上为正)
        self.sendReCopterSim(CopterID,-1,-1,3,[lat,lon,yaw])

    ##
    # @brief 请求指定的CopterSim切换到GPS显示模式。
    #  - @anchor sendEnGpsMode
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    #
    def sendEnGpsMode(self,CopterID=1):
        # 请求指定CopterSim更换GPS显示模式
        self.sendReCopterSim(CopterID,-1,-1,4)

    ##
    # @brief 请求指定的CopterSim切换到xy显示模式。
    #  - @anchor sendEnXyMode
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    #
    def sendEnXyMode(self,CopterID=1):
        # 请求指定CopterSim更换xy显示模式
        self.sendReCopterSim(CopterID,-1,-1,5)

    ##
    # @brief 请求指定的CopterSim将Xyz和RollPitchYaw更改为指定的值。
    #  - @anchor sendReSimXyzRPYaw
    #
    # @param CopterID 要发送命令的Copter的ID。默认为1。
    # @param Xyz 所需的Xyz值。默认为[0, 0, 0]。
    # @param RPYaw 所需的滚转、俯仰和偏航值。默认为[0, 0, 0]。
    #
    def sendReSimXyzRPYaw(self,CopterID=1,Xyz=[0,0,0],RPYaw=[0,0,0]):
        # 请求指定CopterSim更换Xyz为指定值，单位米，北东地
        # 请求指定CopterSim更换Roll pitch yaw为指定值，单位度
        xyYaw=[Xyz[0],Xyz[1],RPYaw[2]]
        zRollPitch=[Xyz[2],RPYaw[0],RPYaw[1]]
        self.sendReCopterSim(CopterID,-1,-1,1,xyYaw,1,zRollPitch)

