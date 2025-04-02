#!/usr/bin/env python3

"""
通过 MAVLink 打开一个 shell。

@author: Beat Kueng (beat-kueng@gmx.net)
"""

## @file
#
#  @anchor mavRflyShell接口库文件

# 从 __future__ 导入 print_function，以确保在 Python 2 中也能使用 Python 3 的 print 函数
from __future__ import print_function
from io import StringIO
import sys  # 处理 Python 运行时环境和标准输入输出
from timeit import default_timer as timer  # 用于获取当前时间戳
from argparse import ArgumentParser  # 用于解析命令行参数
import os  # 与操作系统交互
import re
import time
import subprocess
import json
import socket

nuttxCmd = ['netman show',
          'echo IPADDR=192.168.151.101 >> /fs/microsd/net.cfg',
          'echo NETMASK=255.255.255.0 >>/fs/microsd/net.cfg',
          'echo ROUTER=192.168.151.1 >>/fs/microsd/net.cfg',
          'echo DNS=192.168.151.1 >>/fs/microsd/net.cfg',
          'netman update -i eth0'
          ]
#original_stdout = sys.stdout
#output = StringIO()
#sys.stdout = output

# 尝试导入 pymavlink 库，并处理导入失败的情况
try:
    from pymavlink import mavutil
except ImportError as e:
    print("无法导入 pymavlink: " + str(e))
    print("")
    print("你可能需要安装它，使用以下命令:")
    print("    pip3 install --user pymavlink")
    print("")
    sys.exit(1)  # 如果 pymavlink 未安装，则退出程序

# 尝试导入 pyserial 库，并处理导入失败的情况
try:
    import serial
except ImportError as e:
    print("无法导入 pyserial: " + str(e))
    print("")
    print("你可能需要安装它，使用以下命令:")
    print("    pip3 install --user pyserial")
    print("")
    sys.exit(1)  # 如果 pyserial 未安装，则退出程序

##  @class MavlinkSerialPort
#   @brief 定义一个 MavlinkSerialPort 类，用于通过 MAVLink 进行串口通信
class MavlinkSerialPort():
    ## @brief 初始化串口连接，用于与飞控进行 MAVLink 通信。
    #  - @anchor __init__
    #  @param  portname: 串口名称（如 /dev/ttyUSB0）。
    #  @param  baudrate: 串口波特率（常见值如 115200）。 
    #  @param  devnum: 设备编号，默认为 0。
    #  @param  debug: 调试级别，默认为 0。
    #  @return 无返回值。
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.debug("使用 MAVLink 连接到 %s ..." % portname)  # 输出调试信息
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)  # 连接 MAVLink
        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)  # 发送心跳包
        self.mav.wait_heartbeat()  # 等待心跳包以确认连接
        self.debug("HEARTBEAT OK\n")  # 输出连接成功信息
        self.debug("锁定串口设备\n")  # 输出设备锁定信息

    ## @brief 输出调试信息。
    #  - @anchor debug
    #  @param  s: 要输出的调试信息字符串。
    #  @param  level: 调试级别，默认为 1，决定输出的详细程度。 
    #  @return 无返回值。
    def debug(self, s, level=1):
        '''写入调试文本'''
        if self._debug >= level:
            print(s)

    ## @brief 向飞控发送字节数据。
    #  - @anchor write
    #  @param  b: 需要发送的字节数据。
    #  @return 无返回值。
    def write(self, b):
        '''写入字节数据'''
        self.debug("发送 '%s' (0x%02x) 长度 %u\n" % (b, ord(b[0]), len(b)), 2)  # 输出调试信息
        while len(b) > 0:
            n = len(b)
            if n > 70:
                n = 700
            buf = [ord(x) for x in b[:n]]  # 将字符串转换为字节列表
            buf.extend([0]*(70-len(buf)))  # 填充到 70 字节
            self.mav.mav.serial_control_send(self.port,
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                             0,
                                             0,
                                             n,
                                             buf)  # 发送数据
            b = b[n:]  # 更新待发送数据

    ## @brief 关闭串口连接。
    #  - @anchor close
    #  @param  无
    #  @return 无返回值。
    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)  # 关闭串口连接

    ## @brief 从 MAVLink 中读取字节数据，处理接收到的数据。
    #  - @anchor _recv
    #  @param  无
    #  @return 接收到的字节数据。
    def _recv(self):
        '''从 MAVLink 中读取字节数据'''
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True,
                                timeout=0.03)  # 等待接收数据
        if m is not None:
            if self._debug > 2:
                print(m)
            data = m.data[:m.count]  # 提取有效数据
            self.buf += ''.join(str(chr(x)) for x in data)  # 更新缓冲区

    ## @brief 读取指定长度的字节数据。
    #  - @anchor read
    #  @param  n: 要读取的字节数。
    #  @return 读取到的字节数据。
    def read(self, n):
        '''读取字节数据'''
        if len(self.buf) == 0:
            self._recv()  # 如果缓冲区为空，则从 MAVLink 读取数据
        if len(self.buf) > 0:
            if n > len(self.buf):
                n = len(self.buf)
            ret = self.buf[:n]  # 获取要返回的数据
            self.buf = self.buf[n:]  # 更新缓冲区
            if self._debug >= 2:
                for b in ret:
                    self.debug("读取 0x%x" % ord(b), 2)
            return ret
        return ''

## @brief 连接飞控控制台
#  - @anchor ConnectNsh
#  @return 成功：返回初始化的 MavlinkSerialPort 对象，用于后续与飞控的通信。失败：输出错误信息，函数直接返回。
def ConnectNsh():
    parser = ArgumentParser(description=__doc__)  # 创建命令行解析器
    parser.add_argument('port', metavar='PORT', nargs='?', default=None,
            help='Mavlink 端口名称: serial: DEVICE[,BAUD], udp: IP:PORT, tcp: tcp:IP:PORT。 例如: \
/dev/ttyUSB0 或 0.0.0.0:14550。 如果未给定，则自动检测串口。')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int,
                    help="Mavlink 端口波特率 (默认 57600)", default=57600)
    args = parser.parse_args()  # 解析命令行参数
    print(args)
    if args.port == None:
        if sys.platform == "darwin":
            args.port = "/dev/tty.usbmodem01"  # macOS 默认端口
        else:
            serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',
                "*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*', "*Gumstix*"])  # 自动检测串口

            # 调用外部程序并捕获输出
            try:
                # 获取指定的环境变量 PSP_PATH
                psp_path = os.environ.get('PSP_PATH')
                if psp_path is not None:
                    print("PSP_PATH:", psp_path)
                else:
                    print("环境变量 PSP_PATH 不存在。")
                result = subprocess.run(
                    [psp_path + r"\CopterSim\GetComList.exe"],  # 注意前面加上r表示原始字符串
                    capture_output=True,
                    text=True,
                    check=True  # 如果返回码非零，会引发异常
                )

                # 输出结果
                print("标准输出：")
                print(result.stdout)

                print("标准错误：")
                print(result.stderr)  # 错误信息（如果有）

            except subprocess.CalledProcessError as e:
                print("命令执行失败，返回码：", e.returncode)
                print("错误信息：", e.stderr)

            if len(result.stdout) == 0:
                print("错误: 未找到串口连接")
                return
            args.port = 'COM'+result.stdout;

    print("连接到 MAVLINK...")
    mav_serialport = MavlinkSerialPort(args.port, args.baudrate, devnum=10)  # 创建 MavlinkSerialPort 对象
    mav_serialport.write('\n')  # 发送换行符，确保 shell 启动
    # mav_serialport.write('\n')  # 发送换行符，确保 shell 启动
    return mav_serialport


##  @class RflyShell
#   @brief 定义一个 RflyShell 类，用于向飞控发送命令
class RflyShell:
    ## @brief 初始化 MAVLink 串口，并创建 MAVLink 连接。
    #  - @anchor __init__
    #  @param  port: 串口名称。
    #  @param  baudrate: 波特率，默认为 115200。 
    #  @return 无返回值。
    def __init__(self,port,baudrate=115200):
        self.mav_serialport = MavlinkSerialPort(port, baudrate, devnum=10)  # 创建 MavlinkSerialPort 对象
        self.mav_serialport.write('\n')  # 发送换行符，确保 shell 启动
        
    ## @brief 检查数据中是否包含提示符（nsh>），用于判断命令是否完成。
    #  - @anchor is_prompt
    #  @param  data: 接收到的数据字符串。
    #  @return 布尔值，若包含提示符则返回 True，否则返回 False。
    def is_prompt(self,data):
        """检查数据中是否包含完整的提示符"""
        # 使用正则表达式匹配提示符 'nsh>'，并考虑可能的转义序列和换行符
        prompt_pattern = re.compile(r'nsh>\s*[\r\n]*\x1b\[K*\s*$')
        return bool(prompt_pattern.search(data))

    ## @brief 向飞控发送命令并接收响应。
    #  - @anchor SendCmdNsh
    #  @param  cmd: 要发送的命令字符串。
    #  @return 接收到的响应数据。
    def SendCmdNsh(self,cmd):
        try:
            output = ""
            next_heartbeat_time = timer()  # 下一个心跳包时间
            quit_time = None
            #self.mav_serialport.write('\n')  # 发送换行符，确保 shell 启动
            self.mav_serialport.write(cmd +'\n')
            #self.mav_serialport.write('\n')  # 发送换行符，确保 shell 启动

            while quit_time is None or quit_time > timer():
                data = self.mav_serialport.read(1024*10)  # 从 MAVLink 读取数据
                last_data_time = timer()  # 更新最后一次收到数据的时间
                time.sleep(1)
                if data and len(data) > 0:
                    sys.stdout.write(data)  # 输出数据
                    sys.stdout.flush()  # 刷新输出
                    # print(data)
                    last_data_time = timer()  # 更新最后一次收到数据的时间
                    output =  output + data
                    time.sleep(1)
                    # 检查数据中是否包含完整的提示符
                    if self.is_prompt(data):
                        sys.stdout.flush()  # 刷新输出

                        # output(data)
                        break
                else:
                    # 如果数据流在一段时间内没有更新，退出循环
                    if timer() - last_data_time >1:  # 超过2秒没有数据更新
                        sys.stdout.flush()  # 刷新输出
                        break

                # 处理心跳包发送
                heartbeat_time = timer()
                if heartbeat_time > next_heartbeat_time:
                    self.mav_serialport.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)  # 发送心跳包
                    next_heartbeat_time = heartbeat_time + 1  # 设置下一个心跳包时间

        except serial.serialutil.SerialException as e:
            print(e)  # 打印串口异常

        # except KeyboardInterrupt:
        #     mav_serialport.close()  # 关闭串口连接

        finally:
            print('')
            # print("cmd finish！！！")
        time.sleep(0.5)
        return output

    ## @brief 更新指定参数的值。
    #  - @anchor SendUpdateParam
    #  @param  ParamKey: 参数的名称。
    #  @param  ParamValue: 要更新的参数值。
    #  @return 无返回值。
    def SendUpdateParam(self,ParamKey,ParamValue):
        cmd = "param set " + ParamKey + " " + ParamValue
        self.SendCmdNsh(cmd)

    ## @brief 提交参数更改，使更改生效。
    #  - @anchor SendCommitParam
    #  @param  无
    #  @return 无返回值。
    def SendCommitParam(self):
        self.SendCmdNsh("param commit")
    
    ## @brief 修改指定参数的值
    #  - @anchor SendChangeParam
    #  @param  ParamKey: 参数的名称
    #  @param  ParamValue: 要更新的参数值 
    #  @return 无返回值。
    def SendChangeParam(self,ParamKey,ParamValue):
        cmd = "param set " + ParamKey + " " + ParamValue
        self.SendCmdNsh(cmd)
        self.SendCmdNsh("param commit")

    ## @brief 显示指定参数的当前值。
    #  - @anchor SendShowParam
    #  @param  ParamKey: 要显示的参数名称。
    #  @return 当前参数值的字符串表示。
    def SendShowParam(self,ParamKey):
        cmd = "param show " + ParamKey
        return self.SendCmdNsh(cmd)

    ## @brief 重启飞控。
    #  - @anchor SendRebootPX4
    #  @param  无
    #  @return 无返回值。
    def SendRebootPX4(self):
        cmd = "reboot"
        self.SendCmdNsh(cmd)

    ## @brief 关闭 MAVLink 串口。
    #  - @anchor SendClose
    #  @param  无
    #  @return 无返回值。
    def SendClose(self):
        time.sleep(0.5)
        self.mav_serialport.close()

    ## @brief 向飞控发送参数并接收响应
    #  - @anchor SendCmdNshS
    #  @param  ParamKey: 要发送给飞控的命令参数名。
    #  @return 接收到的响应数据。
    def SendCmdNshS(self,ParamKey):
        return self.SendCmdNsh(ParamKey)
    

    ## @brief 设置机架类型为 HIL。
    #  - @anchor SendAirframe
    #  @param  无
    #  @return 接收到的响应数据
    def SendAirframe(self):
        cmd = "param set SYS_AUTOSTART 1001"  # 发送 SYS_AUTOSTART 参数值 1001
        response = self.SendCmdNsh(cmd)
        return response

##  @class MavRflyShell
#   @brief 定义一个 MavRflyShell 类，用于
class MavRflyShell():
    
    ## @brief 获取主机上的可用串口。
    #  - @anchor getPX4Com
    #  @return 可用串口的列表。
    def getPX4Com(self):
         # 调用外部程序并捕获输出
            gotCom=False
            ComList=[]
            try:
                result = subprocess.run(r'%PSP_PATH%\CopterSim\GetComList.exe 3', shell=True, capture_output=True, text=True)

                if result.returncode == 0: # 正确执行命令
                    #print(result.stdout)
                    strSplt=result.stdout.split('#')
                    if len(strSplt)==3:
                        ComNum=strSplt[0]
                        ComNum=int(ComNum)
                        ComIdxStr=strSplt[1]
                        ComIdxStr=ComIdxStr.split(',')
                        ComInfoStr=strSplt[2]
                        ComInfoStr=ComInfoStr.split(';')
                        gotCom=True

                        print("=============检索主机串口设备中=============")
                        for i in range(len(ComInfoStr)):
                            print(ComInfoStr[i])
                        print("=============================================")
                        if ComNum<=0:
                            print('电脑上没有可用的飞控串口')
                        else:
                            print('电脑上有',ComNum,'个飞控串口分别为：')
                            for i in range(len(ComIdxStr)):
                                print('COM',ComIdxStr[i])
                            print('当前使用的串口号为COM',ComIdxStr[0])
                            print("=============================================")
                            return int(ComIdxStr[0])
                else: # 执行失败
                    pass

            except subprocess.CalledProcessError as e:
                print("命令执行失败，返回码：", e.returncode)
                print("错误信息：", e.stderr)
            print("=============================================")
            return 0


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
    
    ## @brief 检查 IP 地址是否符合标准格式。
    #  - @anchor validate_ip
    #  @param  ip: 要检查的 IP 地址字符串。
    #  @return 布尔值，若 IP 地址格式正确则返回 True，否则返回 False。
    def validate_ip(self, ip):
        """ 检查 IP 是否以 192.168.151 开头 """
        return ip.startswith("192.168.151.")

    ## @brief 根据目标飞控 ID 自动生成 IP 地址。
    #  - @anchor auto_configure_ip
    #  @param  targetID: 目标设备的 ID。
    #  @return 生成的 IP 地址字符串。
    def auto_configure_ip(self, targetID):
        """ 自动配置飞控的 IP 地址 """
        base_ip = self.getLocalIp()
        ip_list = base_ip.split('.')
        ip_last = 100 + targetID  # 假设从 192.168.151.101 开始
        new_ip = f"{ip_list[0]}.{ip_list[1]}.{ip_list[2]}.{ip_last}"
        return new_ip

    ## @brief 设置飞控的系统 ID 和 IP 地址，并进行参数配置。
    #  - @anchor setSysIDIP
    #  @param  com: 串口名称。
    #  @param  airplane_id: 飞机的系统 ID。 
    #  @param  baud: 串口波特率，默认为 921600。
    #  @param  is_last: 是否为最后一个设备，默认为 False。
    #  @return 无返回值。
    def setSysIDIP(self, com, airplane_id, baud=921600, is_last=False):

        sys_id = str(airplane_id)
        airplane_ip = self.auto_configure_ip(airplane_id)

        ipList = airplane_ip.split('.')
        dns_ip = ipList[0] + '.' + ipList[1] + '.' + ipList[2] + '.1'

        # 检查 IP 地址
        if not self.validate_ip(airplane_ip):
            print(f"错误: IP 地址 {airplane_ip} 不符合要求，必须以 192.168.151 开头。")
            return False

        dns_ip = f"{airplane_ip.rsplit('.', 1)[0]}.1"

        # 2. 写入飞控参数
        rflyShell = RflyShell('COM' + str(com), baud)
        rflyShell.SendAirframe()
        rflyShell.SendUpdateParam("MAV_SYS_ID", sys_id)
        rflyShell.SendCmdNsh("echo BOOTPROTO=static >> /fs/microsd/net.cfg")
        rflyShell.SendCmdNsh("echo IPADDR=" + airplane_ip + " >> /fs/microsd/net.cfg")
        rflyShell.SendCmdNsh("echo NETMASK=255.255.255.0 >> /fs/microsd/net.cfg")
        rflyShell.SendCmdNsh("echo ROUTER=" + dns_ip + " >> /fs/microsd/net.cfg")
        rflyShell.SendCmdNsh("echo DNS=" + dns_ip + " >> /fs/microsd/net.cfg")
        rflyShell.SendUpdateParam("MAV_2_CONFIG", str(1000))  # 设置为网口模式
        rflyShell.SendUpdateParam("MAV_2_BROADCAST", str(1))   # 设置为始终广播
        rflyShell.SendUpdateParam("MAV_2_MODE", str(8))   # 设置为外部消息模式
        rflyShell.SendUpdateParam("MAV_2_RATE", str(100000))   # 设置为100M
        rflyShell.SendUpdateParam("MAV_2_UDP_PRT", str(6000 + int(sys_id)))
        rflyShell.SendUpdateParam("MAV_2_REMOTE_PRT", str(6000 + int(sys_id)))

        # 3. 重启飞控
        rflyShell.SendRebootPX4()
        rflyShell.SendClose()

        print('请耐心等待，飞控重启中...')
        time.sleep(20)  # 等待飞控重启


        # 重新连接飞控
        rflyShell = self.connect_to_device('COM' + str(com), baud)
        if not rflyShell:
            return False  # 如果连接失败，则返回

        # 5. 读取新的配置参数
        output_text = rflyShell.SendShowParam("MAV_SYS_ID")
        match = re.search(r'MAV_SYS_ID \[.*?\] : (\d+)', output_text)

        rflyShell.SendClose()

        mav_sys_id_value = None
        if mav_sys_id_value is not None and mav_sys_id_value == sys_id:
            mav_sys_id_value = match.group(1)
            print(f'MAV_SYS_ID 的值是: {mav_sys_id_value}')
        else:
            print("===============未找到 MAV_SYS_ID 参数，即将重新查询==============")
            # 调用新的函数进行重新查询
            mav_sys_id_value = self.query_mav_sys_id(rflyShell, sys_id)

        if mav_sys_id_value == sys_id:
            print("==============飞控参数已正确配置，开始进行校验=============")
            self.check_udp_data(6000 + int(sys_id))  # 使用这个端口进行UDP校验
            time.sleep(5)
            print("===============UDP校验成功===============")

            # 根据 is_last 判断是否监控
            if not is_last:

                # 开始监控串口状态
                self.monitorSerialPorts(com)
                return True
        else:
            print("===============飞控参数配置有误===============")
            self.monitorSerialPorts(com)
            return False



    ## @brief 监控串口状态，检测新设备连接。
    #  - @anchor monitorSerialPorts
    #  @param  com: 串口名称。
    #  @return 无返回值。
    def monitorSerialPorts(self, com):
        print("请断开当前的飞控串口...")
        while True:
            # 检查串口是否仍然可用
            if self.checkSerialAvailable(com):
                print("当前串口仍然可用，请断开。")
            else:
                print("检测到串口已断开，准备检测新设备...")
                # 进入监测新设备的状态
                self.waitForNewDevice()
                break
            time.sleep(1)  # 每秒检查一次

    ## @brief 检查当前串口是否仍然可用
    #  - @anchor checkSerialAvailable
    #  @param  com: 串口名称
    #  @return 布尔值，检查的串口是否仍然可用
    def checkSerialAvailable(self, com):
        """ 检查当前串口是否仍然可用 """
        available_ports = self.getAvailableSerialPorts()
        return f'COM{com}' in available_ports
   
    ## @brief 获取当前可用的串口列表
    #  - @anchor getAvailableSerialPorts
    #  @param  无
    #  @return 当前可用的串口列表
    def getAvailableSerialPorts(self):
        """ 获取当前可用的串口列表 """
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append(port.device)  # 只获取设备名称，例如 COM5
        return ports

    ## @brief 等待用户插入新的飞控设备
    #  - @anchor waitForNewDevice
    #  @param  无
    #  @return 新设备的串口号。
    def waitForNewDevice(self):
        """ 等待用户插入新的飞控设备 """
        print("正在等待新的飞控设备连接...")
        while True:
            new_com = self.getPX4Com()  # 获取当前可用的飞控COM口
            if new_com > 0:
                print(f"检测到新设备，新的飞控串口是 COM{new_com}。")
                return new_com
            time.sleep(1)  # 每秒检查一次

    ## @brief 从指定的UDP端口读取数据
    #  - @anchor check_udp_data
    #  @param  port: 要读取数据的 UDP 端口。
    #  @return 接收到的 UDP 数据。
    def check_udp_data(self, port):
        """ 从指定的UDP端口读取数据 """
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind(('0.0.0.0', port))  # 绑定到指定端口

        udp_socket.settimeout(1)  # 设置超时时间为1秒

        try:
            while True:
                try:
                    data, addr = udp_socket.recvfrom(1024)  # 接收最多1024字节的数据
                    print(f"接收到来自 {addr} 的数据: {data}")

                    # 这里可以添加验证或处理接收到的数据的逻辑
                    break  # 收到数据后退出循环
                except socket.timeout:
                    print("未接收到数据... 正在重试...")

        finally:
            udp_socket.close()  # 关闭套接字

    ## @brief 连接到指定串口，包含错误处理与重试机制。
    #  - @anchor connect_to_device
    #  @param  port: 串口名称。
    #  @param  baud: 串口波特率。 
    #  @return 连接状态，成功返回 True，失败返回 False。
    def connect_to_device(self, port, baud):
        attempts = 0
        max_attempts = 5  # 最大尝试次数
        while attempts < max_attempts:
            try:
                rflyShell = RflyShell(port, baud)  # 尝试连接
                print(f"成功连接到: {port}")
                return rflyShell  # 成功返回
            except serial.serialutil.SerialException as e:
                print(f"尝试连接失败: {e}")
                attempts += 1
                time.sleep(2)  # 等待几秒再重试
            except PermissionError as e:
                print(f"权限错误: {e}")
                break  # 如果是权限错误，直接退出

        print(f"无法连接到 {port}，请检查设备。")
        return None
    
    ## @brief 重新查询 MAV_SYS_ID 参数，直到找到为止。
    #  - @anchor query_mav_sys_id
    #  @param  rflyShell: RflyShell 实例。
    #  @param  sys_id: 目标系统 ID。 
    #  @return 找到的系统 ID。
    def query_mav_sys_id(self, rflyShell, sys_id):

        start_time = time.time()
        while True:
            output_text = rflyShell.SendShowParam("MAV_SYS_ID")
            match = re.search(r'MAV_SYS_ID \[.*?\] : (\d+)', output_text)

            if match:
                mav_sys_id_value = match.group(1)
                print(f'MAV_SYS_ID 的值是: {mav_sys_id_value}')
                return mav_sys_id_value  # 返回找到的值

            # 检查是否超过 1 分钟
            if time.time() - start_time > 60:
                print("===================超时未找到 MAV_SYS_ID 参数。==============")
                return None  # 超时返回 None

            time.sleep(2)  # 每次重试之间稍作等待
