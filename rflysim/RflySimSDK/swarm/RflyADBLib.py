import time
from ppadb.client import Client as AdbClient
import serial
import sys
import signal
import re
import subprocess
import os


## @file
#
#  @anchor RflyADBLib接口库文件
#  @brief 该库是一键配置150脚本相关的函数库。
#  @details 该库主要用于150飞机网络设置(热点和WIFI)、相机参数设置、相机启动、MAVLINK回传设置


##  @class SerialCommunication
#   @brief 类集成了串口传输相关方法。
class SerialCommunication:
    ## @brief 构造函数，设置串口端口和波特率。
    #  - @anchor __init__
    #  @param  port(str): 目标端口号，默认为'COM4'。
    #  @param  baudrate(int): 波特率默认为115200。 
    #  @param  debug_mode(bool): 是否启动调试模式，默认为False。      
    def __init__(self, port='COM4', baudrate=115200,debug_mode= False):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.Debug_Mode = debug_mode

        # 打开串口
        if self.ser.isOpen():
            print(f"Serial port {port} is open.")
        else:
            print(f"Serial port {port} is not open.")
            exit()

    ## @brief 发送命令。
    #  - @anchor send_command
    #  @param  command(str): 发送的命令。  
    #  @return 返回命令执行后的消息
    def send_command(self, command):
        # 向串口发送数据
        self.ser.write(( command  + '\n' ).encode('utf-8'))
        self.ser.flush()
        return self.read_serial_data()
    
    ## @brief 读取串口数据。
    #  - @anchor read_serial_data 
    #  @return 返回串口数据
    def read_serial_data(self):
        data = self.ser.read_all()  # 读取1个字节的数据
        if self.Debug_Mode:
            print(data.decode('utf-8'))
        return data.decode('utf-8')
    
    ## @brief 关闭串口。
    #  - @anchor close_serial
    def close_serial(self):
        # 关闭串口
        self.ser.close()
        print("Serial port closed.")


##  @class adb_communication
#   @brief 类集成了adb使用相关方法。
class adb_communication:
    ## @brief 构造函数，设置adb通信相关函数。
    #  - @anchor __init__
    #  @param  debug_mode(bool): 是否启动调试模式，默认为False。   
    def __init__(self,debug_mode = False):
        self.debug_mode = debug_mode
        # 连接到ADB服务器
        client = AdbClient(host="127.0.0.1", port=5037)
        # 获取连接的设备列表
        devices = client.devices()
        if devices:
            if len(devices) == 1:
                # 选择第一个连接的设备
                self.device = devices[0]
                # 获取设备的状态
                print("Connect Device serial:", self.device.serial)
            else:
               print("请移除手机等无关的USB设备，只保留一个RK3566板卡连接")
        else:
            print("No devices connected.")

    ## @brief 发送命令。
    #  - @anchor send_command
    #  @param  cmd(str): 发送的命令。  
    #  @return 返回命令执行后的消息
    def send_command(self,cmd):
        result = self.device.shell(cmd)
        if self.debug_mode:
            print(cmd)
            print(result)
        return result


##  @class wifi_command
#   @brief 类集成了所有的设置方法，通过调用不同方法进行设置。
class wifi_command:
    ## @brief 构造函数，设置脚本相关函数。
    #  - @anchor __init__
    #  @param  send_command_function: 发送命令的函数。   
    def __init__(self,send_command_function):
        self.send_command =  send_command_function

    ## @brief 打开wifi
    #  - @anchor open_wifi
    #  @return 返回命令执行后的消息
    def open_wifi(self):
        """打开wifi"""
        return self.send_command("nmcli radio wifi on").strip()

    ## @brief 检查无线网络接口(wlan0)的连接状态
    #  - @anchor current_wifi
    #  @return 返回命令执行后的消息
    def current_wifi(self):
        """检查无线网络接口(wlan0)的连接状态，并打印出当前连接的无线网络名称(SSID)"""
        return self.send_command("nmcli dev status |awk '$1== \"wlan0\" && $3==\"connected\" {print $4}'").strip()

    ## @brief 获取当前wifi的状态
    #  - @anchor statu_wifi
    #  @return 连接wifi的状态
    def statu_wifi(self):
        """获取当前wifi的状态"""
        # wifi_connect_statu =  (self.send_command("nmcli radio wifi").strip() == "\x1b[32menabled\x1b[0m")
        wifi_connect_statu = self.send_command("nmcli device show wlan0| awk '$1 ==\"GENERAL.STATE:\" {print $3}'").strip() == "(connected)"
        return wifi_connect_statu
    ## @brief 关闭wifi
    #  - @anchor close_wifi
    #  @return 返回命令执行后的消息
    def close_wifi(self):
        """关闭wifi"""
        return self.send_command("nmcli radio wifi off").strip()
    ## @brief 获取wifi的Ip
    #  - @anchor get_ip
    #  @return 返回命令执行后的消息
    def get_ip(self):
        """获取连接wifi的IP"""
        wifi_connect_ip = self.send_command("nmcli device show wlan0| awk '$1 ==\"IP4.ADDRESS[1]:\" {print $2}'|awk -F '/' '{print $1}'").strip()
        return wifi_connect_ip

    ## @brief 创建热点
    #  - @anchor create_wifihot
    #  @param  SSID(str): 创建热点的名称。
    #  @param  PASSWD(str): 创建热点的密码。
    #  @param  PASSWD(int): 创建热点的通道。
    #  @return 返回是否创建成功的布尔值
    def create_wifihot(self,SSID='AP_150_HOT',PASSWD="droneyee",CHANNEL=11):
        """创建热点"""
        result = self.send_command("nmcli device wifi hotspot ifname wlan0 con-name Hotspot ssid {} band bg channel {} password {}".format(SSID,CHANNEL,PASSWD)).strip()
        time.sleep(3)
        if(result.startswith("\x1b[2K\rDevice 'wlan0' successfully activated")):
            print("热点创建成功!!!")
            return True
        else:
            print("热点创建失败!!!")
            return False

    ## @brief 修改热点IP
    #  - @anchor modify_wifihot_ip
    #  @param  IPADDRESS(str): 期望的热点IP
    #  @return 返回命令执行后的消息
    def modify_wifihot_ip(self,IPADDRESS):
        """修改热点IP，不建议使用"""
        self.send_command("nmcli connection modify Hotspot ipv4.addresses {}/24".format(IPADDRESS)).strip()
        self.send_command("nmcli connection down Hotspot").strip()
        self.send_command("nmcli connection up Hotspot").strip()
        return self.send_command("nmcli device show wlan0| awk '$1 ==\"IP4.ADDRESS[1]:\" {print $2}'|awk -F '/' '{print $1}'").strip()

    ## @brief 获取热点的密码
    #  - @anchor get_wifihot_passwd
    #  @return 返回wifi密码
    def get_wifihot_passwd(self):
        """获取热点的密码"""
        return self.send_command("nmcli dev wifi show-password")

    ## @brief 连接wifi
    #  - @anchor connect_wifi
    #  @param  WIFI_SSID(str): 所要连接WIFI的SSID(一般是名称)
    #  @param  WIFI_PASSWD(str): 所要连接WIFI的密码
    #  @return 返回是否创建成功的布尔值
    def connect_wifi(self,WIFI_SSID,WIFI_PASSWD):
        """连接wifi"""
        if self.statu_wifi():
            print("wifi 已打开")
        else:
            self.open_wifi()
            time.sleep(1)

        # result = self.connect_wifi() 121.89.215.148:6379
        result = self.send_command("nmcli dev wifi connect {} password {}".format(WIFI_SSID,WIFI_PASSWD)).strip()

        if(result.startswith("\x1b[2K\rDevice 'wlan0' successfully activated")):
            print("网络连接成功!!!")
            return True
        elif(result.startswith("Error: No network with SSID")):
            print("没有搜索到此WIFI网络!!!")
        elif(result.startswith("Error: Connection activation failed: (7) Secrets were required, but not provided.")):
            print("WIFI密码输入错误!!!")
        return False

    ## @brief 设置相机命令到配置文件
    #  - @anchor echo_cmd
    #  @param  cmd(str): 要设置的命令
    #  @param  mode(int): 选择命令的添加模式,默认为0
    #  @return 返回命令执行后的消息
    def echo_cmd(self, cmd, mode=0):
        """进行相机参数设置，cmd为追加的命令,mode=0为追加,1为覆盖"""
        if mode ==0:
            # 进行追加
            cmd = "echo \"" + cmd + "\" >> /home/marvsmart/camera_udp/config.yaml"
            # print("追加命令:",cmd)
            return self.send_command(cmd).strip()
        elif mode ==1:
            # 进行覆盖
            cmd = "echo \"" + cmd + "\" > /home/marvsmart/camera_udp/config.yaml"
            # print("覆盖命令:",cmd)
            return self.send_command(cmd).strip()
    
    ## @brief 对给定路径的文件内容进行替换
    #  - @anchor replace_cmd
    #  @param  cmd(str): 要替换的命令
    #  @param  place(str): 要替换的行数
    #  @param  path(str): 要替换的路径
    #  @return 返回命令执行后的消息
    def replace_cmd(self, cmd, place, path):
        """cmd为期望替换的命令,place代表替换内容的行数,path为修改文件的路径
        注：对camera_upd/config.yaml修改时,对应位置为:
        enable:5/13 
        ip:9/17 
        compress_rate:8/16
        相机路径：/home/marvsmart/camera_udp/config.yaml
        自启动路径：/etc/./rc.local
        """
        place = str(place)
        cmd = f"sudo sed -i \'{place}c\\" + cmd + "\' " + path
        # print(cmd)
        return self.send_command(cmd).strip()

    ## @brief 对相机图像回传IP进行修改
    #  @param  ip(str): 相机回传图像的IP
    #  @param  cam(int): 要设置的相机标号(0或1)
    #  - @anchor replace_cam_ip
    def replace_cam_ip(self,ip,cam):
        """
        修改相机IP
        其中IP是期望的IP(给字符串),cam是修改哪个相机(0或1)"""
        path = "/home/marvsmart/camera_udp/config.yaml"
        cmd = "  router_ip: \"" + ip + "\""
        if cam == 0:
            self.replace_cmd(cmd, 9, path)
        elif cam == 1:
            self.replace_cmd(cmd, 17, path)
        else:
            print("请输入正确相机编号(0或1)")

    ## @brief 对相机图像压缩率进行修改
    #  @param  rate(int): 相机回传图像的压缩率
    #  @param  cam(int): 要设置的相机标号(0或1)
    #  - @anchor replace_cam_compress
    def replace_cam_compress(self,rate,cam):
        """其中rate是期望的图片压缩率,cam是修改哪个相机(0或1)"""
        path = "/home/marvsmart/camera_udp/config.yaml"
        cmd = "  compress_rate: " + str(rate) + " # jpg(0-100),png(0-9)"
        if cam == 0:
            self.replace_cmd(cmd, 8, path)
        elif cam == 1:
            self.replace_cmd(cmd, 16, path)
        else:
            print("请输入正确相机编号(0或1)")

    ## @brief 对相机图像使能端设置进行修改
    #  @param  enable(int): 要使能的相机标号(0或1)
    #  @param  cam(int): 要设置的相机标号(0或1)
    #  - @anchor replace_cam_ehable
    def replace_cam_ehable(self,enable,cam):
        """其中rate是期望的图片压缩率,cam是修改哪个相机(0或1)"""
        path = "/home/marvsmart/camera_udp/config.yaml"
        cmd = "  enable: " + str(enable)
        if cam == 0:
            self.replace_cmd(cmd,5, path)
        elif cam == 1:
            self.replace_cmd(cmd, 13, path)
        else:
            print("请输入正确相机编号(0或1)")

    ## @brief 对mavlink回传IP进行修改
    #  @param  ip(str): 回传的IP
    #  @param  port(int): 回传IP的端口
    #  - @anchor replace_mav_link
    def replace_mav_link(self, ip, port=15502):
        """设置mavlink回传的ip地址和端口"""
        path = "/etc/init.d/startMavRoute.sh"
        if isinstance(port, str):
            cmd = "sudo /home/mavlink-routerd -e " + ip + ":" + port + " /dev/ttyS7:115200 0.0.0.0:" + port
        elif isinstance(port, int):
            cmd = "sudo /home/mavlink-routerd -e " + ip + ":" + str(port) + " /dev/ttyS7:115200 0.0.0.0:" + str(port)
        else:
            print("请输入正确端口")
        # print(cmd)
        self.replace_cmd(cmd, 3, path)

    ## @brief 设置是否自启动热点
    #  @param  SSID(str): 自启动热点的名称, 默认为AP_150_HOT
    #  @param  PASSWD(str): 自启动热点的密码, 默认为droneyee
    #  @param  CHANNEL(int): 自启动热点的通道，默认为11
    #  @param  enable(int): 是否设置自启动热点，默认为1即启动
    #  - @anchor aotuStartWIFI
    def aotuStartWIFI(self, SSID='AP_150_HOT', PASSWD="droneyee", CHANNEL=11, enable=1):
        """用于设置自动启动WIFI热点或关闭"""
        path = "/etc/./rc.local"
        if enable == 1:
            cmd = "echo 'marvsmart' | nmcli device wifi hotspot ifname wlan0 con-name Hotspot ssid {} band bg channel {} password {}".format(SSID,CHANNEL,PASSWD)
        elif enable == 0:
            cmd = "\\"
        # print(cmd)
        self.replace_cmd(cmd, 3, path)

    ## @brief 开启相机
    #  - @anchor openCam
    #  @return 返回命令执行后的消息
    def openCam(self):
        cmd = "cd /home/marvsmart/camera_udp/ && ./camera"
        return self.send_command(cmd).strip()

    ## @brief 获取设备MAC地址
    #  - @anchor getMac
    #  @return 返回设备MAC地址
    def getMac(self):
        cmd = "ip link show"
        str = self.send_command(cmd).strip()
        Mac = extract_specific_mac(str)
        return Mac

    ## @brief 设置连接WIFI的静态IP
    #  - @anchor setStaticIp
    #  @param  SSID(str): 需要设置静态IP的WIFI的名称
    #  @param  ip(str): 期望的静态IP
    #  @return 返回命令执行后的消息
    def setStaticIp(self, SSID, ip):
        self.send_command(f"nmcli con modify {SSID} ipv4.addresses {ip}/24 ipv4.method manual").strip()
        self.send_command(f"nmcli con down {SSID}").strip()
        self.send_command(f"nmcli con up {SSID}").strip()
        return self.send_command("nmcli device show wlan0| awk '$1 ==\"IP4.ADDRESS[1]:\" {print $2}'|awk -F '/' '{print $1}'").strip()

    ## @brief 对指定IP进行ping操作
    #  - @anchor pingIp
    #  @param  ip(str): 指定的IP
    #  @return 网络是否ping通的布尔值
    def pingIp(self, ip):
        cmd = f"ping -c 1 {ip} > /dev/null 2>&1 && echo 0 || echo 1"
        return self.send_command(cmd)
    
## @brief 处理获取到的MAC地址消息
#  - @anchor extract_specific_mac
#  @param  interface_name(str): 设置指定网络
#  @return MAC地址
def extract_specific_mac(text, interface_name="wlan0"):
    # 定义正则表达式模式来匹配指定接口的MAC地址
    pattern = rf"{interface_name}.*?link/ether (\S+)" 
    
    # 使用正则表达式在字符串中查找指定接口的MAC地址
    match = re.search(pattern, text, re.DOTALL)
    
    if match:
        str = match.group(1)
        return str.replace(":", "_")
    else:
        return "没有找到Mac地址,请手动指定"
    
## @brief 对指定IP进行ping操作
#  - @anchor ping_ip
#  @param  ip_address(str): 需要PING的IP
#  @return 能否ping通的布尔值
def ping_ip(ip_address):
    try:
        result = subprocess.run(['ping', '-n', '1', ip_address], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output = result.stdout.decode('gbk')  # 使用 'gbk' 解码中文系统的输出
        if "无法访问目标主机" in output or "请求超时" in output:
            return False
        elif "来自" in output or "Reply from" in output:
            return True
        return False
    except Exception as e:
        print(f"Error occurred: {e}")
        return False


## @brief 对指定指定路径打开adb驱动程序
#  @param  path(str): adb所在的路径
#  - @anchor start_adb
def start_adb(path):
    # 进入adb所在的文件目录
    adb_directory = path  # 将路径替换为adb.exe所在的目录
    os.chdir(adb_directory)

    # 启动adb.exe
    subprocess.Popen(['adb.exe'], shell=True)

    # 等待adb启动
    subprocess.run(['adb.exe', 'start-server'], shell=True)

## @brief 检查是否有设备连接
#  - @anchor check_device_connected
#  @return 是否有设备连接的布尔值
def check_device_connected():
    # 检查设备列表
    result = subprocess.run(['adb.exe', 'devices'], stdout=subprocess.PIPE, shell=True)
    output = result.stdout.decode('utf-8').strip()

    # 解析输出，判断是否有设备连接
    devices = output.splitlines()[1:]  # 跳过第一行 "List of devices attached"
    if devices:
        for device in devices:
            if "device" in device:
                print("设备已连接:", device.split()[0])
                return True
    print("没有设备连接。")
    return False

## @brief 关闭adb服务
#  - @anchor stop_adb
def stop_adb():
    # 停止adb服务
    subprocess.run(['adb.exe', 'kill-server'], shell=True)
    print("ADB服务已关闭。")

## @brief 处理程序关闭情况
#  - @anchor signal_handler
def signal_handler(sig, frame):
    print("\n检测到结束信号，正在停止ADB服务...")
    stop_adb()
    sys.exit(0)


if __name__ == "__main__":
        # 注册信号处理器
        signal.signal(signal.SIGINT, signal_handler)
        # 调用函数启动adb并查看设备列表
        
        # 获取指定的环境变量 PSP_PATH  
        psp_path = os.environ.get('PSP_PATH')
        if psp_path is None:  # 如果没有成功获取路径，就使用PX4PSP默认路径
            psp_path=r'C:\PX4PSP'
        start_adb(psp_path + r"\RflySimAPIs\RflySimSDK\adb")
        
        check_device_connected()
        # 让程序保持运行，直到检测到Ctrl+C
        adb_communication = adb_communication(debug_mode= False)
        wifi_command = wifi_command(adb_communication.send_command)
        print("按 Ctrl+C 结束程序并关闭ADB服务。")
        while True:
            try:
                time.sleep(1)  # 等待1秒
            except KeyboardInterrupt:
                signal_handler(None, None)
        # ip_plane = wifi_command.get_ip()
        # print(ip_plane)
        # ip = "10.42.0.1"
        # if ping_ip(ip):
        #     print(f"{ip} is reachable")
        # else:
        #     print(f"{ip} is not reachable")
        #修改wifi的ip地址
        # str = wifi_command.pingIp("192.168.151.6")
        # print(str)









