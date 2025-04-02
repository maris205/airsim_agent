import socket
import time
import re 

## @file
#
#  @anchor RJ45_px6x接口库文件
#  @brief 本文件包含RJ_Device和TCPServer类的定义与相关功能实现。
#         使用socket进行网络设备的发现和管理，以及通过TCP服务端模式设置设备的网络连接模式。

## @class RJ_Device
#  @brief RJ_Device类用于通过广播AT指令发现网络设备，并对其进行重启、启动、修改ID、修改端口号、以及设置IP地址和DHCP状态等操作。
class RJ_Device:
    ## @brief 构造函数，初始化广播IP和端口号以及存储设备信息的字典。
    #  - @anchor __init__
    #  @param broadcast_ip(str): 广播IP地址，默认为'255.255.255.255'
    #  @param broadcast_port(int): 广播端口号，默认为5000    
    def __init__(self, broadcast_ip='255.255.255.255', broadcast_port=5000):
        self.broadcast_ip = broadcast_ip
        self.broadcast_port = broadcast_port
        self.response_devices = {}

    ## @brief 发现网络中的设备，并获取其设备ID、端口号和MAC地址信息。
    #  - @anchor discover_devices
    #  @param timeout(int): 接收响应的超时时间，默认为1秒
    def discover_devices(self, timeout=1):
        # 创建UDP套接字
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp_socket.settimeout(timeout)
        # 发送AT指令广播
        AT_COMMAND = b"AT\r\n"
        udp_socket.sendto(AT_COMMAND, (self.broadcast_ip, self.broadcast_port))

        # 接收设备响应
        try:
            
            while True:
                data, addr = udp_socket.recvfrom(1024)
                print(addr)
                time.sleep(0.1)
                # 发送查询设备ID命令
                DEVICE_ID_COMMAND = b"AT+WID=?\r\n"
                udp_socket.sendto(DEVICE_ID_COMMAND, (addr[0], addr[1]))
                # 接收设备ID响应
                device_id_data, _ = udp_socket.recvfrom(1024)
                print("device id = ",device_id_data)
                device_id = device_id_data.strip().decode().split(":")[-1]

                time.sleep(0.1)  # 等待一段时间

                # 发送查询端口号命令
                PORT_COMMAND = b"AT+PORT=?\r\n"
                udp_socket.sendto(PORT_COMMAND, (addr[0], addr[1]))
                # 接收端口号响应
                port_data, _ = udp_socket.recvfrom(1024)
                port = port_data.strip().decode().split(":")[-1]

                # 发送查询MAC地址命令
                MAC_COMMAND = b"AT+MAC=?\r\n"
                udp_socket.sendto(MAC_COMMAND, (addr[0], addr[1]))
                # 接收MAC地址响应
                mac_data, _ = udp_socket.recvfrom(1024)
                mac_address = mac_data.strip().decode().split(":")[-1]
                 
                # 将接收到的设备信息存储到字典中
                self.response_devices[device_id] = {"ip": addr[0], "port": int(port),"mac": mac_address}

        except socket.timeout:
            pass

        udp_socket.close()

    ## @brief 重启指定设备ID对应的设备。
    #  - @anchor reboot_device_by_id    
    #  @param device_id(str): 要重启的设备ID
    def reboot_device_by_id(self, device_id):
        if device_id in self.response_devices:
            device_info = self.response_devices[device_id]
            ip_address = device_info["ip"]
            port = device_info["port"] 

            # 先设置延迟单位，固定为1（例如1表示100毫秒 0表示1秒）
            DLY_UNIT_COMMAND = b"AT+DLYUNIT=1\r\n"
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # 发送延迟单位命令
            udp_socket.sendto(DLY_UNIT_COMMAND, (ip_address, port))
            time.sleep(0.1)  # 等待设备反应
                                             
            REBOOT_COMMAND = b"AT+STACH1=3,30\r\n"  #点动关，开启后延时3秒时间再开启，
            # 创建UDP套接字
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # 发送重启命令到指定设备
            udp_socket.sendto(REBOOT_COMMAND, (ip_address, port))
            udp_socket.close()
            print(f"向设备ID为 {device_id} 的设备发送重启命令")

    ## @brief 启动已发现的所有设备。
    #  - @anchor start_all_devices
    def start_all_devices(self):
        START_COMMAND = b"AT+STACH1=0\r\n"
        # 创建UDP套接字
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 发送启动命令到所有设备
        for device_info in self.response_devices.values():
            ip_address = device_info["ip"]
            port = device_info["port"]
            udp_socket.sendto(START_COMMAND, (ip_address, port))
            time.sleep(0.1) 

    ## @brief 获取指定设备ID的IP地址。
    #  - @anchor get_current_ip
    #  @param device_id(str): 目标设备的ID
    #  @return 返回设备的IP地址或None
    def get_current_ip(self, device_id):
        """
        :param device_id: 目标设备的ID
        :return: 设备的IP地址或None

        """
        if device_id in self.response_devices:
            device_info = self.response_devices[device_id]
            ip_address = device_info["ip"]
            print(f"当前设备 {device_id} 的IP地址为: {ip_address}")
            return ip_address
        else:
            print(f"未找到设备ID {device_id}")
            return None
        
    ## @brief 设置DHCP状态
    #  - @anchor set_dhcp
    #  @param device_id(str): 目标设备的ID
    #  @param enable(bool): True表示开启DHCP，False表示禁用DHCP
    def set_dhcp(self, device_id, enable):
        """
        :param device_id: 目标设备的ID
        :param enable: True表示开启DHCP，False表示禁用DHCP

        """

        if device_id in self.response_devices:
            device_info = self.response_devices[device_id]
            port = device_info["port"]
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # 设置DHCP
            if enable:
                DHCP_COMMAND = b"AT+DHCP=1\r\n"  # 开启DHCP
            else:
                DHCP_COMMAND = b"AT+DHCP=0\r\n"  # 禁用DHCP
            
            udp_socket.sendto(DHCP_COMMAND, (device_info["ip"], port))
            time.sleep(0.1)  # 等待设备反应
            print(f"设备 {device_id} 的DHCP状态已设置为 {'开启' if enable else '禁用'}")

            udp_socket.close()
        else:
            print(f"未找到设备ID {device_id}")

    ## @brief 设置静态IP、网关和子网掩码
    #  - @anchor set_static_ip
    #  @param device_id(str): 目标设备的ID
    #  @param ip_address(str): 要设置的IP地址
    #  @param gateway(str): 要设置的网关
    #  @param mask(str): 要设置的子网掩码
    def set_static_ip(self, device_id, ip_address, gateway, mask):
        """
        :param device_id: 目标设备的ID
        :param ip_address: 要设置的IP地址
        :param gateway: 要设置的网关
        :param mask: 要设置的子网掩码
        """
        if device_id in self.response_devices:
            device_info = self.response_devices[device_id]
            port = device_info["port"]
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # 禁用DHCP
            DHCP_COMMAND = b"AT+DHCP=0\r\n"
            udp_socket.sendto(DHCP_COMMAND, (device_info["ip"], port))
            time.sleep(0.1)  # 等待设备反应

            # 设置固定IP
            IP_COMMAND = f"AT+IP={ip_address}\r\n".encode()
            udp_socket.sendto(IP_COMMAND, (device_info["ip"], port))
            time.sleep(0.1)  # 等待设备反应

            # 设置网关
            GATEWAY_COMMAND = f"AT+GATEWAY={gateway}\r\n".encode()
            udp_socket.sendto(GATEWAY_COMMAND, (device_info["ip"], port))
            time.sleep(0.1)  # 等待设备反应

            # 设置子网掩码
            MASK_COMMAND = f"AT+MASK={mask}\r\n".encode()
            udp_socket.sendto(MASK_COMMAND, (device_info["ip"], port))
            time.sleep(0.1)  # 等待设备反应

            print(f"设备 {device_id} 的IP地址设置为 {ip_address}, 网关为 {gateway}, 子网掩码为 {mask}")

            udp_socket.close()
        else:
            print(f"未找到设备ID {device_id}")
    

    ## @brief 修改设备ID
    #  - @anchor modify_device_id
    #  @param device_id(str): 要修改的设备ID
    #  @return 返回修改后的设备ID
    def modify_device_id(self, device_id):
        if device_id in self.response_devices:
            device_info = self.response_devices[device_id]
            new_device_id = input("请输入新的设备ID: ")
            modify_id_command = f"AT+WID={new_device_id}\r\n".encode()
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.sendto(modify_id_command, (device_info["ip"], device_info["port"]))
            time.sleep(0.1)
            print(f"设备ID已修改为: {new_device_id}")

            #更新设备ID在字典中的键
            self.response_devices[new_device_id] = self.response_devices.pop(device_id)
            return new_device_id  # 返回新的设备ID
        else:
            print("未找到设备ID，无法修改。")
            return device_id  # 返回原设备ID

            
    ## @brief 修改设备端口号
    #  - @anchor modify_device_port
    #  @param device_id(str): 要修改端口号的设备ID
    def modify_device_port(self, device_id):
        if device_id in self.response_devices:
            device_info = self.response_devices[device_id]
            new_port = input("请输入新的端口号: ")
            if new_port.isdigit():
                modify_port_command = f"AT+PORT={new_port}\r\n".encode()
                udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                udp_socket.sendto(modify_port_command, (device_info["ip"], device_info["port"]))
                time.sleep(0.1)
                print(f"设备 {device_id} 的端口号已修改为: {new_port}")

                #更新端口号在字典中的值
                self.response_devices[device_id]["port"] = int(new_port)
                
            else:
                print("无效的端口号输入，未对端口号进行修改。")
        else:
            print("未找到设备ID，无法修改端口号。")


## @class TCPServer
#  @brief TCPServer类用于本机作为TCP服务端，与继电器端建立连接，对其进行连接模式的查询和修改
class TCPServer:
    # 连接模式常量
    TCP_SERVER_MODE = 0
    TCP_CLIENT_MODE = 1
    UDP_CLIENT_MODE = 2
    UDP_SERVER_MODE = 3

    ## @brief 构造函数，初始化TCP服务端模式相关信息，并尝试绑定指定的IP和端口。
    #  - @anchor __init__
    def __init__(self):
        self.current_mode = self.TCP_SERVER_MODE  # 默认连接模式
        self.local_ip = self.get_local_ip()
        self.HOST = self.local_ip if re.match(r"^192\.168\.1\.", self.local_ip) else None
        self.PORT = 6000  # 端口号
        self.BUFSIZ = 1024  # 接收数据缓冲大小
        self.ADDR = (self.HOST, self.PORT)
        self.tcpSerSock = None
        self.tcpCliSock = None

        if self.HOST is None:
            print(u'本机IP不在192.168.1内，程序终止。')
            exit(1)  # 如果不匹配，则终止程序

        self.setup_server()

    ## @brief 获取本机IP地址
    #  - @anchor get_local_ip
    #  @return 返回本机的局域网IP地址
    def get_local_ip(self):
        # 创建一个UDP套接字（IPv4）
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("192.168.1.190", 6000))  # Google公众DNS地址
            local_ip = s.getsockname()[0]  # 获取本地IP地址
        finally:
            s.close()  # 关闭套接字
        return local_ip

    ## @brief 设置TCP服务器，绑定地址和端口，并等待客户端连接。
    #  - @anchor setup_server
    def setup_server(self):
        print(u'本机作为服务端')
        print(u'本机IP：', self.HOST)
        print(u'端口：', self.PORT)

        self.tcpSerSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建TCP服务器套接字
        self.tcpSerSock.bind(self.ADDR)  # 套接字与地址绑定
        self.tcpSerSock.listen(5)  # 监听连接，同时连接请求的最大数目

        print(u'等待客户机的连接')
        self.tcpCliSock, addr = self.tcpSerSock.accept()  # 接收继电器端连接请求
        print(u'连接成功')
        print(u'客户端IP与端口如下：', addr)

    ## @brief 打印当前连接模式
    #  - @anchor print_current_mode
    def print_current_mode(self):
        """打印当前连接模式"""
        mode_names = {
            self.TCP_SERVER_MODE: "TCP_SERVER",
            self.TCP_CLIENT_MODE: "TCP_CLIENT",
            self.UDP_CLIENT_MODE: "UDP_CLIENT",
            self.UDP_SERVER_MODE: "UDP_SERVER"
        }
        print("当前连接模式：", mode_names[self.current_mode])

    ## @brief 修改连接模式，允许用户通过输入选择新的模式，并发送指令给设备。
    #  - @anchor change_connection_mode
    def change_connection_mode(self):
        """修改连接模式，允许用户选择模式"""
        print("请选择新的连接模式：")
        print("0 - TCP_SERVER")
        print("1 - TCP_CLIENT")
        print("2 - UDP_CLIENT")
        print("3 - UDP_SERVER")

        while True:
            try:
                mod = int(input("输入模式编号（0-3）："))  # 让用户输入模式编号

                if mod not in [0, 1, 2, 3]:
                    print("无效的模式，请重新选择（0-3）。")
                    continue

                # 构造命令并发送
                command = f"AT+MODEL={mod}\r\n"
                print(u'发送连接模式修改指令：', command)
                self.tcpCliSock.send(command.encode())  # 发送指令给设备

                # 接收设备的应答
                recv_data = self.tcpCliSock.recv(self.BUFSIZ)
                print(u'设备应答：')
                print(recv_data.decode('gbk'))
                print(u'模式修改指令执行成功!')

                # 更新当前模式
                self.current_mode = mod  # 更新当前模式
                self.print_current_mode()  # 再次打印当前模式

                break  # 退出循环

            except ValueError:
                print("输入无效，请输入数字。")

    ## @brief 释放资源，关闭套接字连接。
    #  - @anchor set_default_cleanup
    def set_default_cleanup(self):
        """释放资源，关闭套接字"""
        if self.tcpCliSock:
            self.tcpCliSock.close()  # 关闭客户端套接字
            print("客户端套接字已关闭。")
            self.tcpCliSock = None  # 将套接字引用设置为None，以避免悬空引用
        if self.tcpSerSock:
            self.tcpSerSock.close()  # 关闭服务器套接字
            print("服务器套接字已关闭。")
            self.tcpSerSock = None  # 将套接字引用设置为None
    
    ## @brief 将网络连接方式设置为默认的UDP_SERVER_MODE。
    #  - @anchor set_default_mode
    def set_default_mode(self):
        """默认修改网络连接方式为UDP_SERVER_MODE"""
        command = f"AT+MODEL={self.UDP_SERVER_MODE}\r\n"
        print(u'发送连接模式修改指令：', command)
        self.tcpCliSock.send(command.encode())