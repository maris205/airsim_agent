import platform
import psutil
import subprocess
import win32com.client
import scipy.io
import winreg
import netifaces
import subprocess
import os
import datetime

## @file 
#  该脚本用于收集和诊断RflySim平台的运行环境和状态。脚本提供了一系列的函数，用于检测操作系统信息、RflySim安装状态、依赖环境、网络环境等，并将结果写入文件。
#  @anchor DeBug接口库文件


## @brief 将给定内容写入指定目录的文件中。
# - @anchor write_to_file
#  @param content 要写入文件的内容。
#  @param target_directory 目标目录路径。
def write_to_file(content, target_directory):
    # 获取当前时间
    current_time = datetime.datetime.now()
    
    # 格式化时间为年月日时分的形式作为文件名的一部分
    file_name ="RflySimDebugInfo-" + current_time.strftime("%Y%m%d%H%M") + ".infos"

    # 创建完整的文件路径
    file_path = os.path.join(target_directory, file_name)

    try:
        # 打开文件，如果不存在会自动创建
        with open(file_path, 'w') as file:
            # 写入内容
            content.encode('utf-8')
            file.write(content)
        print(f"已将本电脑中RflySim平台信息写入：{file_path}文件中。")
    except Exception as e:
        print(f"写入文件时出错：{e}")
        
## @brief 获取RflySim工具链的小版本号（发布日期）。
# - @anchor get_rfly_ver
# @param directory RflySim安装目录。
# @return 返回RflySim工具链的小版本号。
def get_rfly_ver(directory):
    # 获取RflySim工具链的小版本号(发布日期)
    rfly_ver = None
    latest_file_date = ""
    for file in os.listdir(directory):
        # 筛选以 "RflySim" 开头的文件
        if file.startswith("RflySim") and file.endswith(".txt"):
            # 提取日期信息
            file_date = file.split("-")[-1][:-4]  # 去掉文件后缀 ".txt"
            # 比较日期信息，更新最新的文件名
            if file_date > latest_file_date:
                rfly_ver = file[:-4]  # 去掉文件后缀 ".txt"
                latest_file_date = file_date

    return rfly_ver

## @brief 获取本机的所有本地IP地址。
# - @anchor get_local_ips
# @return 返回包含本机所有本地IP地址的列表。
def get_local_ips():
    # 获取本地IP地址列表
    interfaces = netifaces.interfaces()
    ips = []

    # 遍历每个接口
    for interface in interfaces:
        # 获取接口的地址信息
        addresses = netifaces.ifaddresses(interface)
        # 如果接口有IPv4地址，获取对应的IP地址和掩码
        if netifaces.AF_INET in addresses:
            for address in addresses[netifaces.AF_INET]:
                ip = address['addr']
                mask = address['netmask']
                ips.append((ip, mask))

    return ips

## @brief 检查IP地址间的连通性。
# - @anchor ping_all_ips
# @param ips 本地IP地址列表。
# @return 返回无法ping通的IP地址对列表。
def ping_all_ips(ips):
    ping_failures = []

    for i, (ip1, mask1) in enumerate(ips):
        for j, (ip2, mask2) in enumerate(ips[i+1:], start=i+1):
            # 检查是否同一网段
            if ip1.startswith(ip2[:ip2.rindex('.')]) or ip2.startswith(ip1[:ip1.rindex('.')]):
                result = subprocess.run(['ping', '-n', '1', '-w', '1000', ip2], capture_output=True, text=True)
                if "来自" not in result.stdout:
                    ping_failures.append((ip1, ip2))

    return ping_failures

## @brief 运行ipconfig命令并获取网络配置输出。
# - @anchor get_ipconfig_output
# @return 返回包含网络配置信息的列表。
def get_ipconfig_output():
    # 运行ipconfig检测网络环境
    # 执行 ipconfig 命令并获取输出
    result = subprocess.run(["ipconfig", "/all"], capture_output=True, text=True)
    ipconfig_output = result.stdout.splitlines()  # 按行分割输出并存储到数组中
    ipconfig_output = [line for line in ipconfig_output if line.strip()]  # 删除空行
    return ipconfig_output

## @brief 运行wsl命令并获取WSL状态信息。
# - @anchor get_wsl_info
# @return 返回包含WSL状态信息的列表。
# 运行cmd命令检测wsl状态信息
def get_wsl_info():
    result = subprocess.run(["wsl", "-l", "-v"], capture_output=True, text=True)
    wsl_info = []

    if result.returncode == 0:
        wsl_info = result.stdout.splitlines()
    else:
        wsl_info.append(f"Error: {result.stderr}")
    return wsl_info

## @brief 检测电脑中安装的所有软件。
# - @anchor get_installed_software
# @return 返回包含安装软件名称和版本的列表。
# 检测电脑中所有的软件
def get_installed_software():
    software_list = []
    registry_paths = [
        r"SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall",
        r"SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall"
    ]
    
    for registry_path in registry_paths:
        try:
            reg_key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, registry_path)
            for i in range(0, winreg.QueryInfoKey(reg_key)[0]):
                sub_key_name = winreg.EnumKey(reg_key, i)
                sub_key = winreg.OpenKey(reg_key, sub_key_name)
                try:
                    software_name = winreg.QueryValueEx(sub_key, "DisplayName")[0]
                    try:
                        software_version = winreg.QueryValueEx(sub_key, "DisplayVersion")[0]
                    except FileNotFoundError:
                        software_version = "Unknown version"
                    software_list.append((software_name, software_version))
                except FileNotFoundError:
                    continue
                finally:
                    sub_key.Close()
            reg_key.Close()
        except FileNotFoundError:
            continue

    return software_list

## @brief 在已安装软件列表中寻找特定软件。
# - @anchor find_uav_drive
# @param installed_software 已安装软件列表。
# @param SoftWareName 要寻找的软件名称。
# @return 返回包含特定软件名称的软件列表。
# 寻找特定的某个软件
def find_uav_drive(installed_software,SoftWareName):
    uav_drive_versions = [software for software in installed_software if SoftWareName in software[0]]
    return uav_drive_versions

## @brief 获取"文档"文件夹的位置。
# - @anchor get_documents_folder
# @return 返回"文档"文件夹的路径。
# 获取"文档"文件夹的位置。
def get_documents_folder():
    shell = win32com.client.Dispatch("WScript.Shell")
    documents_path = shell.SpecialFolders("MyDocuments")
    return documents_path

## @brief 获取显卡信息。
# - @anchor get_gpu_info
# @return 返回包含显卡信息的字符串。
# 获取显卡信息
def get_gpu_info():
    try:
        
        gpu_info = subprocess.check_output("wmic path win32_videocontroller get name,adapterram,driverdate /format:list", shell=True).decode()

        # 解析输出信息
        gpu_list = gpu_info.strip().split("\n\n")
        os_info = ""
        for gpu in gpu_list:
            if gpu.strip():  # 排除空项
                gpu_details = {}
                for item in gpu.strip().split("\n"):
                    if "=" in item:
                        key, value = item.split("=", 1)
                        gpu_details[key.strip()] = value.strip()

                os_info += f"显卡名称(型号)：{gpu_details.get('Name', 'N/A')}\n"
                gpu_ram_mb = int(gpu_details.get('AdapterRAM', 0)) / 1024 / 1024 / 1024
                os_info += f"显卡运存：{gpu_ram_mb:.2f} GB\n"
                os_info += f"显卡驱动日期：{gpu_details.get('DriverDate', 'N/A')}\n"
                os_info += "\n"

        return os_info
    except Exception as e:
        return f"无法获取GPU信息: {str(e)}\n"



## @brief 收集电脑系统信息。
# - @anchor Os_Info_Get
# @return 返回包含操作系统信息的字符串。
# 1.电脑系统检测
def Os_Info_Get():
    # Get OS information
    # os_info="<span style='font-size:18pt; font-weight:bold;'>版本</span> \n"
    os_info = f"版本: {platform.system()} {platform.release()}\n"
    os_info += f"系统版本号: {platform.version()}\n"
    os_info += f"系统架构: {platform.machine()}\n"
    os_info += f"处理器型号: {platform.processor()}\n"
    os_info += f"物理核心数: {psutil.cpu_count(logical=False)}\n"
    memory = psutil.virtual_memory()
    os_info += f"运行内存: {memory.total / (1024 ** 3):.2f} GB\n"
    os_info += f"当前占比: {memory.percent}\n"
    os_info += get_gpu_info()
    
    return os_info

## @brief 收集RflySim平台信息。
# - @anchor Rfly_Info_Get
# @return 返回包含RflySim平台信息的元组。
# 2.RflySim平台检测
def Rfly_Info_Get():
    documents_path = get_documents_folder()
    RflyMatFile=documents_path+'\MATLAB'+'\Add-Ons\Toolboxes\PX4PSP\code\+codertarget\+pixhawk\+CMAKE_Utils\FirmwareVersion.mat'
    mat_data = scipy.io.loadmat(RflyMatFile)

    # 访问具体的变量
    rfly_info = f"安装信息：{mat_data['__header__'].decode('utf-8')}\n" 
    rfly_dir=mat_data['RflyPSP_Px4_Base_Dir'][0]   
    rfly_info += f"RflySim工具链安装位置：{rfly_dir}\n"
    rfly_info += f"安装时所选PX4版本：{mat_data['FirmwareVersion'][0]}\n"
    rfly_info += f"PX4系列飞控编译命令：{mat_data['RflyPSP_Px4_Cmake_ConfigLast'][0]}\n"
    build_toolchain = mat_data['buildToolchain'][0]
    if build_toolchain=='1':
        b_t='WinWSL编译器'
    elif build_toolchain=='2':
        b_t='Msys2编译器'
    elif build_toolchain=='3':
        b_t='Cygwin编译器'
    else:
        b_t='error'
    rfly_info += f"PX4系列飞控编译命令：{b_t}\n"
    rfly_info += f"是否屏蔽掉PX4官方控制器输出：{mat_data['isBlockOutput_last'][0].encode('utf-8').decode('utf-8')}\n"
    rfly_info += f"RflySim工具链版本号：{mat_data['RflyVersionLast'][0][0]}\n"

    s_ver=get_rfly_ver(rfly_dir)
    rfly_info += f"RflySim工具链小版本号：{s_ver}\n"
    return rfly_info,rfly_dir,build_toolchain

## @brief 收集RflySim平台依赖环境信息。
# - @anchor RflyEnv_Info_Get
# @return 返回包含RflySim依赖环境信息的字符串。
# 3.RflySim平台依赖环境检测
def RflyEnv_Info_Get():
    rfly_info,rfly_dir,build_toolchain=Rfly_Info_Get()
    if build_toolchain=='1':
        rflyenv_info = f""
        w_info = get_wsl_info()
        wsl_list = [item.replace('\x00', '') for item in w_info if item != '\x00']
        rflyenv_info=f"WSL信息：\n"
        for line in wsl_list:
            rflyenv_info += f"{line}\n"
    else:
        rflyenv_info = f""

    installed_software = get_installed_software()
    uav_drive_info = find_uav_drive(installed_software,"UAV")
    rflyenv_info += f"UAV驱动信息：\n"
    for ver in uav_drive_info:
        rflyenv_info += f"  {ver}\n"

    matlab_drive_info = find_uav_drive(installed_software,"MATLAB")
    rflyenv_info += f"MATLAB软件信息：\n"
    for ver in matlab_drive_info:
        rflyenv_info += f"  {ver}\n"

    python_info = find_uav_drive(installed_software,"Python")
    rflyenv_info += f"Python环境信息：\n"
    for ver in python_info:
        rflyenv_info += f"  {ver}\n"
    
    vs_info1 = find_uav_drive(installed_software,"Visual Studio Community")
    vs_info2 = find_uav_drive(installed_software,"Visual Studio Professional")
    vs_info3 = find_uav_drive(installed_software,"Visual Studio Enterprise")
    rflyenv_info += f"Visual Studio软件信息：\n"
    for ver in vs_info1:
        rflyenv_info += f"  {ver}\n"
    for ver in vs_info2:
        rflyenv_info += f"  {ver}\n"
    for ver in vs_info3:
        rflyenv_info += f"  {ver}\n"
    
    return rflyenv_info

## @brief 收集网络环境信息。
# - @anchor NetEnv_Info_Get
# @return 返回包含网络环境信息的字符串。
# 4.网络环境检测
def NetEnv_Info_Get():
    netenv_info = f""
    local_ips = get_local_ips()
    ping_failures = ping_all_ips(local_ips)

    # 打印结果
    if ping_failures:
        netenv_info += f"同一网段的至少一对 IP 地址不能 ping 通，以下是不能 ping 通的 IP 地址对：\n"
        for ip1, ip2 in ping_failures:
            netenv_info += f"{ip1} -> {ip2}\n"
    else:
        netenv_info +=f"本电脑中所有 IP 地址均能 ping 通\n"
        for ip, mask in local_ips:
            netenv_info += f"{ip}：{mask}\n"

    ipconfig_output = get_ipconfig_output()
    netenv_info +=f"\n"
    netenv_info += f"本电脑中详细网络信息总览：\n"
    for line in ipconfig_output:
        netenv_info += f"{line}\n"
    return netenv_info

## @brief 组合所有诊断信息并转换为文本。
# - @anchor ComToTxt
# @return 返回包含所有诊断信息的字符串。
def ComToTxt():
    os_info=Os_Info_Get()
    rfly_info,rfly_dir,build_toolchain=Rfly_Info_Get()
    rflyenv_info=RflyEnv_Info_Get()
    netenv_info=NetEnv_Info_Get()
    
    content = f"{os_info}\n"
    content += f"**------------------------------------------------------------------**\n"
    content += f"{rflyenv_info}\n"
    content += f"**------------------------------------------------------------------**\n"
    content += f"{rfly_info}\n"
    content += f"**------------------------------------------------------------------**\n"
    content += f"{netenv_info}\n"
    
    return content

if __name__ == '__main__':
    content=ComToTxt()
    rfly_info,rfly_dir,build_toolchain=Rfly_Info_Get()
    write_to_file(content,rfly_dir)
    print('请将该文件发送给RflySim维护人员！')