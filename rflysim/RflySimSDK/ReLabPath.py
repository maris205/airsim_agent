# -*- coding: utf-8 -*-
import sys
import subprocess
import platform
import os, stat
import sysconfig
import re
try:
    import cv2
except:
    print('某些功能运行需要opencv库，请提前安装，确保平台功能的正常使用')
# try:
#     import pyulog
# except:
#     print('功能运行需要pyulog库，请提前安装，确保平台功能的正常使用')
# try:
#     import ctypes
# except:
#     print('功能运行需要ctypes库，请提前安装，确保平台功能的正常使用')
try:
    import pymavlink
except:
    print('某些功能运行需要pymavlink库，请提前安装，确保平台功能的正常使用')
try:
    import redis
except:
    #print('某些功能运行需要redis库，请提前安装，确保平台功能的正常使用')
    pass

if platform.system() == 'Windows':

    print(sys.base_prefix)
    basepath = sys.base_prefix

    if not (basepath[-8:] == "Python38"):
        print("The Python version maybe wrong, please confirm!")
        #sys.exit(0)

    rflyPath = basepath[:-9]
    print('RflySim install Path is:',rflyPath)

    pthPath = basepath+'\\Lib\\site-packages\\rflysim.pth'

    curPath = sys.path[0]

    print('Current Path is:',curPath)

    with open(pthPath, 'w') as f:
        f.write(curPath+'\n')
        f.write(curPath+'\\comm'+'\n')
        f.write(curPath+'\\ctrl'+'\n')
        f.write(curPath+'\\phm'+'\n')
        f.write(curPath+'\\swarm'+'\n')
        f.write(curPath+'\\test'+'\n')
        f.write(curPath+'\\ue'+'\n')
        f.write(curPath+'\\vision'+'\n')
        f.write(curPath+'\\word'+'\n')
    print("Lab path modfied.")

elif platform.system() == 'Linux':
    if not (sys.version[0:3] == '3.8' ):
        print("The Python version maybe wrong, please confirm!",sys.version[0:4])
        #sys.exit()
    import getpass
    try:
        import rospy
    except:
        print('功能运行需要rospy库，请提前安装，确保例程的正常使用')
    print('导入可以用')
    # try:
    #     import d3dshot
    # except:
    #     print('功能运行需要d3dshot库，请提前安装，确保例程的正常使用')
    
    type_shell = os.environ['SHELL'].split('/')[-1]
    bashName = ".{}rc".format(type_shell)
    bashapth=os.path.join(os.path.expanduser('~'), bashName)
    search_text = 'rflysimsdk'
    isInPath=False
    with open(bashapth, 'r') as file:
        for line in file:
            if re.search(search_text, line, re.IGNORECASE):
            	isInPath=True
                #break
            
    if isInPath:
        print("rflysimsdk is in path, trying to replace ...")
    else:
        print("rflysimsdk is not in path, trying to add ...")

    #command = 'sudo ls'
    #subprocess.run(command, shell=True)

    # pthPath = '{}/rflysim.pth'.format(sysconfig.get_paths()["platlib"])
    # curPath = sys.path[0]
    # print('curPath is:',curPath)
    # command = 'touch pthPath'
    # subprocess.run(command, shell=True)

    command_empty = "sed -i 's#.*RflySimSDK.*##gI' ~/.{}rc".format(type_shell)
    subprocess.run(command_empty, shell=True)
    
    curPath = sys.path[0]
    command_add = 'echo "export PYTHONPATH=\$PYTHONPATH:{}:{}/comm:{}/ctrl:{}/ue:{}/vision:{}/phm:{}/swarm:{}/test:{}/word" | tee -a ~/.{}rc'.format(curPath, curPath, curPath, curPath, curPath, curPath, curPath, curPath, curPath,type_shell)
    subprocess.run(command_add, shell=True)
    
    command2 = "source {}".format(bashapth)
    print(command2)
    subprocess.run(command2, shell=True, executable=os.environ['SHELL'])
    
    
    # 判断文件是否存在
    needModi=False
    file_path="/proc/sys/net/core/rmem_max"
    if os.path.isfile(file_path):
        print(f"{file_path} exists.")
        with open(file_path, 'r') as file:
            first_line = file.readline()
            #print(first_line)
            if "6000000" not in first_line:
                needModi=True
                print('Current udp max buffer size is '+first_line+', which need to be modified.')
                command2="sudo bash -c \"echo 6000000 > {}\"".format(file_path)
                print(command2)
                subprocess.run(command2, shell=True, executable=os.environ['SHELL'])
            else:
            	print('Udp_max_buffer already modified. skip...')
                
    if needModi:
        file_path="/etc/sysctl.conf"
        if os.path.isfile(file_path):
            with open(file_path, 'r') as file:
                for line in file:
                    if re.search("net.core.rmem_max = 6000000", line, re.IGNORECASE):
                        print("sysctl.conf already modified, skip.")
                        needModi=False
        
    if needModi:
        file_path="/etc/sysctl.conf"
        command2 = 'sudo bash -c "echo net.core.rmem_max = 6000000 >> {}"'.format(file_path)
        print(command2)
        subprocess.run(command2, shell=True, executable=os.environ['SHELL'])
        command2 = 'sudo sysctl -p'
        print(command2)
        subprocess.run(command2, shell=True, executable=os.environ['SHELL'])
                
    
    # # 判断文件是否存在
    # needModi=False
    # file_path="/sys/kernel/ipv4/tcp_rmem_max"
    # if os.path.isfile(file_path):
    #     print(f"{file_path} exists.")
    #     with open(file_path, 'r') as file:
    #         first_line = file.readline()
    #         #print(first_line)
    #         if first_line != "6000000":
    #             needModi=True
    #             print('Current udp max buffer size is '+first_line+', which need to be modified.')
    #             command2="sudo bash -c \"echo 6000000 > {}\"".format(file_path)
    #             print(command2)
    #             subprocess.run(command2, shell=True, executable=os.environ['SHELL'])
