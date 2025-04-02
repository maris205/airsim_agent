
import socket
import threading
import time
from pymavlink.dialects.v20 import common as mavlink2
import os 
import time
import sys
import struct
import math
import sys
import copy
import shutil
## @file 
# 该文件定义了QGCCtrlAPI类，用于与QGroundControl进行通信。QGCCtrlAPI类提供了发送MAVLink命令、请求日志文件和复制日志文件到指定位置的功能。
# @anchor QGCCtrlAPI接口库文件

## @class QGCCtrlAPI
# @brief QGroundControl通信接口类。
# PX4 MAVLink listen and control API and RflySim3D control API
class QGCCtrlAPI:
    ## @brief 构造函数，初始化QGCCtrlAPI类的实例。
    # - @anchor __init__
    # @param ID QGroundControl的实例ID，默认为1。
    # constructor function
    def __init__(self,ID=1):

        self.f = fifo
        self.mav0 = mavlink2.MAVLink(self.f)
        self.QGCPath=os.environ['USERPROFILE']+'\\Documents\\QGroundControl\\Logs'
        self.udp_socketQGC = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socketQGC.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    ## @brief发送长格式的MAVLink命令至QGroundControl。
    # - @anchor SendQgcCmdLong
    # @param command MAVLink命令ID。
    # @param param1 至param7 命令参数，默认值为0。
    # @return 无
    def SendQgcCmdLong(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
            buf = self.mav0.command_long_encode(255, 0,
                                                command, 0,
                                                param1, param2, param3, param4, param5, param6, param7).pack(self.mav0)
            self.udp_socketQGC.sendto(buf, ('127.0.0.1', 14550))
    ## @brief从指定路径获取文本文件内容，并删除该文件。
    # - @anchor getTxtContent
    # @param flagFilePath 标志文件的路径。
    # @return 返回文件内容，若文件不存在或读取失败，返回空字符串。
    #获取日志文件名
    def getTxtContent(self,flagFilePath):
        file=open(flagFilePath, mode='r')
        lines=file.readlines()
        file.close()
        os.remove(flagFilePath)
        return lines[0].replace('\n', '')

    ## @brief将日志文件从源路径复制到目标路径。
    # - @anchor copyLogFile
    # @param source 源文件路径。
    # @param target 目标文件路径。
    # @return 无
    #将日志文件复制指定位置
    def copyLogFile(self,source,target):
        shutil.copyfile(source, target)
    ## @brief请求QGroundControl的日志文件。
    # - @anchor ReqQgcLog
    # @param timeout 请求日志的超时时间，默认为180秒。
    # @param CopterID 指定的无人机ID，默认为1。
    # @return 返回下载的日志文件名，如果请求失败或超时，返回空字符串。
    # 请求QGC日志，设定请求日志的ID号（默认为1），并设置超时时间100s
    def ReqQgcLog(self,timeout=180,CopterID=1):

        outLogName=''

        LogFilePath = self.QGCPath+'\\log.txt'
        hasLogFile=False

        errFilePath = self.QGCPath+'\\error.txt'
        hasErrFile=False

        shutil.rmtree

        #print('Delete log.txt and error.txt')
        if os.path.exists(LogFilePath):
            os.remove(LogFilePath)

        if os.path.exists(errFilePath):
            os.remove(errFilePath)


        # 发送下载日志的请求
        self.SendQgcCmdLong(42700,CopterID,0,0,0,0,0,0)
        #print('Send requst to QGC for log downloading...')


        startTime=time.time()
        while time.time()-startTime<=timeout: # 一直等待，直到超时时间
            if(os.path.exists(LogFilePath)):
                hasLogFile=True
                break

            if(os.path.exists(errFilePath)):
                hasErrFile=True
                break
            
            time.sleep(1)
        
        if hasLogFile:
            logName=self.getTxtContent(LogFilePath)
            if os.path.exists(self.QGCPath+'\\'+logName):
                shutil.copyfile(self.QGCPath+'\\'+logName,sys.path[0]+'\\'+logName)
                #print('Download log'+logName+' successfully.')
                outLogName=logName
                return outLogName
            else:
                print('Error content in log.txt: '+logName)

        if hasErrFile:
            errMsg = self.getTxtContent(errFilePath)
            print(errMsg)

        # 超时的情况
        if ~hasErrFile and ~hasLogFile:
            print('Timeout for waiting log download..')

        return outLogName


## @class fifo
# @brief 用于MAVLink消息处理的先进先出(FIFO)缓冲区类。
# define a class for MAVLink initialization
class fifo(object):
    ## @brief初始化FIFO类的实例。
    # - @anchor __init__
    def __init__(self):
        self.buf = []

    ## @brief向FIFO缓冲区写入数据。
    # - @anchor write
    # @param data 要写入的数据。
    # @return 返回写入的数据长度。
    def write(self, data):
        self.buf += data
        return len(data)

    ## @brief从FIFO缓冲区读取并移除数据。
    # - @anchor read
    # @return 返回缓冲区中的第一条数据，如果缓冲区为空，则返回None。
    def read(self):
        return self.buf.pop(0)
