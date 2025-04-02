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
#  @brief 这是一个集成与三维地形交互接口的模块。
#  @anchor UEMapServe接口库文件
#  对应例程链接见
#  @ref md_ue_2md_2UEMapServe



##  @brief 三维场景地图服务器类。
#  
#  此类通过读取地形灰度图和对应的元数据来获取指定位置的海拔高度。
#  
#  元数据中包含对图像坐标系与地理坐标系转换参数，以及图像的地理范围。
#  
#  点云数据供其他节点订阅。
#  @author 
#  @date 2024.06.03
class UEMapServe:
    ## @brief 初始化UEMapServe类的实例。
    #  - @anchor __init__
    #  @param name 地形图文件的名称（可选）。如果未提供名称，将初始化空的地形数据。
    def __init__(self, name=''):
        if name == '':
            ## @var PosOffsetX
            #  @brief X方向的偏移量。
            self.PosOffsetX = 0
            
            ## @var PosScaleX
            #  @brief X方向的缩放比例。
            self.PosScaleX = 0
            
            ## @var PosOffsetY
            #  @brief Y方向的偏移量。
            self.PosOffsetY = 0
            
            ## @var PosScaleY
            #  @brief Y方向的缩放比例。
            self.PosScaleY = 0
            
            ## @var xMax
            #  @brief 地图的最大X值。
            self.xMax = 0
            
            ## @var yMax
            #  @brief 地图的最大Y值。
            self.yMax = 0
            
            ## @var binmap
            #  @brief 存储地形数据的二进制数组。
            self.binmap = []
            
            ## @var GPS
            #  @brief 存储GPS数据的列表（如果存在）。
            self.GPS = None
        else:
            ## @brief 加载地形图数据。
            #  @param name 地形图文件的名称。
            self.LoadPngData(name)

    ## @brief 加载PNG图像和相关的TXT数据，并进行相应的处理。
    #  - @anchor LoadPngData
    #  @param name 地形图文件的名称。
    #  @return 无返回值。
    #  @exception 无异常抛出。
    #  @anchor LoadPngData_anchor    
    def LoadPngData(self, name):
        
        """

        解析
            psp_path = os.environ.get('PSP_PATH')
            获取指定的环境变量 PSP_PATH  
            if psp_path is None:  
                psp_path=r'C:\PX4PSP'
                如果没有成功获取路径，就使用PX4PSP默认路径
            PSPMapPath=psp_path+ r"\CopterSim\external\map"
            
            fileLoc=os.path.join(sys.path[0],name)
            fileLocTxt = fileLoc+'.txt'
            if not os.path.exists(fileLocTxt): 
                fileLoc=os.path.join(PSPMapPath,name)
                fileLocTxt = fileLoc+'.txt'
            
            如果当前目录没搜到，就搜索CopterSim目录
                if not os.path.exists(fileLocTxt): 
                    print("Failed to find file ",fileLocTxt)
                    sys.exit(0)
                
                CopterSim目录也没搜到就报错
            
            txtFile = open(fileLocTxt)
            line = txtFile.readline()
            txtFile.close()
                打开.txt文件并读取第一行内容，然后关闭文件。

            m_readData = line.split(',')
            if len(m_readData) != 9:
                sys.exit(0)
                将读取的行内容按照逗号分割，并检查分割后的数据是否正好为9个元素，如果不是则退出程序。

            for i in range(len(m_readData)):
                m_readData[i] = float(m_readData[i])
                将分割后的字符串数据转换为浮点数。

            rowmap = cv2.imread(fileLocPng, cv2.IMREAD_ANYDEPTH)
            rowmap = rowmap.astype(np.float32)
            rowmap = rowmap - 32768
                读取.png文件并将其转换为浮点型数组，然后将所有像素值减去32768以调整数据范围。

            rows = np.size(rowmap, 0)
            columns = np.size(rowmap, 1)
                获取rowmap的行数和列数。

            PosScaleX = (m_readData[0] - m_readData[3]) / (columns - 1)
            PosScaleY = (m_readData[1] - m_readData[4]) / (rows - 1)
                计算X和Y方向的缩放比例。

            PosOffsetX = m_readData[3]
            PosOffsetY = m_readData[4]
                设定X和Y方向的偏移量。

            intCol = int((m_readData[6] - PosOffsetX) / PosScaleX)
            intRow = int((m_readData[7] - PosOffsetY) / PosScaleY)
                计算目标位置在图像中的行和列。

            heightInit = float(rowmap[0, 0])
            heightFirst = float(rowmap[rows - 1, columns - 1])
            heightThird = float(rowmap[intRow, intCol])
                获取初始高度、末端高度和目标位置的高度。

            if abs(heightThird - heightFirst) <= abs(heightThird - heightInit):
                if abs(heightInit - heightThird) > 10:
                    PosScaleZ = (m_readData[5] - m_readData[8]) / (heightInit - heightThird)
                else:
                    PosScaleZ = 1
            else:
                if abs(heightThird - heightFirst) > 10:
                    PosScaleZ = (m_readData[2] - m_readData[8]) / (heightFirst - heightThird)
                else:
                    PosScaleZ = 1
                根据高度差计算Z方向的缩放比例。

            intPosInitZ = heightInit
            PosOffsetZ = m_readData[5]
                设定初始高度和Z方向的偏移量。

            self.PosOffsetX = PosOffsetX
            self.PosScaleX = PosScaleX
            self.PosOffsetY = PosOffsetY
            self.PosScaleY = PosScaleY
            self.xMax = abs(m_readData[0] / 100)
            self.yMax = abs(m_readData[1] / 100)
                将计算结果赋值给类的属性。
        """
        
        # 获取指定的环境变量 PSP_PATH  
        psp_path = os.environ.get('PSP_PATH')
        if psp_path is None:  # 如果没有成功获取路径，就使用PX4PSP默认路径
            psp_path=r'C:\PX4PSP'
        PSPMapPath=psp_path+ r"\CopterSim\external\map"
        
        fileLoc=os.path.join(sys.path[0],name)
        fileLocTxt = fileLoc+'.txt'
        if not os.path.exists(fileLocTxt): # 如果当前目录没搜到，就搜索CopterSim目录
            fileLoc=os.path.join(PSPMapPath,name)
            fileLocTxt = fileLoc+'.txt'
            if not os.path.exists(fileLocTxt): # CopterSim目录也没搜到就报错
                print("Failed to find file ",fileLocTxt)
                sys.exit(0)
        fileLocPng = fileLoc+'.png'
        if not os.path.exists(fileLocPng):
            print("Failed to find file ",fileLocPng)
            sys.exit(0)        
        txtFile = open(fileLocTxt)
        line = txtFile.readline()
        txtFile.close()
        m_readData = line.split(',')
        # 检查数据长度
        if len(m_readData) not in (9, 12):
            sys.exit(0)

        # 将数据转换为浮点数
        m_readData = [float(x) for x in m_readData]

        # 处理数据长度为12的情况，将最后3个元素存储为GPS数据
        if len(m_readData) == 12:
            self.GPS = m_readData[-3:]  # 将最后3个数存为GPS数据
            m_readData = m_readData[:9]  # 剩余前9个数据用于后续处理
        else:
            self.GPS = None  # 如果没有GPS数据，设置为None
        
        # for i in range(len(m_readData)):
        #     m_readData[i] = float(m_readData[i])
        #print(m_readData)
        rowmap = cv2.imread(fileLocPng,cv2.IMREAD_ANYDEPTH)
        rowmap=rowmap.astype(np.float32)
        rowmap = rowmap-32768
        rows=np.size(rowmap,0)
        columns=np.size(rowmap,1)
        
        PosScaleX = (m_readData[0]-m_readData[3])/(columns-1)
        PosScaleY = (m_readData[1]-m_readData[4])/(rows-1)

        PosOffsetX = m_readData[3]
        PosOffsetY = m_readData[4]

        intCol = int((m_readData[6]-PosOffsetX)/PosScaleX )
        intRow = int((m_readData[7]-PosOffsetY)/PosScaleY )
        
        heightInit = float(rowmap[0,0])
        heightFirst = float(rowmap[rows-1,columns-1])
        heightThird = float(rowmap[intRow,intCol])

        if abs(heightThird-heightFirst)<=abs(heightThird-heightInit):
            if abs((heightInit-heightThird))>10:
                PosScaleZ = (m_readData[5]-m_readData[8])/((heightInit-heightThird))
            else:
                PosScaleZ = 1
        else:
            if abs(heightThird-heightFirst)>10:
                PosScaleZ = (m_readData[2]-m_readData[8])/((heightFirst-heightThird))
            else:
                PosScaleZ = 1

        intPosInitZ = heightInit
        PosOffsetZ = m_readData[5]

        self.PosOffsetX=PosOffsetX
        self.PosScaleX=PosScaleX
        self.PosOffsetY=PosOffsetY
        self.PosScaleY=PosScaleY
        self.xMax=abs(m_readData[0]/100)
        self.yMax=abs(m_readData[1]/100)
        ## @var rows
        #  @brief 地形网格的行数
        self.rows=rows
        ## @var columns
        #  @brief 地形网格的列数
        self.columns=columns
        self.binmap= -(PosOffsetZ + ((rowmap)-intPosInitZ)*PosScaleZ)/100.0

    ## @brief 根据输入的X和Y坐标获取地形高度数据。
    #  - @anchor getTerrainAltData    
    #  @param xin: 输入的X坐标值。
    #  @param yin: 输入的Y坐标值。
    #  @return 返回插值后的地形高度数据。
    #  @exception 无异常抛出。
    def getTerrainAltData(self, xin, yin):
        """


        解析
            if len(self.binmap) == 0:
                print("Please load a map first!")
                sys.exit(0)
                检查是否已经加载了地形数据，如果没有加载则提示用户并退出程序。

            intCol = (xin * 100 - self.PosOffsetX) / self.PosScaleX + 1
            intRow = (yin * 100 - self.PosOffsetY) / self.PosScaleY + 1
                计算输入坐标在地形图中的对应列和行。

            intColInt = int(intCol)
            intRowInt = int(intRow)
                获取整数部分的列和行。

            a = intCol - intColInt
            b = intRow - intRowInt
                计算列和行的小数部分，用于双线性插值。

            intRowInt1 = intRowInt + 1
            intColInt1 = intColInt + 1
                获取下一列和行的索引，用于插值计算。

            m = self.rows
            n = self.columns
                获取地形图的行数和列数。

            if intColInt < 1:
                intColInt = 1
                intColInt1 = 1
                a = 0
                确保列索引在有效范围内，如果小于1则重置为1，并将小数部分设为0。

            if intColInt >= n:
                intColInt = n
                intColInt1 = intColInt
                a = 0
                确保列索引在有效范围内，如果大于等于列数则重置为最大列索引，并将小数部分设为0。

            if intRowInt < 1:
                intRowInt = 1
                intRowInt1 = 1
                b = 0
                确保行索引在有效范围内，如果小于1则重置为1，并将小数部分设为0。

            if intRowInt >= m:
                intRowInt = m
                intRowInt1 = intRowInt
                b = 0
                确保行索引在有效范围内，如果大于等于行数则重置为最大行索引，并将小数部分设为0。

            binmap = self.binmap
            intRowInt = intRowInt - 1
            intColInt = intColInt - 1
            intRowInt1 = intRowInt1 - 1
            intColInt1 = intColInt1 - 1
                调整索引值以匹配数组索引从0开始的特点。

            zz = (binmap[intRowInt, intColInt] * (1 - b) * (1 - a) +
                binmap[intRowInt1, intColInt] * b * (1 - a) +
                binmap[intRowInt, intColInt1] * (1 - b) * a +
                binmap[intRowInt1, intColInt1] * b * a)
                计算双线性插值，得到输入坐标对应的地形高度。

            return zz
                返回插值后的地形高度数据。
        """
   
        if len(self.binmap)==0:
            print("Please load a map first!")
            sys.exit(0) 
        intCol = (xin*100-self.PosOffsetX)/self.PosScaleX+1
        intRow = (yin*100-self.PosOffsetY)/self.PosScaleY+1

        intColInt=int(intCol)
        intRowInt = int(intRow)
        a=intCol-intColInt
        b=intRow-intRowInt

        intRowInt1=intRowInt+1; 
        intColInt1=intColInt+1; 
        m=self.rows
        n=self.columns
        if intColInt<1:
            intColInt=1
            intColInt1=1
            a=0

        if intColInt>=n:
            intColInt=n
            intColInt1=intColInt
            a=0

        if intRowInt<1:
            intRowInt=1
            intRowInt1=1
            b=0

        if intRowInt>=m:
            intRowInt=m
            intRowInt1=intRowInt
            b=0
        binmap=self.binmap
        intRowInt=intRowInt-1
        intColInt=intColInt-1
        intRowInt1=intRowInt1-1
        intColInt1=intColInt1-1
        zz=binmap[intRowInt,intColInt]*(1-b)*(1-a)+binmap[intRowInt1,intColInt]*b*(1-a)+binmap[intRowInt,intColInt1]*(1-b)*a+binmap[intRowInt1,intColInt1]*b*a
        return zz
    
    ## @brief 生成地形点云。
    #  - @anchor outTerrainPoint    
    #  @return point_cloud_data（元组列表）：包含点云数据的列表。
    #  @exception 无异常抛出。
    def outTerrainPoint(self):
        """
        

        该方法遍历地形网格中的每个单元格，计算对应的实际坐标（x, y），并使用
        getTerrainAltData方法获取这些坐标的高度（h）。结果是一个包含（x, y, h）
        元组的列表，代表地形。

            
        """
        point_cloud_data = []  # 初始化一个空列表以存储点云数据

        # 遍历地形网格中的每一行
        for row in range(self.rows):
            # 遍历地形网格中的每一列
            for col in range(self.columns):
                # 计算实际的x坐标
                x = (self.PosOffsetX + col * self.PosScaleX) / 100
                # 计算实际的y坐标
                y = (self.PosOffsetY + row * self.PosScaleY) / 100
                # 获取（x, y）位置的高度（z坐标）
                h = self.getTerrainAltData(x, y)
                # 将（x, y, h）元组添加到点云数据列表中
                point_cloud_data.append((x, y, h))

        # 返回点云数据列表
        return point_cloud_data

