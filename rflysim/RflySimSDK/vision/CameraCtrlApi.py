from typing import Any
import cv2
import keyboard
import math
## @file
#  这是一个在图像上绘制各种元素功能和通过键盘控制的模块。
#  @anchor CameraCtrlApi接口库文件

## @class ImageCtrl
#  @brief ImageCtrl结构体类。用于在图像上绘制各种元素
class ImageCtrl:
    ## @brief ImageCtrl的构造函数
    # @param 初始化十字星、矩形框、坐标轴的信息
    def __init__(self, screenWidth=640, screenHeight=480) -> None:
        self.image = None
        self.centX = int(screenWidth/2)
        self.centY = int(screenHeight/2)
        #   十字星信息
        self.starMargin = 4  # 中心点间隔
        self.starWidth = 6  # 十字星的宽度
        self.lineWidth = 1  # 线的宽度
        self.lineColor = (0, 0, 255)
        self.valueColor = (0, 255, 255)
        #   十字星-矩形框信息
        self.rectWidthMargin = 60  # 宽度间隔
        self.rectHeightMargin = 40  # 高度间隔
        self.rectWidth = 20  # 矩形线条-宽
        self.rectHeight = 20  # 矩形线条-高
        #   坐标轴
        self.CoordStepWidth = 5  # 坐标轴单位宽度
        self.CoordTopMargin = 35  # top margin
        self.CoordButtomMargin = 40  # buttom margin
        self.CoordLeftMargin = 20  # left margin
        self.CoordRightMargin = 40  # right margin
        self.calibrationLength = 10  # 刻度的长度

    ## @brief 设置图像的中心点
    # #  - @anchor setRect
    # @param window_w  窗口宽度
    # @param window_h  窗口高度
    # @return 空
    def setRect(self, window_w, window_h):
        self.centX = int(window_w/2)
        self.centY = int(window_h/2)

    ## @brief 展示图像，将图像赋值给 self.image
    # #  - @anchor setRect
    # @param img 图像
    # @return 空
    def DisplayImg(self, img):
        self.image = img

    ## @brief 绘制十字星的中心点
    # #  - @anchor drawCross
    # @param 空
    # @return 空
    def drawCross(self):
        cv2.line(self.image, (self.centX, self.centY-self.starMargin),
                 (self.centX, self.centY-self.starMargin), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX, self.centY+self.starMargin),
                 (self.centX, self.centY+self.starMargin+self.starWidth), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX-self.starMargin, self.centY),
                 (self.centX-self.starMargin, self.centY), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX+self.starMargin, self.centY),
                 (self.centX+self.starMargin, self.centY), self.lineColor, self.lineWidth)
    #   绘制十字星外围矩形框信息

    ## @brief 绘制十字星外围的矩形框
    # #  - @anchor drawCrossRect
    # @param 空
    # @return 空
    def drawCrossRect(self):
        cv2.line(self.image, (self.centX-self.rectWidthMargin, self.centY-self.rectHeightMargin),
                 (self.centX-self.rectWidthMargin+self.rectWidth, self.centY-self.rectHeightMargin), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX-self.rectWidthMargin, self.centY-self.rectHeightMargin),
                 (self.centX-self.rectWidthMargin, self.centY-self.rectHeightMargin+self.rectHeight), self.lineColor, self.lineWidth)

        cv2.line(self.image, (self.centX-self.rectWidthMargin, self.centY+self.rectHeightMargin),
                 (self.centX-self.rectWidthMargin+self.rectWidth, self.centY+self.rectHeightMargin), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX-self.rectWidthMargin, self.centY+self.rectHeightMargin),
                 (self.centX-self.rectWidthMargin, self.centY+self.rectHeightMargin-self.rectHeight), self.lineColor, self.lineWidth)

        cv2.line(self.image, (self.centX+self.rectWidthMargin, self.centY-self.rectHeightMargin),
                 (self.centX+self.rectWidthMargin-self.rectWidth, self.centY-self.rectHeightMargin), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX+self.rectWidthMargin, self.centY-self.rectHeightMargin),
                 (self.centX+self.rectWidthMargin, self.centY-self.rectHeightMargin+self.rectHeight), self.lineColor, self.lineWidth)

        cv2.line(self.image, (self.centX+self.rectWidthMargin, self.centY+self.rectHeightMargin),
                 (self.centX+self.rectWidthMargin-self.rectWidth, self.centY+self.rectHeightMargin), self.lineColor, self.lineWidth)
        cv2.line(self.image, (self.centX+self.rectWidthMargin, self.centY+self.rectHeightMargin),
                 (self.centX+self.rectWidthMargin, self.centY+self.rectHeightMargin-self.rectHeight), self.lineColor, self.lineWidth)
    #   当前姿态信息

    ## @brief 绘制姿态信息，包括滚转角、俯仰角和偏航角
    # #  - @anchor drawPosture
    # @param roll 滚转角
    # @param pitch 俯仰角
    # @param yaw 偏航角
    # @return 空
    def drawPosture(self, roll, pitch, yaw):
        startPoint = [self.CoordLeftMargin, self.CoordTopMargin]
        rollStr = 'Roll(rad)  :' + str(round(roll, 6))
        pitchStr = 'Pitch(rad):' + str(round(pitch, 6))
        yawStr = 'Yaw(rad) :' + str(round(yaw, 6))
        cv2.putText(self.image, rollStr,
                    (startPoint[0], startPoint[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.valueColor)
        cv2.putText(self.image, pitchStr,
                    (startPoint[0], startPoint[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.valueColor)
        cv2.putText(self.image, yawStr,
                    (startPoint[0], startPoint[1]+40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.valueColor)

    ## @brief 绘制指定方向的坐标轴
    # #  - @anchor drawCoordinate
    # @param CoordName 坐标轴名称
    # @param orientation 方位
    # @param value 坐标轴上的值
    # @param step（默认值为1） 最小刻度
    # @param largeStep（默认值为4） 大刻度
    # @param minNum（默认值为0） 最小值
    # @param maxNum（默认值为100） 最大值
    # @param stepLength（默认值为2） 最小刻度长度
    # @return 空
    def drawCoordinate(self, CoordName, orientation, value, step=1, largeStep=4, minNum=0, maxNum=100, stepLength=2,):
        """
        @description  :
        @param  :
            CoordName:  坐标轴名称
            screenWidth：  窗口宽度
            screenHeight：  窗口高度
            orientation：   方位
            value：值
            step：最小刻度
            largeStep：大刻度
            minNum：最小值
            maxNum：最大值
            stepLength：最小刻度长度
        @Returns  :
        """
        stepNum = int(abs(maxNum-minNum)/step)  # 刻度数
        CoordinateLength = stepNum*stepLength  # 坐标轴总长度
        CoordNamePoint = [0, 0]  # 轴名称坐标
        CoordOriginPoint = [0, 0, 0, 0]  # 原点-终点坐标
        valuePoint = [0, 0, 0, 0]  # 值的起点和终点
        textPoint = [0, 0]
        valueLength = int(abs(value-minNum)/step*stepLength)  # 值的长度

        if orientation == 'top':
            CoordNamePoint = [
                self.centX-int(CoordinateLength/2)-len(CoordName)*10, self.CoordTopMargin]
            CoordOriginPoint = [
                self.centX-int(CoordinateLength/2), self.CoordTopMargin, self.centX+int(CoordinateLength/2), self.CoordTopMargin]
            valuePoint = [self.centX-int(CoordinateLength/2)+valueLength, self.CoordTopMargin, self.centX-int(
                CoordinateLength/2)+valueLength, self.CoordTopMargin-self.calibrationLength-5]
            textPoint = [
                self.centX-int(CoordinateLength/2)+valueLength, self.CoordTopMargin-self.calibrationLength]

            for i in range(stepNum+1):

                addLenth = i*stepLength
                if (i == 0 or (i != 0 and (i % largeStep) == 0)):
                    cv2.line(self.image, (CoordOriginPoint[0]+addLenth, self.CoordTopMargin), (CoordOriginPoint[0] +
                             addLenth, self.CoordTopMargin-self.calibrationLength-4), self.lineColor, self.lineWidth)  # largeStep
                else:
                    cv2.line(self.image, (CoordOriginPoint[0]+addLenth, self.CoordTopMargin), (CoordOriginPoint[0] +
                             addLenth, self.CoordTopMargin-self.calibrationLength), self.lineColor, self.lineWidth)  # calibration
        elif orientation == 'left':
            CoordNamePoint = [self.CoordLeftMargin,
                              self.centY-int(CoordinateLength/2)-5]
            CoordOriginPoint = [self.CoordLeftMargin, self.centY-int(
                CoordinateLength/2), self.CoordLeftMargin, self.centY+int(CoordinateLength/2)]
            valuePoint = [self.CoordLeftMargin, self.centY-int(CoordinateLength/2)+valueLength,
                          self.CoordLeftMargin+self.calibrationLength+5, self.centY-int(CoordinateLength/2)+valueLength]
            textPoint = [self.CoordLeftMargin+self.calibrationLength+10,
                         self.centY-int(CoordinateLength/2)+valueLength]

            for i in range(stepNum+1):
                addLenth = i*stepLength
                if (i == 0 or (i != 0 and (i % largeStep) == 0)):
                    cv2.line(self.image, (CoordOriginPoint[0], CoordOriginPoint[1]+addLenth), (CoordOriginPoint[0] +
                             self.calibrationLength+4, CoordOriginPoint[1]+addLenth), self.lineColor, self.lineWidth)  # largeStep
                else:
                    cv2.line(self.image, (CoordOriginPoint[0], CoordOriginPoint[1]+addLenth), (CoordOriginPoint[0] +
                             self.calibrationLength+4, CoordOriginPoint[1]+addLenth), self.lineColor, self.lineWidth)  # calibration

        elif orientation == 'buttom':
            value
        elif orientation == 'right':
            value
        else:
            return
        cv2.putText(self.image, CoordName, (CoordNamePoint[0], CoordNamePoint[1]),
                    cv2.FONT_HERSHEY_COMPLEX, 0.4, self.valueColor)  # Coordinate name
        cv2.line(self.image, (CoordOriginPoint[0], CoordOriginPoint[1]), (
            CoordOriginPoint[2], CoordOriginPoint[3]), self.lineColor, self.lineWidth)  # Coordinate

        cv2.line(self.image, (valuePoint[0], valuePoint[1]), (
            valuePoint[2], valuePoint[3]), self.valueColor, self.lineWidth)  # value
        cv2.putText(self.image, str(
            value), (textPoint[0], textPoint[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.valueColor)  # value-text

## @class KeyCtrl
#  @brief KeyCtrl结构体类。用于通过键盘控制吊舱相机的运动和参数调节
class KeyCtrl:
    '''
        该类是通过键盘控制吊舱相机运动
            上(↑)下(↓)键控制俯仰角(pitch)；
            左(←)右(→)键控制偏航角(yaw);
            右Ctrl建 + 左(←)右(→) 控制横滚角(roll)
            焦距调节 alt+上, alt+下
    '''

    ## @brief KeyCtrl的构造函数
    # @param 初始化各种参数
    def __init__(self):
        '''
            增加参数
                mask_***: &比较, 0: 未激活  1: 激活
                press_**: 按键控制
                FOVOrFocalLength: 视场角/焦距       模式切换的时候会同步焦距/视场角
                CameraFocalLength: 焦距
                CmaeraFocalLengthBaseNum: 焦距步长
                AngOrAngV: 角度/角速度      切换至角度模式控制的时候会同步下当前角度
                AngVel: 角速度
                CurOZ: 当前变倍, 默认1
                OZBaseNum: 变倍基数
                TrackID: 追踪目标的ID

            控制按键
                上、下、左、右: Pitch、Yaw角度和角速度控制
                Num1: 角度/角速度控制模式切换
                Num3: 焦距/视场角控制模式切换
                Num5: 相机回中(只调整角度为初始化状态, 不会调整焦距)
                Num7、9: 焦距/视场角控制
                +、-: 光学变倍, 基于基数变倍 (非当前焦距)
                /: 追踪默认目标ID(1000 仅供测试
                *: 切换传感器类型(0:图像 1:热力图 2:红外灰度)
        '''
        
        self.mask_FOV = 1 << 6          # 视场角
        self.mask_Ang = 1 << 7          # 角度
        self.mask_Quat = 1 << 8         # 四元数
        self.mask_FocalLength = 1 << 9  # 焦距
        self.mask_BackCenter = 1 << 10  # 回中
        self.mask_AngVel = 1 << 11      # 角速度
        self.mask_OpticalZoom = 1 << 12 # 光学变倍
        self.mask_OpticalBase = 1 << 13 # 变倍基数
        self.mask_Track = 1 << 14 # 目标
        self.mask_Distance = 1 << 15    # 测距
        self.mask_SensorType = 1 << 16  # 传感器类型（0:图像 1:热力图 2:红外灰度）
        self.bitMask = self.mask_FOV + self.mask_Ang + self.mask_Distance       # 基准参数
        self.AngFlag = 1                # 标志位，每次从角速度控制切换至角度控制的时候，更新当前角度数据
        self.AngOrAngV = 0              # 默认为角度控制
        self.AngEular = [0, 0, 0]       # 当前角度
        self.AngVel = [0, 0, 0]         # 角速度
        # self.offset = math.pi / 180  # 每次偏移量1个degrees
        self.offset = 1  # 每次偏移量1个degrees
        self.angle_scalar = 1  # 最后每次增加角度为 angle_scalar * offset
        self.FOV_offset = 0.01
        self.FOV_scalar = 0  # 这个值应该再（-1，1）之间
        self.mode = 1  # 0:表示松开键盘保持模式,1 表示松开键盘，恢复模式
        self.press_up = keyboard.KeyboardEvent('down', 0, 'up')  # ↑被按下
        self.press_down = keyboard.KeyboardEvent('down', 1, 'down')  # ↓被按下
        self.press_right = keyboard.KeyboardEvent('down', 2, 'right')  # →被按下
        self.press_left = keyboard.KeyboardEvent('down', 3, 'left')  # 左被按下
        self.press_r_ctrl = keyboard.KeyboardEvent(
            'down', 4, 'right ctrl')  # 右ctrl被按下
        self.press_r_alt = keyboard.KeyboardEvent(
            'down', 5, 'right alt')  # 左alt被按下
        self.press_CenterBack = keyboard.KeyboardEvent('down', 6, '5')  # Num5 被按下
        self.press_AngOrAngV = keyboard.KeyboardEvent('down', 7, '1')   # Num1 被按下
        self.press_FovOrFL = keyboard.KeyboardEvent('down', 8, '3')     # Num3 被按下
        self.press_Ang_Sub = keyboard.KeyboardEvent('down', 9, '7')     # Num7 被按下
        self.press_Ang_Add = keyboard.KeyboardEvent('down', 10, '9')    # Num9 被按下
        self.press_OZ_Next = keyboard.KeyboardEvent('down', 11, '+')    # + 被按下
        self.press_OZ_Back = keyboard.KeyboardEvent('down', 12, '-')    # - 被按下
        self.press_Track = keyboard.KeyboardEvent('down', 13, '/')      # / 被按下
        self.press_Sensor = keyboard.KeyboardEvent('down', 14, '*')     # * 被按下
        self.is_press_ctrl = False
        self.is_press_alt = False
        self.count = 0  # 因为读键盘的频率太快，所以需要做些处理
        self.FOVOrFocalLength = 0   # 默认视场角
        self.CameraFlag = 1         # 标志位，每次切换状态的时候从UE端同步当前视场角/焦距
        self.CameraFOV = 90         # 焦距  单位:度
        self.CameraFocalLength = 18  # 焦距mm
        self.CmaeraFocalLengthBaseNum = 1   # Step
        self.CurOZ = 1      # 当前变倍
        self.OZBaseNum = 4  # 变倍基数
        self.TrackID = 0    # 追踪目标ID
        self.SensorTypeID = 0   # 0:图像 1:热力图 2:红外灰度
        self.initkey()

    ## @brief 获取相机的当前偏航角
    # #  - @anchor getYaw
    #  @param 无
    #  @return 相机的当前偏航角
    def getYaw(self):
        return self.AngEular[2]

    ## @brief 获取相机的当前俯仰角
    # #  - @anchor getPitch
    #  @param 无
    #  @return 相机的当前俯仰角
    def getPitch(self):
        return self.AngEular[1]

    ## @brief 获取相机的当前滚转角
    # #  - @anchor getRoll
    #  @param 无
    #  @return 相机的当前滚转角
    def getRoll(self):
        return self.AngEular[0]

    ## @brief 获取相机的当前视场角
    # #  - @anchor getCameraFOV
    #  @param 无
    #  @return 相机的当前视场角
    def getCameraFOV(self):
        return self.CameraFOV

    ## @brief 获取相机的当前欧拉角
    # #  - @anchor getRadiansAngEular
    #  @param 无
    #  @return 相机的当前欧拉角
    def getRadiansAngEular(self):
        return [math.radians(self.AngEular[0]), math.radians(
            self.AngEular[1]), math.radians(self.AngEular[2])]
    # def TrackExit(self):
    #     if(self.bitMask & self.mask_Track):
    #         if(self.bitMask & self.mask_AngVel):
    #             self.AngOrAngV = 1
    #         else:
    #             self.AngOrAngV = 0
    #             self.AngFlag = 1
    #         self.bitMask -= self.mask_Track

    ## @brief 控制俯仰角的增大，根据当前模式（角度或角速度）进行相应的处理
    # # - @anchor upPress
    #  @param 无
    #  @return 无
    def upPress(self):
        if(self.AngOrAngV == 0):
            self.AngEular[1] += self.angle_scalar * self.offset
            if(self.AngEular[1] > 180):
                self.AngEular[1] -= 360
        elif(self.AngOrAngV == 1):
            self.AngVel[1] += 1
        else:       # 追踪状态会先退出
            self.AngOrAngV = 0
            self.AngFlag = 1

    ## @brief 控制俯仰角（Pitch）的向下调整
    # # - @anchor downPress
    #  @param 无
    #  @return 无
    def downPress(self):
        if(self.AngOrAngV == 0):
            self.AngEular[1] -= self.angle_scalar * self.offset
            if(self.AngEular[1] < -180):
                self.AngEular[1] += 360
        elif(self.AngOrAngV == 1):
            self.AngVel[1] -= 1
        else:
            self.AngOrAngV = 0
            self.AngFlag = 1

    ## @brief 控制偏航角（Yaw）的向右调整
    # # - @anchor rightPress
    #  @param 无
    #  @return 无
    def rightPress(self):
        if(self.AngOrAngV == 0):
            self.AngEular[2] += self.angle_scalar * self.offset
            if(self.AngEular[2] > 180):
                self.AngEular[2] -= 360
        elif(self.AngOrAngV == 1):
            self.AngVel[2] += 1
        else:
            self.AngOrAngV = 0
            self.AngFlag = 1

    ## @brief 控制偏航角（Yaw）的向左调整
    # # - @anchor leftPress
    #  @param 无
    #  @return 无
    def leftPress(self):
        if(self.AngOrAngV == 0):
            self.AngEular[2] -= self.angle_scalar * self.offset
            if(self.AngEular[2] < -180):
                self.AngEular[2] += 360
        elif(self.AngOrAngV == 1):
            self.AngVel[2] -= 1
        else:
            self.AngOrAngV = 0
            self.AngFlag = 1

    ## @brief 控制横滚角（Roll）的向左调整 
    # # - @anchor rightPress
    #  @param 无
    #  @return 无
    def ctrlLeftPress(self):
        self.AngEular[0] += self.angle_scalar*self.offset
        if(self.AngEular[0] > 180):
            self.AngEular[0] -= 360

    ## @brief 控制横滚角（Roll）的向右调整
    # # - @anchor leftPress
    #  @param 无
    #  @return 无
    def ctrlRightPress(self):
        self.AngEular[0] -= self.angle_scalar*self.offset
        if(self.AngEular[0] < -180):
            self.AngEular[0] += 360

    ## @brief 控制视场角（FOV）的缩小
    # # - @anchor altUpPress
    #  @param 无
    #  @return 无   
    def altUpPress(self):  # 需要根据需求来确定这个值
        self.FOV_scalar -= self.FOV_offset
        if(self.FOV_scalar < -1):
            self.FOV_offset = -1
        self.CameraFOV = self.FOV_scalar * 90 + 90

    ## @brief 控制视场角（FOV）的放大
    # # - @anchor altDownPress
    #  @param 无
    #  @return 无
    def altDownPress(self):
        self.FOV_scalar += self.FOV_offset
        if(self.FOV_scalar > 1):
            self.FOV_scalar = 1
        self.CameraFOV = self.FOV_scalar * 90 + 90

    ## @brief 将相机的滚转角、俯仰角和偏航角设置为当前的滚转角、俯仰角和偏航角
    # # - @anchor Reset
    #  @param 无
    #  @return 无  
    def Reset(self):
        yaw = self.getYaw()
        roll = self.getRoll()
        pitch = self.getPitch()
        self.inner(yaw, 2)
        self.inner(pitch, 1)
        self.inner(roll, 0)

    ## @brief 根据当前角度值和偏移量将角度归零
    # # -@anchor inner
    # @param angle 角度值
    # @param i 欧拉角中的第i+1位
    # @return 无
    def inner(self, angle, i):
        if(abs(angle / self.offset) < 1):
            self.AngEular[i] = 0
        elif(angle < 0):
            self.AngEular[i] += self.angle_scalar * self.offset
        else:
            self.AngEular[i] -= self.angle_scalar * self.offset

    ## @brief 静态方法，将角度限制在[-π, π]范围内
    # # -@anchor fun
    # @param 无
    # @param 无
    # @return 无
    def fun(angle):
        if angle < -math.pi:
            return angle+2*math.pi
        elif angle > math.pi:
            return angle - 2*math.pi

    ## @brief 将相机的所有角度限制在[-π, π]范围内
    # # -@anchor CheckAngleWid
    # @param 无
    # @param 无
    # @return 无
    def CheckAngleWid(self):
        '''每个角度范围(-pi,pi)'''
        self.AngEular[0] = self.fun(self.AngEular[0])
        self.AngEular[1] = self.fun(self.AngEular[1])
        self.AngEular[2] = self.fun(self.AngEular[2])

    ## @brief 处理键盘事件的回调方法
    # # -@anchor callback
    # @param key_v 键盘事件
    # @return 无
    def callback(self, key_v):
        if(key_v.event_type == 'down'):  # 按下键
            if(key_v.name == self.press_r_ctrl.name):
                self.is_press_ctrl = True
            if(key_v.name == self.press_r_alt):
                self.is_press_alt = True

            self.count += 1
            if(self.count % 100 == 0):
                self.count = 0
            else:
                return
            if(key_v.name == self.press_up.name and not self.is_press_alt):
                self.upPress()
            if(key_v.name == self.press_down.name and not self.is_press_alt):
                self.downPress()
            if(key_v.name == self.press_left.name and not self.is_press_ctrl):
                self.leftPress()
            if(key_v.name == self.press_right.name and not self.is_press_alt):
                self.rightPress()
            if(key_v.name == self.press_up.name and self.is_press_alt):
                self.altUpPress()
            if(key_v.name == self.press_down.name and self.is_press_alt):
                self.altDownPress()

        elif (key_v.event_type == 'up'):  # 松开键
            if(key_v.name == self.press_r_ctrl.name):
                self.is_press_ctrl = False
            if(key_v.name == self.press_r_alt):
                self.is_press_alt = False
        self.CheckAngleWid()

    ## @brief 初始化键盘快捷键。为每个方向键、Ctrl+方向键和Alt+方向键添加快捷键，并将其绑定到相应的方法上
    # # -@anchor initkey
    # @param 无
    # @return 无
    def initkey(self):
        keyboard.add_hotkey('left', self.leftPress)
        keyboard.add_hotkey('right', self.rightPress)
        keyboard.add_hotkey('up', self.upPress)
        keyboard.add_hotkey('down', self.downPress)
        keyboard.add_hotkey('ctrl+left', self.ctrlLeftPress)
        keyboard.add_hotkey('ctrl+right', self.ctrlRightPress)
        keyboard.add_hotkey('alt+up', self.altUpPress)
        keyboard.add_hotkey('alt+down', self.altDownPress)
