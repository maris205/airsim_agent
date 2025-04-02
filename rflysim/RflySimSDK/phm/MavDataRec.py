import threading
## @file 
# 该文件定义了MavDataRec类，用于接收和记录无人机相关的数据。MavDataRec类提供了一个线程安全的方式来监听和存储无人机发送的特定类型消息。
# @anchor MavDataRec接口库文件

## @class MavDataRec
# @brief 用于接收和记录无人机数据的类。
class MavDataRec:
    # constructor function
    ## @brief 构造函数，初始化MavDataRec类的实例。
    # - @anchor __init__
    # @param mav 无人机对象，用于接收无人机发送的消息。
    def __init__(self, mav):
        self.mav = mav
        # 在这里添加列表变量

        self.msgDic={}
        self.NameList=['']
        self.LenList=0

    # - @anchor startRecMsg
    ## @brief 开始监听消息的函数。
    # @param NameList 要监听的消息名称列表。
    # @param LenList 每个消息名称对应的消息数量限制列表。
    # 开始监听消息的函数
    def startRecMsg(self,NameList=['HIGHRES_IMU'],LenList=[10]):
        if len(NameList) != len(LenList):
            disp('Error: the input length not match!')
            return

        self.NameList = NameList
        self.LenList = LenList

        for i in range(len(self.NameList)):
            self.msgDic[self.NameList[i]]=[] # 初始化为列表

        # 创建线程并开始监听
        self.t1 = threading.Thread(target=self.getMavMsg, args=())
        self.t1Stop=False # 停止标志位
        self.t1.start()


    # - @anchor stopRecMsg
    ## @brief 停止监听消息的函数。
    # 开始监听消息的函数
    def stopRecMsg(self):
        self.t1Stop=True # 停止标志位
        self.t1.join()        

    # - @anchor getMavMsg
    ## @brief 在单独的线程中运行，用于获取无人机消息。
    def getMavMsg(self):
        while(True):
            if self.t1Stop: # 如果停止标志位启用，就跳出循环
                break

            # 阻塞，直到受到Mavlink消息
            self.mav.uavEvent.wait()
            msg = self.mav.uavMsg
            # 重新使能阻塞状态
            self.mav.uavEvent.clear()

            for i in range(len(self.NameList)):
                if msg.get_type() == self.NameList[i]:
                    self.msgDic[self.NameList[i]].append(msg) # 存入一个数据
                    if len(self.msgDic[self.NameList[i]])>=self.LenList[i]+1:
                        del self.msgDic[self.NameList[i]][0] # 如果大于指定数量，就删除最前面的一个


