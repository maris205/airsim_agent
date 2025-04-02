import PX4MavCtrlV4 as PX4MavCtrl
import time
import numpy as np
import threading
import DllSimCtrlAPI

dll = DllSimCtrlAPI.DllSimCtrlAPI()

# 测试方法
# 1. 打开CopterSim，选择SITL模式，点击运行
# 2. 逐行取消注释，点击运行，观察CopterSim左下角信息打印
# 3. 测完一条后，注释上一条，重复现象

# 普通list输入，或np.array输入都行。
#dll.sendSILIntFloat([1,2,3],np.array([4,5,6,7]))

#dll.sendSILIntDouble([1,2,3],np.array([4,5,6,7]))

#dll.sendInDoubCtrls([1,2,3,4,5,6])

#dll.sendInCtrlExt([1,2,3],[4,5,6,7],3)

#dll.sendInCtrlExtDoub([1,2,3,4,5,6],3)

#dll.sendInCtrlExtAll([1,2,3,4,5,6]*5)

#dll.sendModelInParams(0b11111001110101,[1,2,3,4,5,6]*5)

#dll.sendInitInParams(0b11111001110101,[1,2,3,4,5,6]*5)

#dll.sendDynModiParams(0b11111001110101,[1,2,3,4,5,6]*5)

# dll.sendUE2Coptersim([1,2,3,4,5,6]*5)

# dll.sendFloatsColl(1,[2,3,4],[5,6,7,8,9,0],[11,12,13])

# dll.sendColl20d([1,2,3,4,5,6])

# dll.sendTerrIn15d([1,2,3,4,5,6])