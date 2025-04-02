from crazyflie import TimeHelper, CrazyflieServer

## @file
#  
#  @anchor crazyswarm接口库文件
class Crazyswarm:
    def __init__(self, crazyflies_yaml=None, parent_parser=None, args=None, num=1, ids=None, mode=7):
        self.allcfs = CrazyflieServer(crazyflies_yaml, num, ids, mode)
        self.timeHelper = TimeHelper()
