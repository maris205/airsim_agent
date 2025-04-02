#!/bin/python3
import numpy as np
import sys
try:
    import open3d
except:
    print('Failed to Load open3d module')
    print('Please run the following command:')
    print('pip install open3d==0.10.0')
    sys.exit(0)
## @file
#  这是一个主要用来显示点云图像的模块。
#  @anchor Open3DShow接口库文件
#  对应例程链接见

## @class Open3DShow
#  @brief Open3DShow结构体类.
class Open3DShow:
    """This is the API class for python to request image from UE4"""

    ## @brief 在对象销毁时调用。它确保在对象被销毁时关闭显示窗口，释放资源
    # # - @anchor __del__
    # @param 无
    # @return 无
    def __del__(self):
        self.CloseShow()

    ## @brief 这是Open3DShow的构造函数初始化了一个 Open3D 的 Visualizer 对象用于显示点云
    def __init__(self):
        #点云显示
        self.visualizer = open3d.visualization.Visualizer()
        
        # 构建 Open3D 中提供的 gemoetry 点云类
        self.pcd = open3d.geometry.PointCloud()  
        self.pcdTmp = open3d.geometry.PointCloud() 
        
    ## @brief 创建一个 Open3D 的窗口，用于显示点云
    # # - @anchor CreatShow
    # @param idx（默认值为 0） 窗口号
    # @return 无
    def CreatShow(self,idx=0):
        # 创建显示创建
        self.visualizer.create_window(window_name="PointCloud_"+str(idx), width=1280, height=720) 
        self.visualizer.get_render_option().background_color = np.asarray([0.8, 0.8, 0.8])
  
    
    ## @brief 更新并显示新的点云数据
    # # - @anchor UpdateShow
    # param Cloud 点云数据
    # @return 无
    def UpdateShow(self,Cloud):
        self.visualizer.remove_geometry(self.pcd)
        
        # 添加点云到可视化
        p3d = np.array(Cloud)[:, :3] # 从4D*n向量提取3D*n点云。
        self.pcd.points = open3d.utility.Vector3dVector(p3d)
        self.visualizer.add_geometry(self.pcd)
        

        # # 设置观察视角
        # self.visualizer.get_view_control().set_front([1, 0, -1])
        # self.visualizer.get_view_control().set_lookat([-2, 0, 5])
        # self.visualizer.get_view_control().set_up([1, 0, 1])     
        
        # self.visualizer.get_view_control().set_front([1, 0, 0])
        # self.visualizer.get_view_control().set_lookat([0, 0, 0])
        # self.visualizer.get_view_control().set_up([0, 0, 1])     
        
        # 更新渲染
        self.visualizer.update_geometry(self.pcd)
        self.visualizer.poll_events()
        self.visualizer.update_renderer()

    ## @brief  更新点云显示，使用 pcdTmp 中的数据替换 pcd 中的数据
    # # - @anchor UpdatePCD
    # @param 无
    # @return 无
    def UpdatePCD(self):
        self.visualizer.remove_geometry(self.pcd)
        
        # 添加点云到可视化
        self.pcd.points = self.pcdTmp.points
        self.pcd.colors = self.pcdTmp.colors
        
        self.visualizer.add_geometry(self.pcd)
        
        # 更新渲染
        self.visualizer.update_geometry(self.pcd)
        self.visualizer.poll_events()
        self.visualizer.update_renderer()


    ## @brief 关闭显示窗口，释放资源
    # # - @anchor CloseShow
    # @param 无
    # @return 无
    def CloseShow(self):
        self.visualizer.destroy_window()