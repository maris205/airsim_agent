import socket
import threading
import time
import struct
import math
import sys
import copy
import os
import cv2
import numpy as np
## @file 
#  这是一个处理地理坐标转换的模块
#  @anchor EarthModel接口库文件
#  对应例程链接见


## @brief EarthModel类目前定义了同一地球基准下的坐标转换
# 
#  该类目前定义了WGS84椭球体基准面（GPS的使用标准）下的坐标系变换。
#  
#  主要涉及三个坐标系，地心地固坐标系（ECEF，是空间直角坐标系中的一种，空间直角坐标系还包括参心系），大地坐标系（LLA）和站心坐标系（ENU或NED）。
#  
#  
# 坐标转换的基本方法如下： 
# @image html Possible_paths_for_datum_transform.svg.png width=600px
# 
# 坐标转换通常包含两个方面，坐标系的变换和基准面的变换：
#
# 
# 坐标系变换
# 
# @image html ENU.png "站心地平直角坐标系 (ENU)" width=900px
# 坐标系变换就是在同一地球椭球下，空间点的不同坐标表示形式进行变换。包括大地坐标系与空间直角坐标系的相互转换、空间直角坐标系与站心坐标系的转换（若考虑投影变换，也就是将地球展平，还有大地坐标系与高斯平面坐标系的转换）。
# 
# 基准变换
# @image html 大地基准面.png "大地基准面和椭球体的关系" width=400px
# 基准变换是指空间点在不同椭球的坐标转换。通常使用空间的七参数（Helmert模型）实现不同椭球间空间直角坐标系或者不同椭球间大地坐标系的转换。
# 
# 七参数公式如下: 
# \f[\left[\begin{array}{l}X \\Y \\Z\end{array}\right]=\left[\begin{array}{l}\Delta X \\\Delta Y \\\Delta Z\end{array}\right]+(1+m)\left[\begin{array}{ccc}1 & \theta_{z} & -\theta_{y} \\-\theta_{z} & 1 & \theta_{x} \\\theta_{y} & -\theta_{x} & 1\end{array}\right]\left[\begin{array}{l}X_{0} \\Y_{0} \\Z_{0}\end{array}\right] \f]
#  式中, ∆X, ∆Y, ∆Z为三个平移参数; θx, θy, θz为3个旋转参数，m为尺度参数。此七个参数可以通过在需要转化的区域里选取3个以上的转换控制点对而获取。
# 
# 坐标转换的一般实施步骤如下：
# 
# 1.收集、整理转换区域内重合点成果：
#
# 重合点是指在两个坐标系下都有明确坐标的点。
# 需要收集足够数量的重合点数据，以确保转换精度。
# 
# 2.分析、选取用于计算坐标转换参数的重合点：
#
# 检查重合点的分布和质量，确保它们覆盖转换区域并且数据准确。
# 根据实际需要，选取合适的重合点用于后续的参数计算。
# 
# 3.确定坐标转换参数计算方法与坐标转换模型：
#
# 常见的转换模型有平面四参数模型和布尔莎七参数模型。
# 根据具体需求和重合点分布，选择适合的转换模型。
# 
# 4.两坐标系下重合点坐标形式的转换：
#
# 若采用平面四参数转换模型，需要将重合点的两坐标系坐标转换成同一投影带的高斯平面坐标。至少需要两对重合点坐标。
# 
# 若采用布尔莎七参数转换模型，则要将重合点的两坐标系坐标转换成各自坐标系下的空间直角坐标。至少需要三对重合点坐标。
# 
# 5.根据确定的转换方法与转换模型利用最小二乘法初步计算坐标转换参数：
#
# 使用最小二乘法对选取的重合点数据进行计算，得到初步的转换参数。
# 
# 6.分析重合点坐标转换残差，根据转换残差剔除粗差点：
#
# 计算转换后的重合点残差，即实际坐标与转换后的坐标之间的差异。
# 剔除残差过大的点（粗差点），以确保转换模型的准确性。
# 
# 7.坐标转换残差满足精度要求时，计算最终的坐标转换参数并估计坐标转换精度：
#
# 当残差在可接受范围内时，进行最终参数计算。
# 对转换精度进行评估，确保满足应用需求。
# 
# 8.根据计算得到的转换参数，转换得到转换点的目标坐标系坐标：
#
# 使用最终确定的转换参数，对所有待转换点进行坐标转换，得到在目标坐标系下的坐标。

class EarthModel():
    
    ## @brief EarthModel类的构造函数
    #  
    #  在实例化类时会自动调用。它初始化了地球椭球体参数。
    def __init__(self):
        ## @var wgs84_a
        #  定义地球模型的长半轴（Equatorial Radius），
        #  
        #  即地球在赤道上的半径，单位为米。值为 6378137 米。
        
        ## @var wgs84_b 
        #  定义地球模型的短半轴（Polar Radius），
        #  
        #  即地球在极点处的半径，单位为米。值为 6356752.3142 米。
        
        ## @var wgs84_f
        #  计算并定义地球模型的扁率（Flattening）。
        #  
        #  扁率表示地球从球形被压扁成椭球形的程度。
        #  
        #  计算公式为：f = (a - b) / a
        #  其中 a 是长半轴，b 是短半轴。
        #  对应的值计算为：(6378137 - 6356752.3142) / 6378137。
        
        ## @var pow_e_2
        #  计算并定义地球模型的第一偏心率的平方（Square of the First Eccentricity）。
        #  
        #  偏心率是描述椭圆形状的重要参数，表示椭圆的离心程度。
        #  
        #  计算公式为：e^2 = f * (2 - f)
        #  其中 f 是扁率。
        #  对应的值计算为：self.wgs84_f * (2 - self.wgs84_f)。
        
        self.wgs84_a = 6378137
        self.wgs84_b = 6356752.3142
        self.wgs84_f = (self.wgs84_a - self.wgs84_b) / self.wgs84_a
        self.pow_e_2 = self.wgs84_f * (2-self.wgs84_f)
 
    ## @brief LLA坐标系转ECEF坐标系
    #  
    #  LLA坐标系下的(lon,lat,alt)转换为ECEF坐标系下点(X,Y,Z)
    #  
    #  \f(\left\{\begin{array}{l}X=(N+\text { alt }) \cos (\text { lat }) \cos (\text { lon }) \\Y=(N+\text { alt }) \cos (\text { lat }) \sin (\text { lon }) \\Z=\left(N\left(1-e^{2}\right)+\text { alt }\right) \sin (\text { lat })\end{array}\right. \f)
    #
    #  其中e为椭球第一偏心率，N为基准椭球面的曲率半径
    #   
    #   \f(\left\{\begin{array}{l}e^{2}=\frac{a^{2}-b^{2}}{a^{2}} \\N=\frac{a}{\sqrt{1-e^{2} \sin ^{2} \text { lat }}}\end{array}\right. \f)
    #  
    #  @image html Geodetic_latitude_and_the_length_of_Normal.svg.png width=300px  
    #  
    #  地理坐标系和地心地固坐标系共用了同一套坐标轴。上图中，椭圆表示了地球的横截面，纵轴为Z轴，横轴为X-Y平面。假设地球的赤道半径为a，极轴半径为b。
    # 
    # 地表上空R点的地理坐标为：纬度ϕ，经度λ，海拔h。P点为发现RQ与椭球面的交点。 \f( \mathrm{PQ} = N(\phi),  \mathrm{IQ} = e^{2} N(\phi) ,\mathrm{R} = (X, Y, Z) \f)
    #  
    #  @param lat 
    #  @param lon
    #  @param h
    #  @return x
    #  @return y
    #  @return z
    #  @note 这里经纬度单位为度，高度单位为米
    def lla2ecef(self, lat, lon, h):
        # (lat, lon) in degrees
        # h in meters
        lamb = math.radians(lat)
        phi = math.radians(lon)
        s = math.sin(lamb)
        N = self.wgs84_a / math.sqrt(1 - self.pow_e_2 * s * s)
    
        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
    
        x = (h + N) * cos_lambda * cos_phi
        y = (h + N) * cos_lambda * sin_phi
        z = (h + (1 - self.pow_e_2) * N) * sin_lambda
    
        return x, y, z
    

 
    ## @brief ECEF坐标系转ENU坐标系
    #
    #  由于ENU坐标系定义在局部坐标系下，所以转换前需要知道ENU局部坐标系的原点的ECEF坐标(Xr,Yr,Zr)，以及原点的经度ϕ和纬度λ。
    #  
    # \f(用户所在坐标点  P_{0}=\left(x_{0}, y_{0}, z_{0}\right)  ， 计算点  P=(x, y, z)  在以点  P_{0}  为坐标原点的enu坐标系位置  (e, n, u)  这里需要用到LLA坐标系的数据，  P_{0}  的LLA坐标点为  L L A_{0}=\left(l o n_{0}, l a t_{0}, a l t_{0}\right) ,计算式为：
    #  
    #  \\\begin{array}{l}{\left[\begin{array}{l}\Delta x \\\Delta y \\\Delta z\end{array}\right]=\left[\begin{array}{l}x \\y \\z\end{array}\right]-\left[\begin{array}{l}x_{0} \\y_{0} \\z_{0}\end{array}\right]} \\{\left[\begin{array}{l}e \\n \\u\end{array}\right]=S \cdot\left[\begin{array}{c}\Delta x \\\Delta y \\\Delta z\end{array}\right]=\left[\begin{array}{ccc}-\sin \left(\text{lon}_{0}\right) & \cos \left(\text{lon}_{0}\right) & 0 \\-\sin \left(\text{lat}_{0}\right) \cos \left(\text{lon}_{0}\right) & -\sin \left(\text{lat}_{0}\right) \sin \left(\text{lon}_{0}\right) & \cos \left(\text{lat}_{0}\right) \\\cos \left(\text { lat }_{0}\right) \cos \left(\text{lon}_{0}\right) & \cos \left(\text{lat}_{0}\right) \sin \left(\text{lon}_{0}\right) & \sin \left(\text{lat}_{0}\right)\end{array}\right] \cdot\left[\begin{array}{l}\Delta x \\\Delta y \\\Delta z\end{array}\right]} \\\end{array} \f)
    #
    # 
    #  \f( 坐标变换矩阵S = \left[
    # \begin{array}{ccc}
    # -\sin(\text{lon}_{0}) & \cos(\text{lon}_{0}) & 0 \\
    # -\sin(\text{lat}_{0}) \cos(\text{lon}_{0}) & -\sin(\text{lat}_{0}) \sin(\text{lon}_{0}) & \cos(\text{lat}_{0}) \\
    # \cos(\text{lat}_{0}) \cos(\text{lon}_{0}) & \cos(\text{lat}_{0}) \sin(\text{lon}_{0}) & \sin(\text{lat}_{0})
    # \end{array}
    # \right]
    # \f)
    # @param x
    # @param y
    # @param z
    # @param lat0
    # @param lon0
    # @param h0
    # @return xEast
    # @return yNorth
    # @return zUp
    def ecef2enu(self, x, y, z, lat0, lon0, h0):
        lamb = math.radians(lat0)
        phi = math.radians(lon0)
        s = math.sin(lamb)
        N = self.wgs84_a / math.sqrt(1 - self.pow_e_2 * s * s)
    
        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
    
        x0 = (h0 + N) * cos_lambda * cos_phi
        y0 = (h0 + N) * cos_lambda * sin_phi
        z0 = (h0 + (1 - self.pow_e_2) * N) * sin_lambda
    
        xd = x - x0
        yd = y - y0
        zd = z - z0
    
        t = -cos_phi * xd -  sin_phi * yd
    
        xEast = -sin_phi * xd + cos_phi * yd
        yNorth = t * sin_lambda  + cos_lambda * zd
        zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd
    
        return xEast, yNorth, zUp
    ## @brief ENU坐标系转ECEF坐标系
    #  
    #  ENU坐标系下的(xEast,yNorth,zUp)转换为ECEF坐标系下点(X,Y,Z):
    #  \f[\left[\begin{array}{l}X \\Y \\Z\end{array}\right]=\left[\begin{array}{ccc}-\sin \lambda & -\sin \phi \cos \lambda & \cos \phi \cos \lambda \\\cos \lambda & -\sin \phi \sin \lambda & \cos \phi \sin \lambda \\0 & \cos \phi & \sin \phi\end{array}\right
    #  @param xEast
    #  @param yNorth
    #  @param zUp
    #  @param lat0
    #  @param lon0
    #  @param h0
    #  @return x
    #  @return y
    #  @return z
    def enu2ecef(self, xEast, yNorth, zUp, lat0, lon0, h0):
        lamb = math.radians(lat0)
        phi = math.radians(lon0)
        s = math.sin(lamb)
        N = self.wgs84_a / math.sqrt(1 - self.pow_e_2 * s * s)
    
        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
    
        x0 = (h0 + N) * cos_lambda * cos_phi
        y0 = (h0 + N) * cos_lambda * sin_phi
        z0 = (h0 + (1 - self.pow_e_2) * N) * sin_lambda
    
        t = cos_lambda * zUp - sin_lambda * yNorth
    
        zd = sin_lambda * zUp + cos_lambda * yNorth
        xd = cos_phi * t - sin_phi * xEast 
        yd = sin_phi * t + cos_phi * xEast
    
        x = xd + x0 
        y = yd + y0 
        z = zd + z0 
    
        return x, y, z
    
    ## @brief ECEF坐标系转LLA坐标系
    #  
    #  ECEF坐标系下点(X,Y,Z)转换为LLA坐标系下的(lat,lon,alt)
    #  
    # ECEF坐标到经度的换算为：
    #  
    # \f(\lambda=\text{atan} 2(Y, X) \f)
    # 
    # 纬度和高度的转换涉及 N（曲率半径N 是纬度的函数） 的循环关系：
    # 
    # \f(  \frac{Z}{p} \cot \phi=1-\frac{e^{2} N}{N+h}\\
    # h=\frac{p}{\cos \phi}-N 
    # \f)
    #
    # 一个基于Ferrari's solution的迭代求解方法如下：
    #  
    #  \f{align*}{ & r  =  \sqrt{X^{2}+Y^{2}} \\& e^{2}  = \left(a^{2}-b^{2}\right) / b^{2} \\& F =  54 b^{2} Z^{2} \\& G =  r^{2}+\left(1-e^{2}\right) Z^{2}-e^{2}\left(a^{2}-b^{2}\right) \\& c  =  \frac{e^{4} r^{2}}{G^{3}} \\& s  =  \sqrt[3]{1+c+\sqrt{c^{2}+2 c}} \\& P  =  \frac{P}{3\left(s+\frac{1}{3}+1\right)^{2} G^{2}} \\& Q  =  \sqrt{1+2 e^{4} P} \\& r_{0}  =  \frac{-\left(P e^{2} r\right)}{1-Q}+\sqrt{\frac{1}{2} a^{2}(1+1 / Q)-\frac{P\left(1-e^{2}\right) Z^{2}}{Q(1+Q)}-\frac{1}{2} P r^{2}} \\& U  =  \sqrt{\left(r-e^{2} r_{0}\right)^{2}+Z^{2}} \\& V  =  \sqrt{\left(r-e^{2} r_{0}\right)^{2}+\left(1-e^{2}\right) Z^{2}} \\& z_{0}  =  \frac{b^{2} Z}{a V} \\& \text { alt }  =  U\left(1-\frac{b^{2}}{a V}\right) \\& \text { lat }  =  \arctan \left[\frac{Z+e^{2} z_{0}}{r}\right] \\& \text { lon }  =  \arctan \left(\frac{y}{x}\right) \f}
    #  
    #  @param x
    #  @param y
    #  @param z
    #  @return lat0
    #  @return lon0
    #  @return h0
    def ecef2lla(self, x, y, z):
    # Convert from ECEF cartesian coordinates to 
    # latitude, longitude and height.  WGS-84
        x2 = x ** 2 
        y2 = y ** 2 
        z2 = z ** 2 
    
        self.wgs84_a = 6378137   # earth radius in meters
        self.wgs84_b = 6356752.3142    # earth semiminor in meters 
        e = math.sqrt (1-(self.wgs84_b/self.wgs84_a)**2) 
        b2 = self.wgs84_b*self.wgs84_b 
        e2 = e ** 2 
        ep = e*(self.wgs84_a/self.wgs84_b) 
        r = math.sqrt(x2+y2) 
        r2 = r*r 
        E2 = self.wgs84_a ** 2 - self.wgs84_b ** 2 
        F = 54*b2*z2 
        G = r2 + (1-e2)*z2 - e2*E2 
        c = (e2*e2*F*r2)/(G*G*G) 
        s = ( 1 + c + math.sqrt(c*c + 2*c) )**(1/3) 
        P = F / (3 * (s+1/s+1)**2 * G*G) 
        Q = math.sqrt(1+2*e2*e2*P) 
        ro = -(P*e2*r)/(1+Q) + math.sqrt((self.wgs84_a*self.wgs84_a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2) 
        tmp = (r - e2*ro) ** 2 
        U = math.sqrt( tmp + z2 ) 
        V = math.sqrt( tmp + (1-e2)*z2 ) 
        zo = (b2*z)/(self.wgs84_a*V) 
    
        height = U*( 1 - b2/(self.wgs84_a*V) ) 
        
        lat = math.atan( (z + ep*ep*zo)/r ) 
    
        temp = math.atan(y/x) 
        if x >=0 :    
            long = temp 
        elif (x < 0) & (y >= 0):
            long = math.pi + temp 
        else :
            long = temp - math.pi 
    
        lat0 = lat/(math.pi/180) 
        lon0 = long/(math.pi/180) 
        h0 = height 
    
        return lat0, lon0, h0
    
    ## @brief LLA到ENU之间的坐标系转换通过先转换为ECEF实现。
    #  
    #  @param lat
    #  @param lon
    #  @param h
    #  @param lat_ref
    #  @param lon_ref
    #  @param h_ref
    #  @return xEast
    #  @return yNorth
    #  @return zUp
    def lla2enu(self, lat, lon, h, lat_ref, lon_ref, h_ref):
    
        x, y, z = self.lla2ecef(lat, lon, h)
        
        return self.ecef2enu(x, y, z, lat_ref, lon_ref, h_ref)
    
    ## @brief ENU到LLA之间的坐标系转换也通过先转换为ECEF实现。
    #  
    #  @param xEast
    #  @param yNorth
    #  @param zUp
    #  @param lat_ref
    #  @param lon_ref
    #  @param h_ref
    #  @return lat0
    #  @return lon0
    #  @return h0
    def enu2lla(self, xEast, yNorth, zUp, lat_ref, lon_ref, h_ref):
    
        x,y,z = self.enu2ecef(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref)
    
        return self.ecef2lla(x,y,z)
    ## @brief LLA到NED之间的坐标系转换通过先转换为ENU实现。
    #  
    #  @param lla
    #  @param lla0
    #  @return yNorth
    #  @return xEast
    #  @return zUp
    def lla2ned(self, lla, lla0):
        lat=lla[0]
        lon=lla[1]
        h=lla[2]
        lat_ref=lla0[0]
        lon_ref=lla0[1]
        h_ref=lla0[2]
        xEast, yNorth, zUp=self.lla2enu(lat, lon, h, lat_ref, lon_ref, h_ref)
        return [yNorth,xEast,-zUp]
    
    ## @brief NED到LLA之间的坐标系转换通过先转换为ENU实现。
    #  
    #  @param ned
    #  @param lla0
    #  @return lat0
    #  @return lon0
    #  @return h0
    def ned2lla(self,ned,lla0):
        xEast=ned[1]
        yNorth=ned[0]
        zUp=-ned[2]
        lat_ref=lla0[0]
        lon_ref=lla0[1]
        h_ref=lla0[2]
        return self.enu2lla(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref)

class Coordinate:
    """
    参考MAV_FRAME
    """
    LOCAL_NED = 1
    GLOBAL_INT = 5
    NED_BODY = 8  # 当发送Offboard控制指令时，位置在NED坐标系，速度加速度在BODY坐标系下
