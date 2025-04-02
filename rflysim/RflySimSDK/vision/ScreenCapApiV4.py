# import all libraries
import win32gui, win32con,win32ui
from ctypes import windll
import numpy
import d3dshot
import sys
import cv2
## @file
#  这是用于捕获和处理由 Unreal Engine 4.23 及以上版本生成的 RflySim3D 窗口的屏幕截图，并转换成 OpenCV 图像格式的模块。
#  @anchor ScreenCapApiV4接口库文件
#  对应例程链接见

# Enable new Screen capture API for UE4 4.23 and above
isNewUE=True

if isNewUE:
    # Use d3dshot to capture screen from UE 4.23+
    windll.shcore.SetProcessDpiAwareness(2)
    d = d3dshot.create(capture_output="numpy",frame_buffer_size=1)
    #d1 = d3dshot.create(capture_output="numpy",frame_buffer_size=1)

# get all RflySim3D window handles with class name UnrealWindow
##  @brief 用于获取所有类名为 "UnrealWindow" 的窗口句柄，并将其添加到 window_hwnds 列表中
def window_enumeration_handler(hwnd, window_hwnds):
    if win32gui.GetClassName(hwnd) == "UnrealWindow":
        window_hwnds.append(hwnd)

# Get handles of all RflySim3D windows
##  @brief 获取所有 RflySim3D 窗口的句柄并按顺序排列。返回这些窗口的句柄列表
def getWndHandls():
    window_hwnds = []
    win32gui.EnumWindows(window_enumeration_handler, window_hwnds)
    window_index=[]
    for hw in window_hwnds:
        name=win32gui.GetWindowText(hw)
        nameList = name.split('-',1)
        windIndex=nameList[1]
        if windIndex.isdigit():
            inx=int(windIndex)
            window_index.append(inx)

    # Windows are recognized by RflySim3D-*, where * is the number
    for j in range(len(window_index)):
        for i in range(len(window_index)-1-j):
            if window_index[i]>window_index[i+1]:
                window_index[i], window_index[i+1] = window_index[i+1], window_index[i]
                window_hwnds[i], window_hwnds[i+1] = window_hwnds[i+1], window_hwnds[i]
        
    print(window_hwnds)
    print(window_index)
    return window_hwnds

# define a class to store information of window handles
## @class WinInfo
#  @brief WinInfo结构体类。存储窗口句柄及其相关信息的类，包括窗口的宽度、高度、设备上下文 (DC) 等
class WinInfo:
    def __init__(self, hWnd, width, height, saveDC, saveBitMap, mfcDC, hWndDC):
        self.hWnd = hWnd
        self.width = width
        self.height = height
        self.saveDC = saveDC
        self.saveBitMap = saveBitMap
        self.mfcDC = mfcDC
        self.hWndDC = hWndDC

# get the window infomation of a handle
##  @brief 获取指定窗口句柄的详细信息，并返回一个 WinInfo 对象
# # - @anchor getHwndInfo
##  @param hWnd 窗口句柄
#  @return 包含窗口句柄，窗宽、窗高信息的WinInfo对象
def getHwndInfo(hWnd):
    left, top, right, bot = win32gui.GetClientRect(hWnd)
    width = right - left
    height = bot - top
    print((width,height))
    if hWnd and width == 0 and height == 0:
        print("The RflySim3D window cannot be in minimized mode")
        sys.exit(1)

    
    if not isNewUE: # Use Windows API to capture screen for UE4.22 windows
        # retrieve the device context (DC) for the entire window, 
        # including title bar, menus, and scroll bars.
        hWndDC = win32gui.GetWindowDC(hWnd)
        mfcDC = win32ui.CreateDCFromHandle(hWndDC)
        
        # creates a memory device context (DC) compatible with the specified device.
        saveDC = mfcDC.CreateCompatibleDC()
        
        # create a bitmap object
        saveBitMap = win32ui.CreateBitmap()
        saveBitMap.CreateCompatibleBitmap(mfcDC, width, height)

        # return image info.
        info = WinInfo(hWnd, width, height, saveDC, saveBitMap, mfcDC, hWndDC)
        return info
    else:
        # Use Windows Desktop Duplication API to capture screen for all UE4 windows
        info = WinInfo(hWnd, width, height, 0, 0, 0, 0)
        return info        


# get the image from RflySim3D window's client area
##  @brief 从 RflySim3D 窗口的客户区获取图像，并返回 OpenCV 格式的图像
## - @anchor getCVImg
##  @param wInfo 窗口信息对象
#  @return OpenCV 格式的图像
def getCVImg(wInfo):
    if not isNewUE:
        wInfo.saveDC.SelectObject(wInfo.saveBitMap)

        #  copies a visual window into the specified device context (DC)
        # The last int value：0-save the whole window，1-only client area
        result = windll.user32.PrintWindow(wInfo.hWnd, wInfo.saveDC.GetSafeHdc(), 1)
        signedIntsArray = wInfo.saveBitMap.GetBitmapBits(True)

        # get the image from bitmap array
        im_opencv = numpy.frombuffer(signedIntsArray, dtype='uint8')
        #print(im_opencv.shape)
        im_opencv.shape = (wInfo.height, wInfo.width, 4)
        im_opencv = cv2.cvtColor(im_opencv, cv2.COLOR_BGRA2BGR)
        # return the image
        return im_opencv
    
    else:
        if not win32gui.IsWindow(wInfo.hWnd):
            sys.exit() # exit if RflySim3D is closed
            
        # Get the top left position of the window
        (x, y)=win32gui.ClientToScreen(wInfo.hWnd,(0,0))
        # Get a screnshot
        #frame_stack=d.screenshot(region=(x, y, wInfo.width+x, wInfo.height+y))
        
        frame_stack=d.screenshot()
        frame_stack=frame_stack[y:wInfo.height+y,x:wInfo.width+x]   

        # covert color for opencv
        frame_stack=cv2.cvtColor(frame_stack, cv2.COLOR_RGB2BGR)
        # get scale_factor
        sc = d.display.scale_factor
        w1=int(wInfo.width/sc+0.5)
        h1=int(wInfo.height/sc+0.5)
        # resize the windows
        frame_stack = cv2.resize(frame_stack, (w1, h1))
        return frame_stack

##  @brief 接收一个窗口信息列表 wInfoList，然后根据是否是新版本的UE4，
# - @anchor getCVImgList
##  @param wInfoList 窗口信息列表
#  @return 包含每个窗口图像的列表
def getCVImgList(wInfoList):
    outList=[]
    if not isNewUE:
        for wInfo in wInfoList:
            wInfo.saveDC.SelectObject(wInfo.saveBitMap)
            #  copies a visual window into the specified device context (DC)
            # The last int value：0-save the whole window，1-only client area
            result = windll.user32.PrintWindow(wInfo.hWnd, wInfo.saveDC.GetSafeHdc(), 1)
            signedIntsArray = wInfo.saveBitMap.GetBitmapBits(True)

            # get the image from bitmap array
            im_opencv = numpy.frombuffer(signedIntsArray, dtype='uint8')
            #print(im_opencv.shape)
            im_opencv.shape = (wInfo.height, wInfo.width, 4)
            im_opencv = cv2.cvtColor(im_opencv, cv2.COLOR_BGRA2BGR)
            outList.append(im_opencv)
    else:
        screenC = d.screenshot()
        for wInfo in wInfoList:
            if not win32gui.IsWindow(wInfo.hWnd):
                sys.exit() # exit if RflySim3D is closed
                
            # Get the top left position of the window
            (x, y)=win32gui.ClientToScreen(wInfo.hWnd,(0,0))
            frame_stack=screenC[y:wInfo.height+y,x:wInfo.width+x]   
            # covert color for opencv
            frame_stack=cv2.cvtColor(frame_stack, cv2.COLOR_RGB2BGR)
            # get scale_factor
            sc = d.display.scale_factor
            w1=int(wInfo.width/sc+0.5)
            h1=int(wInfo.height/sc+0.5)
            # resize the windows
            frame_stack = cv2.resize(frame_stack, (w1, h1))
            outList.append(frame_stack)
    return outList

# move window to desired position and set if always topmost
##  @brief 用于移动窗口到指定位置并设置是否始终置顶
# - @anchor moveWd
##  @param hwd 窗口句柄
##  @param x（默认值为 0） 窗口左上角x坐标
##  @param y（默认值为 0） 窗口左上角y坐标
##  @param topMost（默认值为 False） 是否始终置顶
#  @return 窗口右下角坐标
def moveWd(hwd,x=0,y=0,topMost=False):
    left, top, right, bot = win32gui.GetWindowRect(hwd)
    width = right - left
    height = bot - top
    DispTop = win32con.HWND_TOP
    if topMost and isNewUE:
        DispTop=win32con.HWND_TOPMOST
        win32gui.SetWindowPos(hwd, DispTop, x,y,width,height, win32con.SWP_SHOWWINDOW|win32con.SWP_NOSIZE)
    win32gui.SetWindowPos(hwd, DispTop, x,y,width,height, win32con.SWP_SHOWWINDOW|win32con.SWP_NOSIZE)

    return (x+width,y+height)
    
    
# clear all objects
##  @brief 用于清理和释放窗口对象占用的资源
# - @anchor clearHWND
##  @param wInfo 窗口信息对象
#  @return 无
def clearHWND(wInfo):
    if not isNewUE:
        win32gui.DeleteObject(wInfo.saveBitMap.GetHandle())
        wInfo.saveDC.DeleteDC()
        wInfo.mfcDC.DeleteDC()
        win32gui.ReleaseDC(wInfo.hWnd, wInfo.hWndDC)
    