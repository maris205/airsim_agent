import time
import math
import sys
import PX4MavCtrlV4 as PX4MavCtrl
import UE4CtrlAPI

ue = UE4CtrlAPI.UE4CtrlAPI()

#Create a new MAVLink communication instance, UDP sending port (CopterSim’s receving port) is 20100
mav = PX4MavCtrl.PX4MavCtrler(1)


# sendUE4Cmd: RflySim3D API to modify scene display style
# Format: ue.sendUE4Cmd(cmd,windowID=-1), where cmd is a command string, windowID is the received window number (assuming multiple RflySim3D windows are opened at the same time), windowID =-1 means sent to all windows
# Augument: RflyChangeMapbyName command means to switch the map (scene), the following string is the map name, here will switch all open windows to the grass map
ue.sendUE4Cmd('RflyChangeMapbyName Grasslands')    
time.sleep(2)  

# sendUE4Pos: RflySim3D API to generate 3D objects and control position
# Formart: ue.sendUE4Pos(CopterID, VehicleType, RotorSpeed, PosM, AngEulerRad, windowsID=0)
ue.sendUE4Pos(100,30,0,[2.5,0,-8.086],[0,0,math.pi])
# Send and generate a 3D object to RflySim3D, where: the vehicle ID is CopterID=100;
# Vehicle type VehicleType=30 (a man); RotorSpeed=0RPM; Position coordinate PosM=[2.5,0,-8.086]m
# Vehicle attitude angle AngEulerRad=[0,0,math.pi]rad (rotate 180 degrees to face the vehicle), the receiving window number default windowsID=-1 (sent to all open RflySim3D programs)
# VehicleType options: 3 for quadcopters, 5/6 for hexacopters, 30 for persons, 40 for checkerboard grids, 50/51 for cars, 60 for luminous lights, 100 for flying-wing or fixed-wing aircraft, 150/152 for circular square targets
time.sleep(0.5)

ue.sendUE4PosScale(101,30,0,[10.5,0,-8.086],[0,0,math.pi],[10,10,10])
# Send and generate a 3D object to RflySim3D, where: the vehicle ID is CopterID=101;
# Vehicle type VehicleType=2030 means Type=30 (a man) with model = 2; RotorSpeed=0RPM; Position coordinate PosM=[10.5,0,-8.086]m, scale=[10,10,10] times in xyz direction
time.sleep(0.5)

# RflyChange3DModel command followed by vehicle ID + desired style
ue.sendUE4Cmd('RflyChange3DModel 100 12') 
#Send a message to make CopterID=100 (the character just created) in all scenes, here style=12 represents a walking person
time.sleep(2)


# RflyChange3DModel command followed by vehicle ID + desired style
print('Change to Eric_Walking')
ue.sendUE4Cmd('RflyChange3DModel 100 Eric_Walking') 
#Send a message to make CopterID=100 (the character just created) in all scenes, here style=12 represents a walking person
time.sleep(2)


# Command RflyChangeViewKeyCmd means to simulate the shortcut key pressed in RflySim3D, shortcut key B 1 means to switch the focus to the object with CopterID=1
# Here is set to send to window 0, other windows do not send
ue.sendUE4Cmd('RflyChangeViewKeyCmd B 1',0)
time.sleep(0.5)  


# Shortcut key V 1 means to switch to the 1st onboard camera (front camera)
ue.sendUE4Cmd('RflyChangeViewKeyCmd V 1',0)
time.sleep(0.5)  


# RflyCameraPosAng x y z roll pith yaw 
# Set the position of the camera relative to the center of the body, the default is 0
# Here set the position of the front camera to [0.1 -0.25 0]
ue.sendUE4Cmd('RflyCameraPosAng 0.1 0 0',0)
time.sleep(0.5)


# r.setres 720x405w is a built-in command of UE4, which means to switch the resolution to 720x405
ue.sendUE4Cmd('r.setres 720x405w',0)
time.sleep(0.5)


# Send a shortcut command to window 1 to switch the focus to vehilce 1
ue.sendUE4Cmd('RflyChangeViewKeyCmd B 1',1)
time.sleep(0.5)  


# Send a shortcut key control command to window 0, N 1 shortcut key means to switch the perspective to the ground fixed perspective 1
ue.sendUE4Cmd('RflyChangeViewKeyCmd N 1',1)
time.sleep(0.5)  

# Set the current camera Field of View (FOV) to 90 degrees (the default value is 90 degrees in RflySim3D), the range of FOV is 0 to 180 degrees
ue.sendUE4Cmd('RflyCameraFovDegrees 90',1)
time.sleep(0.5) 

# Set the current camera position here as [-2 0 -9.7]
ue.sendUE4Cmd('RflyCameraPosAng -2 0 -9.7',1)
time.sleep(0.5)


#Turn on MAVLink to monitor CopterSim data and update it in real time. 
mav.InitMavLoop()
time.sleep(0.5)


#Display Position information received from CopterSim
print(mav.uavPosNED)


#Turn on Offboard mode
mav.initOffboard()
# Send the desired position signal, fly to the target point 0,0, -1.7 position, the yaw angle is 0
mav.SendPosNED(0, 0, -1.7, 0) 
print("Send target Pos")
time.sleep(0.5)
#Send arm command to arm the drone
mav.SendMavArm(True) 
print("Send Arm Command")

time.sleep(10)

# Send the desired speed signal, 0.2m/s downwards, the z-axis downward is positive
mav.SendVelNED(0, 0, 0.2, 0) 
print("Send Velocity Speed")

time.sleep(10)

#Exit Offboard control mode
print("Send offboard stop")
mav.endOffboard()
time.sleep(1)

#Exit MAVLink data receiving mode
print("Send Mavlink stop")
mav.stopRun()
time.sleep(1)
#while True:
#    print(mav.uavPosNED)
#    time.sleep(2)
