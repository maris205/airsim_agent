if not defined PSP_PATH (
    SET PSP_PATH=d:\PX4PSP
    SET PSP_PATH_LINUX=/mnt/d/PX4PSP
)
start cmd.exe /k "echo Python3.8 environment has been set with openCV+pymavlink+numpy+pyulog etc. && echo You can use pip or pip3 command to install other libraries && echo Put Python38Run.bat into your code folder && echo Use the command: 'python XXX.py' to run the script with Python && SET PATH=%PSP_PATH%\Python38;%PSP_PATH%\Python38\Scripts;%CD%;%PATH%"
