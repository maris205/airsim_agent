#!/usr/bin/env python

import numpy as np
import time
import threading
from ctrl import Ctrl
## @file
#  
#  @anchor crazyflie接口库文件

## @class TimeHelper
#  @brief 提供与时间相关的功能。
#         该类的存在是为了在真实硬件和仿真环境（可能比实时快或慢）下使用相同的脚本逻辑。
#         在真实硬件上运行时，该类可使用ROS的时间函数；在仿真环境中不依赖ROS。
class TimeHelper:
    """Object containing all time-related functionality.

    This class mainly exists to support both real hardware and (potentially
    faster or slower than realtime) simulation with the same script.
    When running on real hardware, this class uses ROS time functions.
    The simulation equivalent does not depend on ROS.

    Attributes:
        visualizer: No-op object conforming to the Visualizer API used in
            simulation scripts. Maintains the property that scripts should not
            know/care if they are running in simulation or not.
    """

    ## @brief 构造函数，初始化TimeHelper类对象的相关属性。
    #  - @anchor __init__
    def __init__(self):
        self.rosRate = None
        self.rateHz = None
        self.visualizer = None

    ## @brief 获取当前系统时间（单位：秒）。
    #  - @anchor time
    #  @return float: 返回当前系统时间（秒）。
    def time(self):
        """Returns the current time in seconds."""
        return time.time()

    ## @brief 使当前线程休眠指定时长（秒）。
    #  - @anchor sleep
    #  @param duration (float): 需要休眠的时间，单位为秒。
    def sleep(self, duration):
        """Sleeps for the provided duration in seconds."""
        time.sleep(duration)

    ## @brief 根据设定的频率使循环以指定的速率执行。
    #  - @anchor sleepForRate
    #  @param rateHz (float): 期望的循环频率（Hz）。
    def sleepForRate(self, rateHz):
        """Sleeps so that, if called in a loop, executes at specified rate."""
        pass

    ## @brief 检查脚本是否应终止（例如用户Ctrl-C）。
    #  - @anchor isShutdown
    #  @return bool: True表示应终止脚本执行，False表示继续执行。
    def isShutdown(self):
        """Returns true if the script should abort, e.g. from Ctrl-C."""
        pass

## @class Crazyflie
#  @brief 表示单个机器人对象的类。
#         本类封装了控制单个Crazyflie飞行器的主要功能，包括起飞、降落、移动和悬停等操作。
class Crazyflie:
    """Object representing a single robot.

    The bulk of the module's functionality is contained in this class.
    """

    ## @brief 构造函数，初始化Crazyflie对象的相关参数和控制接口。
    #  - @anchor __init__
    #  @param id (int): 机器人ID，范围[0, 255]。该ID用作无线电地址的最后一个字节。
    #  @param initialPosition (iterable of float): 初始位置[x, y, z]，通常在实验空间的地面上(z=0)。
    #  @param tf (tf.TransformListener): ROS的TransformListener对象，用于查询机器人的当前状态。
    #  @param mode (int): 工作模式选择参数，影响通信方式（redis或udp）。
    def __init__(self, id, initialPosition, tf, mode):
        """Constructor.

        Args:
            id (int): Integer ID in range [0, 255]. The last byte of the robot's
                radio address.
            initialPosition (iterable of float): Initial position of the robot:
                [x, y, z]. Typically on the floor of the experiment space with
                z == 0.0.
            tf (tf.TransformListener): ROS TransformListener used to query the
                robot's current state.
        """
        self.id = id
        self.mode = mode
        self.prefix = "/cf" + str(id)
        self.initialPosition = np.array(initialPosition)
        self.tf = tf
        if mode > 5:
            com = 'redis'
        else:
            com = 'udp'
        self.ctrl = Ctrl(copter_id=self.id, com=com, simulink_dll=True)
        task = threading.Thread(target=self.loop)
        task.start()
        self.mapini = -8.04
        self.ini_success = False

    ## @brief 循环线程函数，用于初始化控制循环环境。
    #  - @anchor loop    
    #  当offboard模式启用时，此函数在后台线程中执行初始设置。
    def loop(self):
        self.ctrl.init_loop(self.mode, offboard=True)
        self.ini_success = True
        
    ## @brief 设置此机器人的组掩码位，用于同步执行多机编排动作。
    #  - @anchor setGroupMask
    #  @param groupMask (int): 8位整数，每一位表示此机器人是否属于对应组。
    def setGroupMask(self, groupMask):
        """Sets the group mask bits for this robot.

        The purpose of groups is to make it possible to trigger an action
        (for example, executing a previously-uploaded trajectory) on a subset
        of all robots without needing to send more than one radio packet.
        This is important to achieve tight, synchronized "choreography".

        Up to 8 groups may exist, corresponding to bits in the groupMask byte.
        When a broadcast command is triggered on the :obj:`CrazyflieServer` object
        with a groupMask argument, the command only affects those robots whose
        groupMask has a nonzero bitwise-AND with the command's groupMask.
        A command with a groupMask of zero applies to all robots regardless of
        group membership.

        Some individual robot (not broadcast) commands also support groupMask,
        but it is not especially useful in that case.

        Args:
            groupMask (int): An 8-bit integer representing this robot's
                membership status in each of the <= 8 possible groups.
        """
        pass

    ## @brief 启用机载避碰功能，使用缓冲Voronoi单元方法避免与其他无人机碰撞。
    #  - @anchor enableCollisionAvoidance
    #  @param others (List[Crazyflie]): 在仿真中，与列表中的其他对象避碰。
    #                                   在真实硬件中忽略该列表，并对同一信道上的所有Crazyflie进行避碰。
    #  @param ellipsoidRadii (array-like of float[3]): 椭球碰撞体的半径，单位：米。
    def enableCollisionAvoidance(self, others, ellipsoidRadii):
        """Enables onboard collision avoidance.

        When enabled, avoids colliding with other Crazyflies by using the
        Buffered Voronoi Cells method [1]. Computation is performed onboard.

        Args:
            others (List[Crazyflie]): List of other :obj:`Crazyflie` objects.
                In simulation, collision avoidance is checked only with members
                of this list.  With real hardware, this list is **ignored**, and
                collision avoidance is checked with all other Crazyflies on the
                same radio channel.
            ellipsoidRadii (array-like of float[3]): Radii of collision volume ellipsoid in meters.
                The Crazyflie's boundary for collision checking is a tall
                ellipsoid. This accounts for the downwash effect: Due to the
                fast-moving stream of air produced by the rotors, the safe
                distance to pass underneath another rotorcraft is much further
                than the safe distance to pass to the side.

        [1] D. Zhou, Wang, Z., Bandyopadhyay, S., and Schwager, M.
            Fast, On-line Collision Avoidance for Dynamic Vehicles using
            Buffered Voronoi Cells.  IEEE Robotics and Automation Letters
            (RA-L), vol. 2, no. 2, pp. 1047 - 1054, 2017.
            https://msl.stanford.edu/fast-line-collision-avoidance-dynamic-vehicles-using-buffered-voronoi-cells
        """
        pass
    
    ## @brief 禁用机载避碰功能。
    #  - @anchor disableCollisionAvoidance
    def disableCollisionAvoidance(self):
        """Disables onboard collision avoidance."""
        pass

    ## @brief 执行起飞动作，以设定的速度飞到指定高度并保持悬停。
    #  - @anchor takeoff
    #  @param targetHeight (float): 期望的悬停高度（z坐标，米）。
    #  @param duration (float): 达到目标高度所需时间（秒）。
    #  @param groupMask (int): 组掩码，用于同步多个飞行器动作。
    def takeoff(self, targetHeight, duration, groupMask=0):
        """Execute a takeoff - fly straight up, then hover indefinitely.

        Asynchronous command; returns immediately.

        Args:
            targetHeight (float): The z-coordinate at which to hover.
            duration (float): How long until the height is reached. Seconds.
            groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
        """
        self.ctrl.takeoff(-targetHeight + self.mapini)

    ## @brief 执行降落动作，以设定的速度飞回指定高度。
    #  - @anchor land
    #  @param targetHeight (float): 降落到的目标高度（z坐标，米）。
    #  @param duration (float): 达到目标高度所需时间（秒）。
    #  @param groupMask (int): 组掩码。
    def land(self, targetHeight, duration, groupMask=0):
        """Execute a landing - fly straight down. User must cut power after.

        Asynchronous command; returns immediately.

        Args:
            targetHeight (float): The z-coordinate at which to land. Meters.
                Usually should be a few centimeters above the initial position
                to ensure that the controller does not try to penetrate the
                floor if the mocap coordinate origin is not perfect.
            duration (float): How long until the height is reached. Seconds.
            groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
        """
        self.ctrl.land(-targetHeight + self.mapini)

    ## @brief 停止飞行器（在低级命令模式下切断电机电源）。
    #  - @anchor stop
    #  将电机关闭，但后续低/高级命令能再次重启电机。
    #  @param groupMask (int): 组掩码。
    def stop(self, groupMask=0):
        """Cuts power to the motors when operating in low-level command mode.

        Intended for non-emergency scenarios, e.g. landing with the possibility
        of taking off again later. Future low- or high-level commands will
        restart the motors.

        Args:
            groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
        """
        self.ctrl.stopRun()

    ## @brief 平滑移动到指定目标位置（并悬停），支持绝对或相对坐标和偏航角设定。
    #  - @anchor goTo
    #  @param goal (iterable of float): 目标位置 [x, y, z]，单位：米。
    #  @param yaw (float): 目标偏航角（弧度）。
    #  @param duration (float): 从当前状态到达目标点所需时间（秒）。
    #  @param relative (bool): 若为True，则goal为相对当前位置的偏移量；否则为绝对坐标。
    #  @param groupMask (int): 组掩码。
    def goTo(self, goal, yaw, duration, relative=False, groupMask=0):
        """Move smoothly to the goal, then hover indefinitely.

        Asynchronous command; returns immediately.

        Plans a smooth trajectory from the current state to the goal position.
        Will stop smoothly at the goal with minimal overshoot. If the current
        state is at hover, the planned trajectory will be a straight line;
        however, if the current velocity is nonzero, the planned trajectory
        will be a smooth curve.

        Plans the trajectory by solving for the unique degree-7 polynomial that
        satisfies the initial conditions of the current position, velocity,
        and acceleration, and ends at the goal position with zero velocity and
        acceleration. The jerk (derivative of acceleration) is fixed at zero at
        both boundary conditions.

        .. warning::
            Calling ``goTo`` rapidly and/or with short durations (<< 1 sec) can
            cause instability. Consider using :meth:`cmdPosition()` instead.

        Args:
            goal (iterable of 3 floats): The goal position. Meters.
            yaw (float): The goal yaw angle (heading). Radians.
            duration (float): How long until the goal is reached. Seconds.
            relative (bool): If true, the goal position is interpreted as a
                relative offset from the current position. Otherwise, the goal
                position is interpreted as absolute coordintates in the global
                reference frame.
            groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
        """
        pos = [0, 0, 0]
        pos[0] = -goal[1]
        pos[1] = -goal[0]
        pos[2] = -goal[2] + self.mapini
        self.ctrl.send_pos_ned(pos, yaw)

    ## @brief 上传分段多项式轨迹以供后续执行。
    #  - @anchor uploadTrajectory
    #  @param trajectoryId (int): 轨迹ID号，可上传多个轨迹，每条轨迹有唯一ID。
    #  @param pieceOffset (int): 分段起始偏移量（具体含义待补充）。
    #  @param trajectory (Trajectory): 分段多项式轨迹对象，详见uav_trajectory.py说明。
    def uploadTrajectory(self, trajectoryId, pieceOffset, trajectory):
        """Uploads a piecewise polynomial trajectory for later execution.

        See uav_trajectory.py for more information about piecewise polynomial
        trajectories.

        Args:
            trajectoryId (int): ID number of this trajectory. Multiple
                trajectories can be uploaded. TODO: what is the maximum ID?
            pieceOffset (int): TODO(whoenig): explain this.
            trajectory (:obj:`pycrazyswarm.uav_trajectory.Trajectory`): Trajectory object.
        """
        pass

    ## @brief 开始执行预先上传的轨迹。
    #  - @anchor startTrajectory
    #  @param trajectoryId (int): 轨迹ID，对应uploadTrajectory上传的ID。
    #  @param timescale (float): 时间缩放因子，大于1表示执行时间更长。
    #  @param reverse (bool): 若为True，轨迹按时间反向执行。
    #  @param relative (bool): 若为True，轨迹的起点位置为当前定位点，否则为绝对位置。
    #  @param groupMask (int): 组掩码，用于多机同步控制
    def startTrajectory(self, trajectoryId, timescale=1.0, reverse=False, relative=True, groupMask=0):
        """Begins executing a previously uploaded trajectory.

        Asynchronous command; returns immediately.

        Args:
            trajectoryId (int): ID number as given to :meth:`uploadTrajectory()`.
            timescale (float): Scales the trajectory duration by this factor.
                For example if timescale == 2.0, the trajectory will take twice
                as long to execute as the nominal duration.
            reverse (bool): If true, executes the trajectory backwards in time.
            relative (bool): If true (default), the position of the trajectory
                is shifted such that it begins at the current position setpoint.
                This is usually the desired behavior.
            groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
        """
        pass

    ## @brief 通知飞行器低级控制指令（如cmdVelocityWorld、cmdFullState）即将停止发送，随后返回上层行为。
    #  - @anchor notifySetpointsStop
    #  @param remainValidMillisecs (int): 最后一次低级控制指令在无更新下保持有效的毫秒数。
    #  @param groupMask (int): 组掩码。
    def notifySetpointsStop(self, remainValidMillisecs=100, groupMask=0):
        """Informs that streaming low-level setpoint packets are about to stop.

        Streaming setpoints are :meth:`cmdVelocityWorld`, :meth:`cmdFullState`,
        and so on. For safety purposes, they normally preempt onboard high-level
        commands such as :meth:`goTo`.

        Once preempted, the Crazyflie will not switch back to high-level
        commands (or other behaviors determined by onboard planning/logic) until
        a significant amount of time has elapsed where no low-level setpoint
        was received.

        This command short-circuits that waiting period to a user-chosen time.
        It should be called after sending the last low-level setpoint, and
        before sending any high-level command.

        A common use case is to execute the :meth:`land` command after using
        streaming setpoint modes.

        Args:
            remainValidMillisecs (int): Number of milliseconds that the last
                streaming setpoint should be followed before reverting to the
                onboard-determined behavior. May be longer e.g. if one radio
                is controlling many robots.
        """
        pass

    ## @brief 返回最近一次由运动捕捉系统测量的飞行器真实位置。
    #  - @anchor position
    #  如无测量数据，本函数将阻塞直到接收到第一条位置测量数据。
    #  @return np.array[3]: 当前位置（米）。
    def position(self):
        """Returns the last true position measurement from motion capture.

        If at least one position measurement for this robot has been received
        from the motion capture system since startup, this function returns
        immediately with the most recent measurement. However, if **no**
        position measurements have been received, it blocks until the first
        one arrives.

        Returns:
            position (np.array[3]): Current position. Meters.
        """
        pos = self.ctrl.get_pos_ned()
        ret_pos = [0, 0, 0]
        ret_pos[0] = -pos[1]
        ret_pos[1] = -pos[0]
        ret_pos[2] = -pos[2] + self.mapini
        return np.array(ret_pos)
   
    ## @brief 获取指定名称的机载参数的当前值。
    #  - @anchor getParam
    #  参数由固件中的变量决定，可用于调节控制器或其他内部行为。
    #  @param name (str): 参数名称。
    #  @return Any: 参数值。
    def getParam(self, name):
        """Returns the current value of the onboard named parameter.

        Parameters are named values of various primitive C types that control
        the firmware's behavior. For more information, see
        https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/logparam/.

        Parameters are read at system startup over the radio and cached.
        The ROS launch file can also be used to set parameter values at startup.
        Subsequent calls to :meth:`setParam()` will update the cached value.
        However, if the parameter changes for any other reason, the cached value
        might become stale. This situation is not common.

        Args:
            name (str): The parameter's name.

        Returns:
            value (Any): The parameter's value.
        """
        pass

    ## @brief 设置指定名称参数的值。
    #  - @anchor setParam
    #  @param name (str): 参数名称。
    #  @param value (Any): 参数新值。
    def setParam(self, name, value):
        """Changes the value of the given parameter.

        See :meth:`getParam()` docs for overview of the parameter system.

        Args:
            name (str): The parameter's name.
            value (Any): The parameter's value.
        """
        pass

    ## @brief 批量设置多个参数的值。
    #  - @anchor setParams
    #  @param params (Dict[str, Any]): 参数名-值对的字典。
    def setParams(self, params):
        """Changes the value of several parameters at once.

        See :meth:`getParam()` docs for overview of the parameter system.

        Args:
            params (Dict[str, Any]): Dict of parameter names/values.
        """
        pass

    ## @brief 发送全状态(setFullState)控制指令，包括位置、速度、加速度、偏航角及角速度。
    #  - @anchor cmdFullState
    #  发送此类型指令会使飞行器切换到低级控制模式，不再响应高层命令（如land、goTo）。
    #  @param pos (array-like[3]): 位置（米）。
    #  @param vel (array-like[3]): 速度（米/秒）。
    #  @param acc (array-like[3]): 加速度（米/秒²）。
    #  @param yaw (float): 偏航角（弧度）。
    #  @param omega (array-like[3]): 机体坐标系下的角速度（弧度/秒）。
    def cmdFullState(self, pos, vel, acc, yaw, omega):
        """Sends a streaming full-state controller setpoint command.

        The full-state setpoint is most useful for aggressive maneuvers where
        feedforward inputs for acceleration and angular velocity are critical
        to obtaining good tracking performance. Full-state setpoints can be
        obtained from any trajectory parameterization that is at least three
        times differentiable, e.g. piecewise polynomials.

        Sending a streaming setpoint of any type will force a change from
        high-level to low-level command mode. Currently, there is no mechanism
        to change back, but it is a high-priority feature to implement.
        This means it is not possible to use e.g. :meth:`land()` or
        :meth:`goTo()` after a streaming setpoint has been sent.

        Args:
            pos (array-like of float[3]): Position. Meters.
            vel (array-like of float[3]): Velocity. Meters / second.
            acc (array-like of float[3]): Acceleration. Meters / second^2.
            yaw (float): Yaw angle. Radians.
            omega (array-like of float[3]): Angular velocity in body frame.
                Radians / sec.
        """
        # self.cmdFullStateMsg.header.stamp = rospy.Time.now()
        # self.cmdFullStateMsg.header.seq += 1
        # self.cmdFullStateMsg.pose.position.x = pos[0]
        # self.cmdFullStateMsg.pose.position.y = pos[1]
        # self.cmdFullStateMsg.pose.position.z = pos[2]
        # self.cmdFullStateMsg.twist.linear.x = vel[0]
        # self.cmdFullStateMsg.twist.linear.y = vel[1]
        # self.cmdFullStateMsg.twist.linear.z = vel[2]
        # self.cmdFullStateMsg.acc.x = acc[0]
        # self.cmdFullStateMsg.acc.y = acc[1]
        # self.cmdFullStateMsg.acc.z = acc[2]
        # self.cmdFullStateMsg.pose.orientation = geometry_msgs.msg.Quaternion(
        #     *tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))
        # self.cmdFullStateMsg.twist.angular.x = omega[0]
        # self.cmdFullStateMsg.twist.angular.y = omega[1]
        # self.cmdFullStateMsg.twist.angular.z = omega[2]
        # self.cmdFullStatePublisher.publish(self.cmdFullStateMsg)
        adj_pos = [0, 0, 0]
        adj_pos[0] = -pos[1]
        adj_pos[1] = -pos[0]
        adj_pos[2] = -pos[2] + self.mapini
        adj_vel = [0, 0, 0]
        adj_vel[0] = -vel[1]
        adj_vel[1] = -vel[0]
        adj_vel[2] = -vel[2]
        adj_acc = [0, 0, 0]
        adj_acc[0] = -acc[1]
        adj_acc[1] = -acc[0]
        adj_acc[2] = -acc[2]
        self.ctrl.send_full(adj_pos, adj_vel, adj_acc, yaw)
    
    ## @brief 发送世界坐标系下的速度与偏航角速度控制指令。
    #  - @anchor cmdVelocityWorld
    #  该指令以速度矢量和偏航角速度作为输入。
    #  发送流式(setpoint)指令会使飞行器进入低级控制模式，不再响应高层指令（如land、goTo）。
    #  
    #  @param vel (array-like of float[3]): 期望速度（米/秒）。
    #  @param yawRate (float): 偏航角速度（度/秒）。
    def cmdVelocityWorld(self, vel, yawRate):
        """Sends a streaming velocity-world controller setpoint command.

        In this mode, the PC specifies desired velocity vector and yaw rate.
        The onboard controller will try to achive this velocity.

        NOTE: the Mellinger controller is Crazyswarm's default controller, but
        it has not been tuned (or even tested) for velocity control mode.
        Switch to the PID controller by changing
        `firmwareParams.stabilizer.controller` to `1` in your launch file.

        Sending a streaming setpoint of any type will force a change from
        high-level to low-level command mode. Currently, there is no mechanism
        to change back, but it is a high-priority feature to implement.
        This means it is not possible to use e.g. :meth:`land()` or
        :meth:`goTo()` after a streaming setpoint has been sent.

        Args:
            vel (array-like of float[3]): Velocity. Meters / second.
            yawRate (float): Yaw angular velocity. Degrees / second.
        """
        adj_vel = [0, 0, 0]
        adj_vel[0] = -vel[1]
        adj_vel[1] = -vel[0]
        adj_vel[2] = -adj_vel[2]
        self.ctrl.send_vel_ned(adj_vel, yawRate)

    # def cmdAcc(self, acc, yawRate):
    #     self.cmdVelocityWorldMsg.header.stamp = rospy.Time.now()
    #     self.cmdVelocityWorldMsg.header.seq += 1
    #     self.cmdVelocityWorldMsg.vel.x = acc[0]
    #     self.cmdVelocityWorldMsg.vel.y = acc[1]
    #     self.cmdVelocityWorldMsg.vel.z = acc[2]
    #     self.cmdVelocityWorldMsg.yawRate = yawRate
    #     self.cmdVelocityWorldPublisher.publish(self.cmdVelocityWorldMsg)

    ## @brief 停止飞行器，在高层命令模式下切断电机动力。
    #  - @anchor cmdStop
    #  后续的低/高层命令可以重新启动电机。
    def cmdStop(self):
        """Interrupts any high-level command to stop and cut motor power.

        Intended for non-emergency scenarios, e.g. landing with the possibility
        of taking off again later. Future low- or high-level commands will
        restart the motors. Equivalent of :meth:`stop()` when in high-level mode.
        """
        self.ctrl.stopRun()

    ## @brief 发送用于初学者模式（绝对滚转/俯仰角度、偏航角速度和推力）的流式指令。
    #  - @anchor cmdVel
    #  
    #  在此模式下，通过绝对角度而不是角速度来控制滚转和俯仰，适合初学者手动操作。
    #  发送流式指令会切换到低级控制模式，不能再使用高层指令（如land、goTo）。
    #  
    #  @param roll (float): 滚转角度（度），正值表示向右滚转。
    #  @param pitch (float): 俯仰角度（度），正值表示向前/向下俯仰。
    #  @param yawrate (float): 偏航角速度（度/秒），正值表示逆时针旋转。
    #  @param thrust (float): 推力大小，无单位标度，范围[0, 65535]。
    def cmdVel(self, roll, pitch, yawrate, thrust):
        """Sends a streaming command of the "easy mode" manual control inputs.

        The (absolute roll & pitch, yaw rate, thrust) inputs are typically
        used for manual flying by beginners or causal pilots, as opposed to the
        "acrobatic mode" inputs where roll and pitch rates are controlled
        instead of absolute angles. This mode limits the possible maneuvers,
        e.g. it is not possible to do a flip because the controller joystick
        would need to rotate 360 degrees.

        For more information on streaming setpoint commands, see the
        :meth:`cmdFullState()` documentation.

        !NOTE!: The angles and angular velocities in this command are in
        degrees, whereas they are in radians for cmdFullState.

        TODO: should we change the name from cmdVel to something else?
        IMO (japreiss), cmdVel implies controlling linear velocity.

        Args:
            roll (float): Roll angle. Degrees. Positive values == roll right.
            pitch (float): Pitch angle. Degrees. Positive values == pitch
                forward/down.
            yawrate (float): Yaw angular velocity. Degrees / second. Positive
                values == turn counterclockwise.
            thrust (float): Thrust magnitude. Non-meaningful units in [0, 2^16),
                where the maximum value corresponds to maximum thrust.
        """
        self.ctrl.send_att_thrust([roll, pitch, yawrate], thrust)

    ## @brief 发送绝对位置和偏航角的流式指令，飞行器将移动并保持在指定位置上空。
    #  - @anchor cmdPosition
    #  
    #  若缓慢改变位置目标，则此函数适用；若目标点变化较慢且距离较远，建议使用goTo()。
    #  发送流式指令会切换到低级控制模式，不能再使用高层指令（如land、goTo）。
    #  
    #  @param pos (array-like of float[3]): 期望位置（米）。
    #  @param yaw (float): 偏航角（弧度）。
    def cmdPosition(self, pos, yaw=0):
        """Sends a streaming command of absolute position and yaw setpoint.

        Useful for slow maneuvers where a high-level planner determines the
        desired position, and the rest is left to the onboard controller.

        For more information on streaming setpoint commands, see the
        :meth:`cmdFullState()` documentation.

        .. warning::
            As a streaming setpoint, ``cmdPosition`` must be sent many times
            per second (10Hz is a conservative minimum). For applications that
            generate goal positions slowly, :meth:`goTo()` may be more
            appropriate, especially if the goal positions are far apart.

        Args:
            pos (array-like of float[3]): Position. Meters.
            yaw (float): Yaw angle. Radians.
        """
        adj_pos = [0, 0, 0]
        adj_pos[0] = -pos[1]
        adj_pos[1] = -pos[0]
        adj_pos[2] = -pos[2] + self.mapini
        self.ctrl.send_pos_ned(adj_pos, yaw)

    ## @brief 设置LED颜色（需要将param "ring/effect"设置为7才能生效）。
    #  - @anchor setLEDColor
    #  此为阻塞命令，在大编队或高频率变化时可能引发稳定性问题。
    #  
    #  @param r (float): 红色分量，[0, 1]。
    #  @param g (float): 绿色分量，[0, 1]。
    #  @param b (float): 蓝色分量，[0, 1]。
    def setLEDColor(self, r, g, b):
        """Sets the color of the LED ring deck.

        While most params (such as PID gains) only need to be set once, it is
        common to change the LED ring color many times during a flight, e.g.
        as some kind of status indicator. This method makes it convenient.

        PRECONDITION: The param "ring/effect" must be set to 7 (solid color)
        for this command to have any effect. The default mode uses the ring
        color to indicate radio connection quality.

        This is a blocking command, so it may cause stability problems for
        large swarms and/or high-frequency changes.

        Args:
            r (float): Red component of color, in range [0, 1].
            g (float): Green component of color, in range [0, 1].
            b (float): Blue component of color, in range [0, 1].
        """
        pass

    ## @brief 读取消息（占位函数，尚未实现）。
    #  - @anchor readmsg
    def readmsg(self):
        pass

    ## @brief 获取当前飞行器运动位置（通过底层接口获得NED坐标系下位置后转换）。
    #  - @anchor getMotionPos
    #  @return np.array([x, y, z]): 当前位置（米）。
    def getMotionPos(self):
        pos = self.ctrl.get_pos_ned()
        ret_pos = [0, 0, 0]
        ret_pos[0] = -pos[1]
        ret_pos[1] = -pos[0]
        ret_pos[2] = -pos[2] + self.mapini
        return np.array(ret_pos)

    ## @brief 获取当前飞行器运动位置（通过底层接口获得NED坐标系下位置后转换）。
    #  - @anchor getMotionVel
    #  @return np.array([x, y, z]): 当前位置（米）。
    def getMotionVel(self):
        vel = self.ctrl.get_vel_ned()
        ret_vel = [0, 0, 0]
        ret_vel[0] = -vel[1]
        ret_vel[1] = -vel[0]
        ret_vel[2] = -vel[2]
        return np.array(ret_vel)

    ## @brief 获取当前飞行器运动加速度（通过底层接口获得NED坐标系下加速度后转换）。
    #  - @anchor getMotionAcc
    #  @return np.array([ax, ay, az]): 当前加速度（米/秒²）。
    def getMotionAcc(self):
        acc = self.ctrl.get_vel_ned()
        ret_acc = [0, 0, 0]
        ret_acc[0] = -acc[1]
        ret_acc[1] = -acc[0]
        ret_acc[2] = -acc[2]
        return np.array(ret_acc)

## @class Translation
#  @brief 用于表示空间中的平移信息（x, y, z）。
class Translation:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

## @class Transform
#  @brief 用于表示一个变换，包含平移部分。
class Transform:
    def __init__(self):
        self.translation = Translation()

## @class Msgg
#  @brief 用于封装消息数据结构，包含transform对象。
class Msgg:
    def __init__(self):
        self.transform = Transform()

## @class CrazyflieServer
#  @brief 用于广播指令给所有机器人，以及容纳所有Crazyflie对象的服务器类。
#
#  CrazyflieServer通过解析YAML文件或字符串，从而初始化多个Crazyflie对象并维护其引用。
#  提供批量指令接口（如同时起飞、降落、GoTo等）实现多机同步动作。
class CrazyflieServer:
    """Object for broadcasting commands to all robots at once.

    Also is the container for the individual :obj:`Crazyflie` objects.

    Attributes:
        crazyfliesById (Dict[int, Crazyflie]): Index to the same Crazyflie
            objects by their ID number (last byte of radio address).
    """

    ## @brief 构造函数，初始化CrazyflieServer。
    #  - @anchor __init__
    #  @param crazyflies_yaml (str): 配置文件路径或YAML字符串。
    #  @param num (int): 飞行器数量，默认1。
    #  @param ids (list[int]): 飞行器ID列表，如果为空则按照1开始递增分配ID。
    #  @param mode (int): 控制模式，影响构建Crazyflie实例的方式。
    def __init__(self, crazyflies_yaml="../launch/crazyflies.yaml", num=1, ids=None, mode=7):
        """Initialize the server. Waits for all ROS services before returning.

        Args:
            crazyflies_yaml (str): If ends in ".yaml", interpret as a path and load
                from file. Otherwise, interpret as YAML string and parse
                directly from string.
        """
        self.vehicle_num = int(num)
        self.ini_poses = list()
        self.interval = 2
        self.init_height = 0
        self.crazyflies = []
        self.crazyfliesById = dict()
        self.init_poses()
        self.tf = None
        self.msgg = Msgg()
        for i in range(self.vehicle_num):
            if ids:
                id = ids[i]
            else:
                id = i + 1
            initialPosition = self.ini_poses[i]
            cf = Crazyflie(i + 1, initialPosition, self.tf, mode)
            self.crazyflies.append(cf)
            self.crazyfliesById[id] = cf
        while True:
            ini = []
            for cf in self.crazyflies:
                ini.append(cf.ini_success)
            if all(ini):
                break

    ## @brief 初始化飞行器的起始位置列表。
    #  - @anchor init_poses
    #  根据vehicle_num计算出sqrt_num，构建一个sqrt_num x sqrt_num的网格阵列，
    #  并在此网格中为每个飞行器分配初始位置。
    def init_poses(self):
        sqrt_num = 1
        while True:
            if sqrt_num * sqrt_num >= self.vehicle_num:
                break
            sqrt_num = sqrt_num + 1

        for i in range(0, sqrt_num):
            for j in range(0, sqrt_num):
                self.ini_poses.append([i * self.interval, j * self.interval, self.init_height])
        self.ini_poses = self.ini_poses[:self.vehicle_num]


    ## @brief 紧急停止所有飞行器，切断电源并忽略后续指令。
    #  - @anchor emergency
    #  若发生控制逻辑错误或紧急情况，可调用此方法停止所有飞行器并防止后续操作。
    def emergency(self):
        """Emergency stop. Cuts power; causes future commands to be ignored.

        This command is useful if the operator determines that the control
        script is flawed, and that continuing to follow it will cause wrong/
        self-destructive behavior from the robots. In addition to cutting
        power to the motors, it ensures that any future commands, both high-
        level and streaming, will have no effect.

        The only ways to reset the firmware after an emergency stop has occurred
        are a physical hard reset or an nRF51 Reboot command.
        """
        pass

    ## @brief 广播指令让所有匹配groupMask的飞行器起飞到指定高度并悬停。
    #  - @anchor takeoff
    #  异步操作，函数立即返回。
    #  @param targetHeight (float): 起飞后的悬停高度（米）。
    #  @param duration (float): 达到目标高度所需时间（秒）。
    #  @param groupMask (int): 组掩码，用于同时控制子集飞行器。
    def takeoff(self, targetHeight, duration, groupMask=0):
        """Broadcasted takeoff - fly straight up, then hover indefinitely.

        Broadcast version of :meth:`Crazyflie.takeoff()`. All robots that match the
        groupMask take off at exactly the same time. Use for synchronized
        movement. Asynchronous command; returns immediately.

        Args:
            targetHeight (float): The z-coordinate at which to hover.
            duration (float): How long until the height is reached. Seconds.
            groupMask (int): Group mask bits. See :meth:`setGroupMask()` doc.
        """

        for cf in self.crazyflies:
            cf.takeoff(targetHeight, duration, groupMask)

    ## @brief 广播指令让所有匹配groupMask的飞行器降落到指定高度。
    #  - @anchor land
    #  异步操作，函数立即返回。
    #  降落结束后使用cmdStop()停机。
    #  @param targetHeight (float): 降落目标高度（米），通常略高于起点以避免地面误差。
    #  @param duration (float): 达到目标高度所需时间（秒）。
    #  @param groupMask (int): 组掩码。
    def land(self, targetHeight, duration, groupMask=0):
        """Broadcasted landing - fly straight down. User must cut power after.

        Broadcast version of :meth:`Crazyflie.land()`. All robots that match the
        groupMask land at exactly the same time. Use for synchronized
        movement. Asynchronous command; returns immediately.

        Args:
            targetHeight (float): The z-coordinate at which to land. Meters.
                Usually should be a few centimeters above the initial position
                to ensure that the controller does not try to penetrate the
                floor if the mocap coordinate origin is not perfect.
            duration (float): How long until the height is reached. Seconds.
            groupMask (int): Group mask bits. See :meth:`Crazyflie.setGroupMask()` doc.
        """
        for cf in self.crazyflies:
            cf.land(targetHeight, duration, groupMask)
        time.sleep(2)
        for cf in self.crazyflies:
            cf.cmdStop()

    # def stop(self, groupMask = 0):
    #     self.stopService(groupMask)

    ## @brief 广播goTo指令，让所有匹配groupMask的飞行器同时移动到目标点。
    #  - @anchor goTo
    #  使用相对坐标的移动方式，让飞行器从当前位置平滑移动到新目标位置。
    #  异步操作，函数立即返回。
    #  @param goal (iterable of float[3]): 目标点偏移（米）。
    #  @param yaw (float): 目标偏航角（弧度）。
    #  @param duration (float): 达到目标点所需时间（秒）。
    #  @param groupMask (int): 组掩码。
    def goTo(self, goal, yaw, duration, groupMask=0):
        """Broadcasted goTo - Move smoothly to goal, then hover indefinitely.

        Broadcast version of :meth:`Crazyflie.goTo()`. All robots that match the
        groupMask start moving at exactly the same time. Use for synchronized
        movement. Asynchronous command; returns immediately.

        While the individual goTo() supports both relative and absolute
        coordinates, the broadcasted goTo only makes sense with relative
        coordinates (since absolute broadcasted goTo() would cause a collision).
        Therefore, there is no `relative` kwarg.

        See docstring of :meth:`Crazyflie.goTo()` for additional details.

        Args:
            goal (iterable of 3 floats): The goal offset. Meters.
            yaw (float): The goal yaw angle (heading). Radians.
            duration (float): How long until the goal is reached. Seconds.
            groupMask (int): Group mask bits. See :meth:`Crazyflie.setGroupMask()` doc.
        """
        for cf in self.crazyflies:
            cf.goTo(goal, yaw, duration, groupMask)

    ## @brief 广播开始执行已上传的轨迹。
    #  - @anchor startTrajectory
    #  异步操作，函数立即返回。
    #  @param trajectoryId (int): 轨迹ID，对应uploadTrajectory的ID。
    #  @param timescale (float): 时间缩放因子。
    #  @param reverse (bool): 若True，则反向执行轨迹。
    #  @param relative (bool): 若True，则轨迹相对于当前位置偏移执行。
    #  @param groupMask (int): 组掩码。
    def startTrajectory(self, trajectoryId, timescale=1.0, reverse=False, relative=True, groupMask=0):
        """Broadcasted - begins executing a previously uploaded trajectory.

        Broadcast version of :meth:`Crazyflie.startTrajectory()`.
        Asynchronous command; returns immediately.

        Args:
            trajectoryId (int): ID number as given to :meth:`Crazyflie.uploadTrajectory()`.
            timescale (float): Scales the trajectory duration by this factor.
                For example if timescale == 2.0, the trajectory will take twice
                as long to execute as the nominal duration.
            reverse (bool): If true, executes the trajectory backwards in time.
            relative (bool): If true (default), the position of the trajectory
                is shifted such that it begins at the current position setpoint.
            groupMask (int): Group mask bits. See :meth:`Crazyflie.setGroupMask()` doc.
        """
        pass
    
    ## @brief 广播设置参数命令，参考Crazyflie.setParam()。
    #  - @anchor setParam
    #  @param name (str): 参数名称。
    #  @param value (Any): 参数新值。    
    def setParam(self, name, value):
        """Broadcasted setParam. See Crazyflie.setParam() for details."""
        pass
    
    ## @brief 读取消息的占位方法（未实现）。
    #  - @anchor readmsg
    def readmsg(self):
        pass
    
    ## @brief 获取特定ID飞行器的位置，并更新msgg对象的transform.translation以供使用。
    #  - @anchor singleGetpos
    #  @param id (int): 飞行器ID。
    def singleGetpos(self, id):
        cf = self.crazyfliesById[id]
        pos = cf.position()
        self.msgg.transform.translation.x = pos[0]
        self.msgg.transform.translation.y = pos[1]
        self.msgg.transform.translation.z = pos[2]

    # def Getpos(self,id):
    #     if id == 0:
    #         self.msgg=rospy.wait_for_message("/vicon/cf1/cf1",TransformStamped,timeout=None)
    #     elif id == 1:
    #         self.msgg=rospy.wait_for_message("/vicon/cf2/cf2",TransformStamped,timeout=None)
    #     elif id == 2:
    #         self.msgg=rospy.wait_for_message("/vicon/cf3/cf3",TransformStamped,timeout=None)
    #     elif id == 3:
    #         self.msgg=rospy.wait_for_message("/vicon/cf4/cf4",TransformStamped,timeout=None)
    #     else:
    #         self.msgg=rospy.wait_for_message("/vicon/cf5/cf5",TransformStamped,timeout=None)

    def Getpos(self, id):
        """实现在这里不合适"""
        cf = self.crazyfliesById[id]
        pos = cf.position()
        self.msgg.transform.translation.x = pos[0]
        self.msgg.transform.translation.y = pos[1]
        self.msgg.transform.translation.z = pos[2]

    def Getvel(self, id):
        """实现在这里不合适"""
        pass

    def Getacc(self, id):
        """实现在这里不合适"""
        pass
