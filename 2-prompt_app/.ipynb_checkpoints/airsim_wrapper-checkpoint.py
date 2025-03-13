import sys
sys.path.append('../external-libraries')
import airsim
import math
import numpy as np

objects_dict = {
    "turbine1": "BP_Wind_Turbines_C_1",
    "turbine2": "StaticMeshActor_2",
    "solarpanels": "StaticMeshActor_146",
    "crowd": "StaticMeshActor_6",
    "car": "StaticMeshActor_10",
    "tower1": "SM_Electric_trellis_179",
    "tower2": "SM_Electric_trellis_7",
    "tower3": "SM_Electric_trellis_8",
}


class AirSimWrapper:
    def __init__(self):
        self.client = airsim.MultirotorClient()#run in some machine of airsim,otherwise,set ip="" of airsim
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)


    def takeoff(self):
        """
        takeoff the drone
        """
        self.client.takeoffAsync().join()

    def land(self):
        """
        land the drone
        """
        self.client.landAsync().join()


    def get_drone_position(self):
        """
        get the current position of the drone
        :return: position, the current position of the drone
        """
        pose = self.client.simGetVehiclePose()
        return [pose.position.x_val, pose.position.y_val, pose.position.z_val]

    def fly_to(self, point):
        """
        fly the drone to a specific point
        :param point: the target point
        """
        if point[2] > 0:
            self.client.moveToPositionAsync(point[0], point[1], -point[2], 5).join()
        else:
            self.client.moveToPositionAsync(point[0], point[1], point[2], 5).join()

    def fly_path(self, points):
        """
        fly the drone along a specific path
        :param points: the path
        """
        airsim_points = []
        for point in points:
            if point[2] > 0:
                airsim_points.append(airsim.Vector3r(point[0], point[1], -point[2]))
            else:
                airsim_points.append(airsim.Vector3r(point[0], point[1], point[2]))
        self.client.moveOnPathAsync(airsim_points, 5, 120, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0), 20, 1).join()

    def set_yaw(self, yaw):
        """
        set the yaw angle of the drone
        """
        self.client.rotateToYawAsync(yaw, 5).join()

    def get_yaw(self):
        """
        get the yaw angle of the drone
        :return: yaw_degree, the yaw angle of the drone in degree
        """
        orientation_quat = self.client.simGetVehiclePose().orientation
        yaw = airsim.to_eularian_angles(orientation_quat)[2] # get the yaw angle
        yaw_degree = math.degrees(yaw)
        return yaw_degree # return the yaw angle in degree

    def get_position(self, object_name):
        """
        get the position of a specific object
        :param object_name: the name of the object
        :return: position, the position of the object
        """
        query_string = objects_dict[object_name] + ".*"
        object_names_ue = []
        while len(object_names_ue) == 0:
            object_names_ue = self.client.simListSceneObjects(query_string)
        pose = self.client.simGetObjectPose(object_names_ue[0])
        return [pose.position.x_val, pose.position.y_val, pose.position.z_val]

    def reset(self):
        self.client.reset()

    def get_distance(self):
        """
        get the distance between the quadcopter and the nearest obstacle
        :return: distance, the distance between the quadcopter and the nearest obstacle
        """
        distance = 100000000

        pose = self.client.simGetVehiclePose()  # get the current pose of the quadcopter
        v_p = [pose.position.x_val, pose.position.y_val, pose.position.z_val]

        # get lidar data
        lidarData = self.client.getLidarData()
        if len(lidarData.point_cloud) < 3:
            return distance # if no points are received from the lidar, return a big distance as 100000000

        points = np.array(lidarData.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))
        distance_list = []
        for p in points:
            distance = np.linalg.norm(np.array(v_p) - p)
            distance_list.append(distance)

        distance = min(distance_list)
        return distance

if __name__ == "__main__":
    airsim_wrapper = AirSimWrapper()
    airsim_wrapper.takeoff()
    airsim_wrapper.fly_to([20, 0, -10])
    airsim_wrapper.land()
    print("done")
