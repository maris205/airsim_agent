import sys
sys.path.append('../external-libraries')
import airsim
import math
import numpy as np
import cv2
import base64
import os
from openai import OpenAI
from gdino import GroundingDINOAPIWrapper, visualize
from PIL import Image
import uuid

api_key="ffd77d7c-f420-4b69-8557-80e7fa85c8b9" #使用自己的key，火山方舟
gdino_token = "885af84f607caa6a12ba509b6c3c03a7" #使用自己的token，dino

objects_dict = {
    "可乐": "airsim_coca",
    "兰花": "airsim_lanhua",
    "椰子水": "airsim_yezishui",
    "小鸭子": "airsim_duck",
    "镜子": "airsim_mirror_06",
    "方桌": "airsim_fangzhuo",
}


class AirSimWrapper:
    def __init__(self):
        #无人机client
        self.client = airsim.MultirotorClient()#run in some machine of airsim,otherwise,set ip="" of airsim
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)


        #大模型client
        self.llm_client = OpenAI(
                api_key=api_key,
                base_url="https://ark.cn-beijing.volces.com/api/v3",
        )


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
        yaw_degree = self.get_yaw()  # angle in degree
        return [pose.position.x_val, pose.position.y_val, pose.position.z_val,yaw_degree]
        
    def fly_to(self, point):
        """
        fly the drone to a specific point
        :param point: the target point
        """
        if point[2] > 0:
            self.client.moveToPositionAsync(point[0], point[1], -point[2], 1).join()
        else:
            self.client.moveToPositionAsync(point[0], point[1], point[2], 1).join()

            

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
        #self.client.moveOnPathAsync(airsim_points, 5, 120, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0), 20, 1).join()
        self.client.moveOnPathAsync(airsim_points, 1).join()
        

    def set_yaw(self, yaw_degree):
        """
        set the yaw angle of the drone
        """
        self.client.rotateToYawAsync(yaw_degree, 5).join()

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
        
        #yaw_degree = math.degrees(pose.orientation.z_val) #angle in degree

        orientation_quat = pose.orientation
        yaw = airsim.to_eularian_angles(orientation_quat)[2] # get the yaw angle
        yaw_degree = math.degrees(yaw)

        
        return [pose.position.x_val, pose.position.y_val, pose.position.z_val, yaw_degree]

    def look_at(self, yaw_degree):
        self.set_yaw(yaw_degree)

    def turn_left(self):
        """
        左转10度
        :return:
        """
        yaw_degree = self.get_yaw()
        yaw_degree = yaw_degree - 10
        self.set_yaw(yaw_degree)


    def turn_right(self):
        """
        右转10度
        :return:
        """
        yaw_degree = self.get_yaw()
        yaw_degree = yaw_degree + 10
        self.set_yaw(yaw_degree)

    def forward(self):
        """
        向前移动1米, 太少了不动
        :return:
        """
        step_length = 1
        cur_position = self.get_drone_position()
        yaw_degree = cur_position[3]
        #将角度转换为弧度
        yaw = math.radians(yaw_degree)
        #向前移动0.1米
        x = cur_position[0] + step_length*math.cos(yaw)
        y = cur_position[1] + step_length*math.sin(yaw)
        z = cur_position[2]
        self.fly_to([x, y, z])
        

    def reset(self):
        self.client.reset()

    def cv2_to_base64(self, image, format='.png'):
        """将 OpenCV 内存中的 numpy 数组转为 Base64 字符串"""
        # 编码为字节流
        success, buffer = cv2.imencode(format, image)
        if not success:
            raise ValueError("图片编码失败，请检查格式参数")
        
        # 转换为 Base64
        img_bytes = buffer.tobytes()
        return base64.b64encode(img_bytes).decode('utf-8')

    def get_image(self):
        """
        获得前置摄像头渲染图像
        :return:
        """
        camera_name = '0'  # 前向中间	0,底部中间 3
        image_type = airsim.ImageType.Scene  # 彩色图airsim.ImageType.Scene, Infrared
        response = self.client.simGetImage(camera_name, image_type, vehicle_name='')  # simGetImage接口的调用方式如下


        img_bgr = cv2.imdecode(np.array(bytearray(response), dtype='uint8'), cv2.IMREAD_UNCHANGED)  # 从二进制图片数据中读
        img = cv2.cvtColor(img_bgr, cv2.COLOR_RGBA2RGB)  # 4通道转3

        #print("image shape:", img.shape)
        return img
    
    def look(self):
        """
        获得前置摄像头渲染图像,并给出图像中主要物体列表
        :return:字符串，目标名称用英文逗号分隔
        """
        #step 1，读取摄像头图片，已经是RGB的了
        rgb_image = self.get_image()
        
        #转成base64格式的png图片
        base64_str = self.cv2_to_base64(rgb_image, ".png")  # png或 '.jpg'

        #step 2，进行图片理解
        # Image input:
        response = self.llm_client.chat.completions.create(
            model="doubao-1-5-vision-pro-32k-250115",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "图片中有哪些目标，请给出名称即可，给出常见的，清晰可见的目标即可，多个目标名称之间用英文逗号分隔"},
                        {
                            "type": "image_url",
                            "image_url": {
                                # "url": "https://yt-shanghai.tos-cn-shanghai.volces.com/tello.jpg"
                                # 使用Base64编码的本地图片，注意img/png， img/jpg不能错
                                "url": f"data:image/png;base64,{base64_str}"
                            }
                        },
                    ],
                }
            ],
            temperature=0.01
        )

        content = response.choices[0].message.content
        return content

    def detect(self, object_name):
        """
        在图像 img 上运行对象检测模型，并返回两个变量 - obj_list，它是场景中检测到的对象名称的列表。obj_locs，每个对象在图像中的边界框坐标列表。
        :param object_name:
        :return:obj_id_list,obj_locs,img_with_box #带box的图片
        """
        #step 1，读取摄像头图片，已经是RGB的了
        rgb_image = self.get_image()

        #直接使用cv图片win下有bug
        # 生成随机文件名（含扩展名）
        file_name = f"random_{uuid.uuid4().hex}.png"  # 示例输出：random_1a2b3c4d5e.png
        cv2.imwrite(file_name, rgb_image)
        
        #step2, 目标检测
        gdino = GroundingDINOAPIWrapper(gdino_token) #使用自己的token
        prompts = dict(image=file_name, prompt=object_name)
        result = gdino.inference(prompts)
        
        #step3, 转换成需要的格式
        obj_id_list = result["categorys"]
        #[xmin, ymin, xmax, ymax]
        obj_locs = result["boxes"]
        
        #画框
        image_pil = Image.open(prompts['image'])
        img_with_box = visualize(image_pil, result)
        return obj_id_list, obj_locs,img_with_box
    

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
    aw = AirSimWrapper()
    aw.takeoff()
    object = aw.look()
    print(object)
    print("done")
