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
from smolagents import tool
from typing import List,Tuple



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


#AirSimWrapper
client = airsim.MultirotorClient()#run in some machine of airsim,otherwise,set ip="" of airsim

#大模型client
llm_client = OpenAI(
            api_key=api_key,
            base_url="https://ark.cn-beijing.volces.com/api/v3",
            )


@tool
def takeoff() -> str:
    """
    起飞无人机。返回为字符串，表示动作是否成功。

    Returns:
        str: 成功状态描述
    """
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()

    return "成功"
    
@tool
def land() -> str:
    """
    降落无人机。返回为字符串，表示动作是否成功。

    Returns:
        str: 成功状态描述
    """
    client.landAsync().join()

    return "成功"

@tool
def get_drone_position()->Tuple[float, float, float, float]:
    """
    获取无人机当前位置和偏航角
    Return:
        Tuple[x, y, z, yaw_degree]: 包含三维坐标（x/y/z）和偏航角（角度制）的元组
    """
    pose = client.simGetVehiclePose()
    yaw_degree = get_yaw()  # angle in degree
    return [pose.position.x_val, pose.position.y_val, pose.position.z_val,yaw_degree]

@tool
def fly_to(point: Tuple[float,float,float,float]) -> str:
    """
    fly the drone to a specific point
    
    Args:
        point:Tuple[x, y, z, yaw_degree]: 目标点，包含三维坐标（x/y/z）和偏航角（角度制）的元组
    """
    if point[2] > 0:
        client.moveToPositionAsync(point[0], point[1], -point[2], 1).join()
    else:
        client.moveToPositionAsync(point[0], point[1], point[2], 1).join()

    return "成功"

        
@tool
def fly_path(points: List[Tuple[float, float, float,float]]) -> str:
    """
    fly the drone along a specific path
    
    Args:
        points: 路径点列表，每个点为三维坐标 (x, y, z)和偏航角（角度制）的元组

    Returns:
        str: 成功状态描述
    """
    airsim_points = []
    for point in points:
        if point[2] > 0:
            airsim_points.append(airsim.Vector3r(point[0], point[1], -point[2]))
        else:
            airsim_points.append(airsim.Vector3r(point[0], point[1], point[2]))
    client.moveOnPathAsync(airsim_points, 1).join()
    return "成功"
    
@tool
def set_yaw(yaw_degree: float) -> str:
    """
    设置无人机的偏航角
    
    Args:
        yaw_degree: 无人机偏航角（角度制）

    Returns:
        str: 成功状态描述
    """
    client.rotateToYawAsync(yaw_degree, 5).join()
    
    return "成功"

@tool
def get_yaw()->float:
    """
    get the yaw angle of the drone

    Returns:
        float: yaw_degree, the yaw angle of the drone in degree
    """
    orientation_quat = client.simGetVehiclePose().orientation
    yaw = airsim.to_eularian_angles(orientation_quat)[2] # get the yaw angle
    yaw_degree = math.degrees(yaw)
    return yaw_degree # return the yaw angle in degree

@tool
def get_position(object_name: str)-> Tuple[float,float,float,float]:
    """
    get the position of a specific object
    
    Args:
        object_name: the name of the object
        
    Returns: 
        Tuple[float,float,float,float]: position, the position of the object,点为三维坐标 (x, y, z)和偏航角（角度制）的元组
    """
    query_string = objects_dict[object_name] + ".*"
    object_names_ue = []
    while len(object_names_ue) == 0:
        object_names_ue = client.simListSceneObjects(query_string)
    pose = client.simGetObjectPose(object_names_ue[0])
    
    #yaw_degree = math.degrees(pose.orientation.z_val) #angle in degree

    orientation_quat = pose.orientation
    yaw = airsim.to_eularian_angles(orientation_quat)[2] # get the yaw angle
    yaw_degree = math.degrees(yaw)

    
    return [pose.position.x_val, pose.position.y_val, pose.position.z_val, yaw_degree]

@tool
def look_at(yaw_degree: float)->str:
    """
    设置无人机的朝向

    Args:
        yaw_degree: 偏航角（角度制）

    Returns:
        str: 成功状态描述
    """
    set_yaw(yaw_degree)
    return "成功"
    

@tool
def turn_left()->str:
    """
    左转, 10度

    Returns:
        str: 成功状态描述
    """
    yaw_degree = get_yaw()
    yaw_degree = yaw_degree - 10
    set_yaw(yaw_degree)
    return "成功"

@tool
def turn_right()->str:
    """
    右转, 10度

    Returns:
        str: 成功状态描述
    """
    yaw_degree = get_yaw()
    yaw_degree = yaw_degree + 10
    set_yaw(yaw_degree)
    return "成功"

@tool
def forward()->str:
    """
    向前移动1米, 太少了不动

    Returns:
        str: 成功状态描述
    """
    step_length = 1
    cur_position = get_drone_position()
    yaw_degree = cur_position[3]
    #将角度转换为弧度
    yaw = math.radians(yaw_degree)
    #向前移动0.1米
    x = cur_position[0] + step_length*math.cos(yaw)
    y = cur_position[1] + step_length*math.sin(yaw)
    z = cur_position[2]
    fly_to([x, y, z])
    return "成功"
    

def reset():
    client.reset()


def cv2_to_base64(image, format='.png'):
    """将 OpenCV 内存中的 numpy 数组转为 Base64 字符串"""
    # 编码为字节流
    success, buffer = cv2.imencode(format, image)
    if not success:
        raise ValueError("图片编码失败，请检查格式参数")
    
    # 转换为 Base64
    img_bytes = buffer.tobytes()
    return base64.b64encode(img_bytes).decode('utf-8')


def get_image():
    """
    获得前置摄像头渲染图像
    :return:
    """
    camera_name = '0'  # 前向中间	0,底部中间 3
    image_type = airsim.ImageType.Scene  # 彩色图airsim.ImageType.Scene, Infrared
    response = client.simGetImage(camera_name, image_type, vehicle_name='')  # simGetImage接口的调用方式如下


    img_bgr = cv2.imdecode(np.array(bytearray(response), dtype='uint8'), cv2.IMREAD_UNCHANGED)  # 从二进制图片数据中读
    img = cv2.cvtColor(img_bgr, cv2.COLOR_RGBA2RGB)  # 4通道转3

    #print("image shape:", img.shape)
    return img

@tool
def look()->str:
    """
    获得前置摄像头渲染图像,并给出图像中主要物体列表
    
    Return:
        str: 目标名称用英文逗号分隔
    """
    #step 1，读取摄像头图片，已经是RGB的了
    rgb_image = get_image()
    
    #转成base64格式的png图片
    base64_str = cv2_to_base64(rgb_image, ".png")  # png或 '.jpg'

    #step 2，进行图片理解
    # Image input:
    response = llm_client.chat.completions.create(
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

@tool
def detect(object_name: str)->Tuple[List[str], List[List[float]]]:
    """
    在图像上运行目标检测模型，返回检测结果及标记框图像
    
    Args:
        object_name: 需要查找的目标名称，注意这个函数输入的目标名称object_name只能是英文，如果需要搜索的名称是中文，则需要翻译一下
        
    Returns:
        Tuple[List[str], List[List[float]]]:
            - 检测到的对象名称列表
            - 每个对象的边界框坐标列表（格式：[xmin, ymin, xmax, ymax]）
            - 带标记框的PIL图像对象
    """
    #step 1，读取摄像头图片，已经是RGB的了
    rgb_image = get_image()

    #直接使用cv图片win下有bug
    # 生成随机文件名（含扩展名）
    file_name = f"random_{uuid.uuid4().hex}.png"  # 示例输出：random_1a2b3c4d5e.png
    cv2.imwrite(file_name, rgb_image)
    
    #step2, 目标检测
    gdino = GroundingDINOAPIWrapper(gdino_token) #使用自己的token
    prompts = dict(image=file_name, prompt=object_name)
    result = gdino.inference(prompts)
    
    #step3, 转换成需要的格式
    #[obj1, obj2,...]
    obj_id_list = result["categorys"]
    
    #[xmin, ymin, xmax, ymax]
    obj_locs = result["boxes"]
    
    #画框
    # image_pil = Image.open(prompts['image'])
    # img_with_box = visualize(image_pil, result)
    return obj_id_list, obj_locs

def detect_with_img(object_name):
    """
    在图像上运行目标检测模型，返回检测结果及标记框图像
    
    Args:
        object_name: 需要查找的目标名称，注意这个函数输入的目标名称object_name只能是英文，如果需要搜索的名称是中文，则需要翻译一下
        
    Returns:
        Tuple[List[str], List[List[float]]]:
            - 检测到的对象名称列表
            - 每个对象的边界框坐标列表（格式：[xmin, ymin, xmax, ymax]）
            - 带标记框的PIL图像对象
    """
    #step 1，读取摄像头图片，已经是RGB的了
    rgb_image = get_image()

    #直接使用cv图片win下有bug
    # 生成随机文件名（含扩展名）
    file_name = f"random_{uuid.uuid4().hex}.png"  # 示例输出：random_1a2b3c4d5e.png
    cv2.imwrite(file_name, rgb_image)
    
    #step2, 目标检测
    gdino = GroundingDINOAPIWrapper(gdino_token) #使用自己的token
    prompts = dict(image=file_name, prompt=object_name)
    result = gdino.inference(prompts)
    
    #step3, 转换成需要的格式
    #[obj1, obj2,...]
    obj_id_list = result["categorys"]
    
    #[xmin, ymin, xmax, ymax]
    obj_locs = result["boxes"]
    
    #画框
    image_pil = Image.open(prompts['image'])
    img_with_box = visualize(image_pil, result)
    return obj_id_list, obj_locs,img_with_box


@tool
def ob_objects(obj_name_list:List[str])-> List[Tuple[str, float, float]]:
    """
    对无人机获得的图像进行目标定位，获得目标列表 [ (对象名称、距离、角度（以度为单位）),...]

    Args:
        obj_name_list: 目标名称列表，必须是英文，如果输入的是中文，请先翻译

    Returns:
        List: [(对象名称、和无人机的距离、和无人机的角度（以度为单位）>,...]
    """
    
    #step1, 目标检测
    prompt = ".".join(obj_name_list)
    # obj_id_list: [obj1, obj2,...], obj_locs: [[xmin, ymin, xmax, ymax],[xmin, ymin, xmax, ymax],...]
    obj_id_list, obj_locs = detect(prompt)

    #step2, 获得深度视觉数据
    responses = client.simGetImages([
        # png format
        airsim.ImageRequest(0, airsim.ImageType.Scene, pixels_as_float=False, compress=True),
    
        # floating point uncompressed image，深度图, 像素点代表到相平面距离
        airsim.ImageRequest(0, airsim.ImageType.DepthPlanar, pixels_as_float=True),
    
        # 像素点代表的到相机的距离
        airsim.ImageRequest(0, airsim.ImageType.DepthPerspective, pixels_as_float=True)
      ]
    )
    
    img_depth_planar = np.array(responses[1].image_data_float).reshape(responses[1].height, responses[0].width)
    img_depth_perspective = np.array(responses[2].image_data_float).reshape(responses[2].height, responses[1].width)

    #一般图片
    image_data = responses[0].image_data_uint8
    img = cv2.imdecode(np.array(bytearray(image_data), dtype='uint8'), cv2.IMREAD_UNCHANGED)  # 从二进制图片数据中读
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)  # 4通道转3
    

    final_obj_list = [] #最终结果列表
    #构建目标结果
    index = 0
    for bbox in obj_locs:
        center_x = int((bbox[0] + bbox[2]) / 2)
        center_y = int((bbox[1] + bbox[3]) / 2)

        depth_distance = img_depth_planar[center_y, center_x, ] #相平面距离
        camera_distance = img_depth_perspective[center_y, center_x] #相机距离

        #求角度
        angel = math.acos(depth_distance / camera_distance)
        angel_degree = math.degrees(angel)

        # 判断正负，左边为正，右边为负，只看偏航角
        if center_x < img.shape[1] / 2:
            # 如果目标在图像的左侧，向左转，degree 为负数
            angel_degree = -1 * angel_degree

        obj_name = obj_id_list[index]#获得目标名称，可能有多个

        obj_info = (obj_name, camera_distance, angel_degree)
        final_obj_list.append(obj_info)
        index = index + 1

    return final_obj_list
    
@tool
def watch(prompt:str)->str:
    """
    获得前置摄像头渲染图像,并根据提示词回答问题

    Args:
        prompt: 提示词
    
    Return:
        str: 目标名称用英文逗号分隔
    """
    #step 1，读取摄像头图片，已经是RGB的了
    rgb_image = get_image()
    
    #转成base64格式的png图片
    base64_str = cv2_to_base64(rgb_image, ".png")  # png或 '.jpg'

    #step 2，进行图片理解
    # Image input:
    response = llm_client.chat.completions.create(
        model="doubao-1-5-vision-pro-32k-250115",
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
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

@tool
def turn(angle: float)->str:
    """
    无人机旋转angle角度

    Args:
        angle: 无人机需要旋转的角度（以度为单位）
        
    Returns:
        str: 成功状态描述
    """
    yaw_degree = get_yaw()
    yaw_degree = yaw_degree + angle
    set_yaw(yaw_degree)
    return "成功"

@tool
def move(distance: float)->str:
    """
    向前移动distance米的距离

    Args:
        distance: 无人机向前移动的距离，单位为米
        
    Returns:
        str: 成功状态描述
    """
    step_length = distance
    cur_position = get_drone_position()
    yaw_degree = cur_position[3]
    #将角度转换为弧度
    yaw = math.radians(yaw_degree)
    #向前移动0.1米
    x = cur_position[0] + step_length*math.cos(yaw)
    y = cur_position[1] + step_length*math.sin(yaw)
    z = cur_position[2]
    fly_to([x, y, z, 0])
    return "成功"
    
if __name__ == "__main__":
    takeoff()
    object = look()
    print(object)
    print("done")
