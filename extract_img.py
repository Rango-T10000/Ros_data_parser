#!/usr/bin/env python3
import rospy                   #注意需要在另一个终端窗口输入roscore,退出conda再启动这个脚本,
import rosbag
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import os
from cv_bridge import CvBridge

def save_image(msg, camera_name, timestamp):
    bridge = CvBridge()
    try:
        # 将 ROS 图像消息转换为 OpenCV 图像
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 构造图像的文件路径，按照时间戳命名
        folder_path = f"./data/{camera_name}"
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)  # 创建文件夹

        # 图像文件名使用时间戳
        file_name = f"{folder_path}/{camera_name}_{timestamp}.jpg"
        cv2.imwrite(file_name, cv_image)  # 保存图像
        print(f"Saved {file_name}")
    except Exception as e:
        print(f"Error: {e}")

def main():
    bag_path = "./2025-01-22-11-16-35.bag"  # ROS bag 文件路径
    bag = rosbag.Bag(bag_path, "r")

    # 获取 bag 中所有消息
    for topic, msg, t in bag.read_messages(topics=[
        "/hikcamera/image_0/compressed",
        "/hikcamera/image_1/compressed",
        "/hikcamera/image_2/compressed",
        "/hikcamera/image_3/compressed",
        "/hikcamera/image_4/compressed",
        "/hikcamera/image_5/compressed",
        "/hikcamera/image_6/compressed"]):

        # 获取时间戳
        timestamp = str(t.to_nsec())

        # 根据相机名称调用保存图像的函数
        if topic == "/hikcamera/image_0/compressed":
            save_image(msg, "CAM_BACK_RIGHT", timestamp)     
        elif topic == "/hikcamera/image_1/compressed":
            save_image(msg, "CAM_FRONT", timestamp)
        elif topic == "/hikcamera/image_2/compressed":
            save_image(msg, "CAM_FRONT_2", timestamp)
        elif topic == "/hikcamera/image_3/compressed":
            save_image(msg, "CAM_FRONT_LEFT", timestamp)
        elif topic == "/hikcamera/image_4/compressed":
            save_image(msg, "CAM_FRONT_RIGHT", timestamp)
        elif topic == "/hikcamera/image_5/compressed":
            save_image(msg, "CAM_BACK_LEFT", timestamp)
        elif topic == "/hikcamera/image_6/compressed":
            save_image(msg, "CAM_BACK", timestamp)

    bag.close()

if __name__ == '__main__':
    rospy.init_node('image_extractor', anonymous=True)
    main()
