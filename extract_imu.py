#!/usr/bin/env python3
import rosbag
import json
import os
from sensor_msgs.msg import Imu

def extract_imu_data(bag_path):
    """从 ROS bag 文件中提取 IMU 数据，并保存到 ./data 目录下的 JSON 文件"""
    
    # 解析 bag 文件名，并生成 JSON 文件名
    base_name = os.path.basename(bag_path).replace(".bag", "_imu.json")
    save_dir = "./data"
    
    # 确保保存目录存在
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    json_file_path = os.path.join(save_dir, base_name)
    
    imu_data_list = []

    # 读取 rosbag
    bag = rosbag.Bag(bag_path, "r")
    for topic, msg, t in bag.read_messages(topics=["/imu/data"]):
        imu_data = {
            "timestamp": t.to_nsec(),
            "orientation": {
                "w": msg.orientation.w,
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z
            }
        }
        imu_data_list.append(imu_data)
    
    bag.close()

    # 保存 JSON 文件
    with open(json_file_path, "w") as f:
        json.dump(imu_data_list, f, indent=4)

    print(f"IMU 数据已成功提取并保存到 {json_file_path}")

if __name__ == "__main__":
    bag_path = "2025-01-22-11-16-35.bag"  # 你的 rosbag 文件路径
    extract_imu_data(bag_path)
