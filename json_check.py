#!/usr/bin/env python3
import json
import os

def read_imu_json(json_path):
    """读取 IMU 数据的 JSON 文件，并打印列表中的第一个元素"""
    
    # 检查文件是否存在
    if not os.path.exists(json_path):
        print(f"错误: 文件 {json_path} 不存在！")
        return
    
    # 读取 JSON 文件
    with open(json_path, "r") as f:
        imu_data_list = json.load(f)

    # 检查列表是否为空
    if len(imu_data_list) == 0:
        print("IMU 数据列表为空！")
        return

    # 打印第一个元素
    first_entry = imu_data_list[0]
    print("IMU 数据列表的第一个元素：")
    print(json.dumps(first_entry, indent=4))

if __name__ == "__main__":
    json_path = "./data/2025-01-22-11-16-35_imu.json"  # JSON 文件路径
    read_imu_json(json_path)
