#!/usr/bin/env python3
import os
import rosbag
import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2

def extract_lidar_data(bag_path, topic, output_folder, prefix):
    """
    从 rosbag 文件中提取点云数据，并保存为 .pcd 文件。
    :param bag_path: rosbag 文件路径
    :param topic: 需要提取的 topic（/lidar_points 或 /velodyne_points）
    :param output_folder: 输出文件夹路径
    :param prefix: 输出文件名前缀（LIDAR_TOP 或 LIDAR_FRONT）
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    bag = rosbag.Bag(bag_path, 'r')
    
    for topic, msg, t in bag.read_messages(topics=[topic]):
        timestamp = t.to_nsec()  # 以纳秒为单位的时间戳
        filename = os.path.join(output_folder, f"{prefix}_{timestamp}.pcd")
        
        # 解析点云数据
        point_cloud = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        
        # 保存点云数据到 PCD 文件
        with open(filename, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z intensity\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
            f.write(f"WIDTH {msg.width}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {msg.width}\n")
            f.write("DATA ascii\n")
            for p in point_cloud:
                f.write(f"{p[0]} {p[1]} {p[2]} {p[3]}\n")
        
        print(f"Saved: {filename}")
    
    bag.close()

if __name__ == "__main__":
    bag_file = "2025-01-22-11-16-35.bag"  # 你的 bag 文件
    extract_lidar_data(bag_file, "/lidar_points", "./data/LIDAR_TOP", "LIDAR_TOP")
    extract_lidar_data(bag_file, "/velodyne_points", "./data/LIDAR_FRONT", "LIDAR_FRONT")
