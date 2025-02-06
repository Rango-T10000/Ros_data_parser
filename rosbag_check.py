#------------------check camera/image_color/compressed,这是车顶部向上的相机器----------------------------
# import rosbag
# from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# bag_path = "2025-01-22-11-16-35.bag"
# bag = rosbag.Bag(bag_path, "r")
# bridge = CvBridge()

# for topic, msg, t in bag.read_messages(topics=["/camera/image_color/compressed"]):
#     try:
#         img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
#         cv2.imshow("Image", img)
#         cv2.waitKey(0)  # 按任意键继续
#     except Exception as e:
#         print(f"Error decoding image: {e}")
#     # break  # 只查看第一帧
# bag.close()
# cv2.destroyAllWindows()


#----------------------打印查看IMU数据--------------------
#!/usr/bin/env python3
import rosbag
from sensor_msgs.msg import Imu
import numpy as np

def print_imu_data(msg, timestamp):
    """打印 IMU 数据"""
    print(f"Timestamp: {timestamp} ns")
    print(f"  Linear Acceleration (m/s²): x={msg.linear_acceleration.x:.6f}, "
          f"y={msg.linear_acceleration.y:.6f}, z={msg.linear_acceleration.z:.6f}")
    print(f"  Angular Velocity (rad/s): x={msg.angular_velocity.x:.6f}, "
          f"y={msg.angular_velocity.y:.6f}, z={msg.angular_velocity.z:.6f}")
    print(f"  Orientation (quaternion): w={msg.orientation.w:.6f}, x={msg.orientation.x:.6f}, "
          f"y={msg.orientation.y:.6f}, z={msg.orientation.z:.6f}")
    print("-" * 60)

def main():
    bag_path = "2025-01-22-11-16-35.bag"  # ROS bag 文件路径
    bag = rosbag.Bag(bag_path, "r")

    count = 0
    for topic, msg, t in bag.read_messages(topics=["/imu/data"]):
        timestamp = t.to_nsec()  # 时间戳（纳秒）
        print_imu_data(msg, timestamp)
        
        count += 1
        if count >= 10:  # 只打印前 10 条数据，防止输出过多
            break

    bag.close()

if __name__ == '__main__':
    main()


