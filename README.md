This is a ros bag including raw data of images, imu and point cloud.
```
xxxxxxxxxxxxx$ rosbag info 2025-01-22-11-16-35.bag 
path:        2025-01-22-11-16-35.bag
version:     2.0
duration:    23:57s (1437s)
start:       Jan 22 2025 11:16:35.79 (1737515795.79)
end:         Jan 22 2025 11:40:32.98 (1737517232.98)
size:        119.5 GB
messages:    718585
compression: none [79712/79712 chunks]
types:       sensor_msgs/CompressedImage [8f7a12909da2c9d3332d540a0977563f]
             sensor_msgs/Imu             [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/PointCloud2     [1158d486dd51d683ce2f1be655c3c181]
topics:      /camera/image_color/compressed    14381 msgs    : sensor_msgs/CompressedImage
             /hikcamera/image_0/compressed     14371 msgs    : sensor_msgs/CompressedImage
             /hikcamera/image_1/compressed     14372 msgs    : sensor_msgs/CompressedImage
             /hikcamera/image_2/compressed     14372 msgs    : sensor_msgs/CompressedImage
             /hikcamera/image_3/compressed     14371 msgs    : sensor_msgs/CompressedImage
             /hikcamera/image_4/compressed     14371 msgs    : sensor_msgs/CompressedImage
             /hikcamera/image_5/compressed     14371 msgs    : sensor_msgs/CompressedImage
             /hikcamera/image_6/compressed     14371 msgs    : sensor_msgs/CompressedImage
             /imu/data                        574875 msgs    : sensor_msgs/Imu            
             /lidar_points                     14371 msgs    : sensor_msgs/PointCloud2    
             /velodyne_points                  14359 msgs    : sensor_msgs/PointCloud2

```

The code is used to extract raw data from this ros bag.
Before run this code, you need to install ros 1 at a Ubuntu 20.04 PC.
