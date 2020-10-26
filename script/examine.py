import cv2
import numpy as np

src_rgb = cv2.imread("/home/shapelim/hdd2/sw_gpslam_rosbag/3f/rgb/117.png", -1)
print(src_rgb.dtype)

src_16UC1 = cv2.imread("/home/shapelim/hdd2/sw_gpslam_rosbag/3f/depth/117.png", -1)
print(src_16UC1.dtype)

src_32FC1 = cv2.imread("/home/shapelim/hdd2/sw_gpslam_rosbag/3f/depth/117_debug.png", -1)
print(src_32FC1.dtype)

