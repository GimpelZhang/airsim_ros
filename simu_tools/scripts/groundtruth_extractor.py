#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
按照一定规律过滤rosbag信息，输出tum格式位姿真值文件。

使用了evo包内文件：

author: Junchuan Zhang


"""
import rospy
import rosbag
import time
import sys
import os
sys.path.insert(0,os.path.abspath('./evo/'))
from evo.tools import file_interface
# 输入的rosbag文件名：
bag_file_name = "dataset_1time_2.bag"
# 将要转换的全体rostopic名称：
topics = ["/odom_local_ned"]
# 输出的文件名：
bag_output_name = "pose_record.txt"

bag_handle = rosbag.Bag(bag_file_name)

time_start=time.time()

# 遍历所有的rostopic：
traj_record = file_interface.read_bag_trajectory(bag_handle, topics[0])
file_interface.write_tum_trajectory_file(file_path, traj_record)
time_end=time.time()
print('time cost',time_end-time_start,'s')