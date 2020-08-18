#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
Solve the problem of empty images

author: Junchuan Zhang


"""
import rospy
import rosbag
import time
# input file name:
bag_file_name = "ablock_dataset1.bag"
# rostopic names in this file：
topics = ["/airsim_node/drone_1/leftcamera_1/Scene","/airsim_node/drone_1/rightcamera_1/Scene","/airsim_node/drone_1/imu/Imu","/airsim_node/drone_1/odom_local_enu"]
# output file name：
bag_output_name = "ablock_dataset1_m.bag"

bag_handle = rosbag.Bag(bag_file_name)
output_handle = rosbag.Bag(bag_output_name,'w')
time_start=time.time()

def image_topic_process(msg,topic,output_handle):
    '''
    :param msg: image topic
    :param topic: topic name
    :param output_handle: output file handle
    '''
    t = msg.header.stamp
    if (t.secs>0) and (t.nsecs>0):
        output_handle.write(topic,msg,t=msg.header.stamp)
    else:
        print("Catched!")
    return
### go through all the topics：
for topic, msg, _ in bag_handle.read_messages(topics):
    if (topic==topics[0]) or (topic==topics[1]):
        image_topic_process(msg,topic,output_handle)
    else:
        output_handle.write(topic,msg,t=msg.header.stamp)
output_handle.close()
time_end=time.time()
print('time cost',time_end-time_start,'s')