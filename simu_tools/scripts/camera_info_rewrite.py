#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
Sync timestamps from images and camera_info topics

author: Junchuan Zhang


"""
import rospy
import rosbag
import time
# input file name:
bag_file_name = "ablock_dataset5.bag"
# rostopic names in this file：
image_topics = ["/airsim_node/drone_1/centercamera_1/Scene","/airsim_node/drone_1/centercamera_2/DepthPerspective"]
topic_pairs = {"/airsim_node/drone_1/centercamera_1/Scene/camera_info":"/airsim_node/drone_1/centercamera_1/Scene","/airsim_node/drone_1/centercamera_2/DepthPerspective/camera_info":"/airsim_node/drone_1/centercamera_2/DepthPerspective"}
# to convert depth images to point cloud, we need the /tf:
other_topics = ["/airsim_node/drone_1/centercamera_1/Scene/camera_info","/airsim_node/drone_1/centercamera_2/DepthPerspective/camera_info","/airsim_node/drone_1/imu/Imu","/airsim_node/drone_1/odom_local_enu","/tf","/tf_static"]
# other_topics = ["/airsim_node/drone_1/centercamera_1/Scene/camera_info","/airsim_node/drone_1/centercamera_2/DepthPerspective/camera_info"]
# output file name：
bag_output_name = "ablock_dataset5_m.bag"

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
    # remove the empty images:
    if (t.secs>0) and (t.nsecs>0):
        output_handle.write(topic,msg,t=msg.header.stamp)
    else:
        print("Catched!")
    return t

image_ts = {}
image_index = {}
for image_key in image_topics:
    # timestamps from the image topics:
    image_ts[image_key]=[]
    # index in image topics, to iterate in the following step: 
    image_index[image_key]=0

# Output the image topics, and record their timestamps:
for image_topic, image_frame, _ in bag_handle.read_messages(image_topics):
    image_ts[image_topic].append(image_topic_process(image_frame,image_topic,output_handle))

### go through all the others topics：
for topic, msg, _ in bag_handle.read_messages(other_topics):
    if topic==other_topics[0] or topic==other_topics[1]:   ### if camera info topics:
        if image_index[topic_pairs[topic]]>len(image_ts[topic_pairs[topic]])-1:
            image_index[topic_pairs[topic]]=len(image_ts[topic_pairs[topic]])-1
        # print(image_ts[topic_pairs[topic]][image_index[topic_pairs[topic]]])
        # use the image timestamps instead of the camera info timestamps:
        msg.header.stamp = image_ts[topic_pairs[topic]][image_index[topic_pairs[topic]]]
        output_handle.write(topic,msg,t=image_ts[topic_pairs[topic]][image_index[topic_pairs[topic]]])
        image_index[topic_pairs[topic]]+=1
    elif topic==other_topics[-1] or topic==other_topics[-2]:  ### if /tf topics: transforms[0]
        output_handle.write(topic,msg,t=msg.transforms[0].header.stamp)
    else:
        output_handle.write(topic,msg,t=msg.header.stamp)
output_handle.close()
time_end=time.time()
print('time cost',time_end-time_start,'s')