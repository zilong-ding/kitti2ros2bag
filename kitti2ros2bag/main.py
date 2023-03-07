#!/usr/bin/env python3
#coding=utf-8

import sys
try:
    import pykitti
except ImportError as e:
    print('Could not load module \'pykitti\'. Please run `pip install pykitti`')
    sys.exit(1)

import rclpy
from rclpy.node import Node
import ros2bag
import cv2
import os
import numpy as np
import argparse
from sensor_msgs.msg import Image, CameraInfo,Imu
from std_msgs.msg import Header
# import sensor_msgs.msg._point_cloud2 as pcl2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
# from sensor_msgs_py import point_cloud2
import time
data_path = '/home/xzb/ros2-project/kitti2ros2bag/src/kitti2ros2bag/datasets/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/'
data_path ='/home/xzb/ros2-project/kitti2ros2bag/src/kitti2ros2bag/datasets/2011_09_26_drive_0019_sync/2011_09_26/2011_09_26_drive_0019_sync/'

imu_topic = 'kitti/imu'
imu_dir = 'oxts/data/'
imu_timestamps = 'oxts/timestamps.txt'

lidar_topic = '/kitti/lidar'
lidar_dir = 'velodyne_points/data/'
lidar_timestamps = 'velodyne_points/timestamps.txt'

cameras = ['/kitti/camera_gray_left','/kitti/camera_gray_right','/kitti/camera_color_left', '/kitti/camera_color_right']
cameras_dir = ['image_00/data/','image_01/data/','image_02/data/','image_03/data/']
cameras_timestamps = ['image_00/timestamps.txt','image_01/timestamps.txt','image_02/timestamps.txt','image_03/timestamps.txt']

def string2timestamp(name):
    time_list = name.split(' ')[1]
    time = time_list.split(':')[2]
    sec,nanosec = time.split('.')
    sec = int(sec)
    nanosec = int(nanosec)
    return sec,nanosec

class kitti2bag2(Node):
    def __init__(self):
        super().__init__('kitti2bag2')
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)
        self.cameras = [None] * len(cameras)
        for i in range(len(cameras)):
            self.cameras[i] = self.create_publisher(Image, cameras[i], 10)
        self.lidar_pub = self.create_publisher(PointCloud2, lidar_topic, 10)
        
    # def read_gray_image(self,image_path,timestamps,pub):
    #     image = cv2.imread(image_path)
    #     bridge = CvBridge()
    #     image_msg = Image()
    #     image_msg = bridge.cv2_to_imgmsg(image, encoding="mono8")
    #     image_msg.header = Header()
    #     image_msg.header.stamp = rclpy.time.Time.from_msg(timestamps)
    #     pub.publish(image_msg)
        
    def read_color_image(self,image_path,timestamps,pub,frame_id):
        image = cv2.imread(image_path)
        bridge = CvBridge()
        image_msg = Image()
        image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_msg.header = Header()
        image_msg.header.stamp.sec, image_msg.header.stamp.nanosec = string2timestamp(timestamps)
        image_msg.header.frame_id = frame_id
        pub.publish(image_msg)
        
    def read_imu(self,imu_path,timestamps,pub):
        imu = np.loadtxt(imu_path)
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec = string2timestamp(timestamps)
        imu_msg.linear_acceleration.x = imu[0]
        imu_msg.linear_acceleration.y = imu[1]
        imu_msg.linear_acceleration.z = imu[2]
        imu_msg.angular_velocity.x = imu[3]
        imu_msg.angular_velocity.y = imu[4]
        imu_msg.angular_velocity.z = imu[5]
        pub.publish(imu_msg)
        
    def read_lidar(self,lidar_path,timestamps,pub):
        lidar = np.fromfile(lidar_path, dtype=np.float32).reshape(-1, 4)[:,:3]
        lidar_msg = PointCloud2()
        lidar_msg.header.stamp.sec, lidar_msg.header.stamp.nanosec = string2timestamp(timestamps)
        lidar_msg.header.frame_id = 'velodyne'
        lidar_msg.height = 1
        lidar_msg.width = len(lidar)
        x = PointField()
        x.name = 'x'
        x.offset = 0
        x.datatype = PointField.FLOAT32
        x.count = 1
        y = PointField()
        y.name = 'y'
        y.offset = 4
        y.datatype = PointField.FLOAT32
        y.count = 1
        z = PointField()
        z.name = 'z'
        z.offset = 8
        z.datatype = PointField.FLOAT32
        z.count = 1
        lidar_msg.fields = [x,y,z]
        lidar_msg.is_bigendian = False
        lidar_msg.point_step = 12
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width
        lidar_msg.is_dense = True
        lidar_msg.data = np.asarray(lidar, dtype=np.float32).tostring()
        pub.publish(lidar_msg)
        
    def run(self):
        time.sleep(1)
        left_gray_dir = data_path + cameras_dir[0]
        right_gray_dir = data_path + cameras_dir[1]
        left_color_dir = data_path + cameras_dir[2]
        right_color_dir = data_path + cameras_dir[3]
        
        with open(data_path + cameras_timestamps[0],'r') as f:
            left_gray_timestamps_list = f.readlines()
        with open(data_path + cameras_timestamps[1],'r') as f:
            right_gray_timestamps_list = f.readlines()
        with open(data_path + cameras_timestamps[2],'r') as f:
            left_color_timestamps_list = f.readlines()
        with open(data_path + cameras_timestamps[3],'r') as f:
            right_color_timestamps_list = f.readlines()
        
        
        
        
        
        imu_datas = data_path + imu_dir
        with open(data_path + imu_timestamps,'r') as f:
            imu_timestamps_list = f.readlines()
        lidar_datas = data_path + lidar_dir
        with open(data_path + lidar_timestamps,'r') as f:
            lidar_timestamps_list = f.readlines()
        for i in range(len(imu_timestamps_list)):
            print(i)
            left_gray = left_gray_dir + str(i).zfill(10) + '.png'
            right_gray = right_gray_dir + str(i).zfill(10) + '.png'
            left_color = left_color_dir + str(i).zfill(10) + '.png'
            right_color = right_color_dir + str(i).zfill(10) + '.png'
            gray_left_timestamp = left_gray_timestamps_list[i].rstrip("\n")
            gray_right_timestamp = right_gray_timestamps_list[i].rstrip("\n")
            color_left_timestamp = left_color_timestamps_list[i].rstrip("\n")
            color_right_timestamp = right_color_timestamps_list[i].rstrip("\n")
            
            imu_path = imu_datas + str(i).zfill(10) + '.txt'
            imu_timestamp = imu_timestamps_list[i].rstrip("\n")
            
            lidar_path = lidar_datas + str(i).zfill(10) + '.bin'
            lidar_timestamp = lidar_timestamps_list[i].rstrip("\n")
            
            self.read_color_image(left_gray,gray_left_timestamp,self.cameras[0],'camera_left')
            self.read_color_image(right_gray,gray_right_timestamp,self.cameras[1],'camera_right')
            self.read_color_image(left_color,color_left_timestamp,self.cameras[2],'camera_left')
            self.read_color_image(right_color,color_right_timestamp,self.cameras[3],'camera_right')
            
            self.read_imu(imu_path,imu_timestamp,self.imu_pub)
            self.read_lidar(lidar_path,lidar_timestamp,self.lidar_pub)
            
            
if __name__ == '__main__':
    rclpy.init()
    node = kitti2bag2()
    node.run()
    rclpy.shutdown()
            
            
            
            
        
        


























