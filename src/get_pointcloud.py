#! /usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import os
import cv2
import open3d as o3d
import pandas as pd
from geometry_msgs.msg._Point import Point

path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
color_intrinsics = np.array([[462.1379699707031, 0.0, 320.0], [0.0, 462.1379699707031, 240.0], [0.0, 0.0, 1.0]])
depth_intrinsics = np.array([[695.9951171875,0.0,640.0],[0.0,695.9951171875,360.0],[0.0,0.0,1.0]])
inv_color_intrinsics = np.linalg.inv(color_intrinsics)
extrinsics = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
transition = [0.017, 0.0132, 0]

rospy.init_node("post_object_3D_position")

bridge = CvBridge()
image_msg = rospy.wait_for_message("/d435/color/image_raw", Image, True)
color_image = bridge.imgmsg_to_cv2(image_msg, "rgb8")
depth_msg = rospy.wait_for_message("/d435/depth/image_raw", Image, True)
depth_image = bridge.imgmsg_to_cv2(depth_msg, "32FC1")


# 将深度图转换为点云
def depth_to_point_cloud(depth_img, intrinsics):
    h, w = depth_img.shape
    i, j = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_img / 1000.0  # 假设深度值以毫米为单位，转换为米
    x = (i - intrinsics[0, 2]) * z / intrinsics[0, 0]
    y = (j - intrinsics[1, 2]) * z / intrinsics[1, 1]
    points = np.stack((x, y, z), axis=-1)
    points = points.reshape(-1, 3)
    return points

# 生成彩色点云
def create_colored_point_cloud(depth_img, color_img, intrinsics_depth, intrinsics_color, extrinsics):
    # 转换深度图为点云
    points_3d = depth_to_point_cloud(depth_img, intrinsics_depth)
    
    # 将点云从深度相机坐标系转换到彩色相机坐标系
    points_3d_homogeneous = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))
    points_3d_transformed = points_3d_homogeneous @ extrinsics.T
    points_3d_transformed = points_3d_transformed[:, :3]
    
    # 将彩色图对齐到点云
    points_2d = points_3d_transformed @ intrinsics_color.T
    points_2d = points_2d[:, :2] / points_3d_transformed[:, 2:3]
    points_2d = points_2d.astype(int)
    
    # 将颜色赋值给点云
    colors = np.zeros_like(points_3d)
    valid_points = (0 <= points_2d[:, 0]) & (points_2d[:, 0] < color_img.shape[1]) & \
                   (0 <= points_2d[:, 1]) & (points_2d[:, 1] < color_img.shape[0])
    
    colors[valid_points] = color_img[points_2d[valid_points, 1], points_2d[valid_points, 0]] / 255.0
    
    # 使用Open3D生成彩色点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_3d)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

# 生成彩色点云
pcd = create_colored_point_cloud(depth_image, color_image, depth_intrinsics, color_intrinsics, extrinsics)

# 显示彩色点云
o3d.visualization.draw_geometries([pcd])


