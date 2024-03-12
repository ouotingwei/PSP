#!/usr/bin/env python3

import cv2
import os
import sys
import numpy as np
from pupil_apriltags import Detector
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from vision_capture2.srv import VisionCapture, VisionCaptureResponse

global image, tag_rotation, tag_translation, centroid

camera_params = (610.319671, 611.570539, 317.992731, 235.564522)
tag_size = 0.0408
tag_rotation = None
tag_translation = None

bridge = CvBridge()
image = None
cloud_data = None
centroid = None

def image_callback(msg):
    global image
    try:
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("Error processing image: %s" % str(e))

def cloud_callback(msg):
    global cloud_data
    cloud_data = msg

def get_pose():
    global image, tag_rotation, tag_translation
    sample = 0

    if image is not None:
        at_detector = Detector(families='tag36h11')
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray_image, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)

        if len(tags) == 0:
            print('\033[91m' + " [MOD] POSE_ERROR: No APRILTAG detected." + '\033[0m')
            LOGGING(state="   [MOD_ERROR]No APRILTAG detected")
            return None, None

        else:
            for tag in tags:
                if 1 == tag.tag_id:
                    sample = 1
            for tag in tags:
                if 0 == tag.tag_id and sample == 1:
                    tag_rotation = tag.pose_R
                    tag_translation = tag.pose_t
                    print('\033[92m' + " [MOD] POSE OK." + '\033[0m')
                elif 0 == tag.tag_id and sample == 0:
                    print('\033[91m' + " [MOD] POSE_ERROR: No APRILTAG detected." + '\033[0m')
                    LOGGING(state="   [MOD_ERROR]No ")

    #TF(tag_rotation, tag_translation)
    return tag_rotation, tag_translation

def LOGGING(state):
    homeDir = os.getenv("HOME")
    if homeDir is None:
        sys.stderr.write("Failed to get the home directory.\n")

    # file_loc = homeDir + '/PSP/files/logging_file.txt'
    file_loc = '/home/honglang/PSP/logfile/logging_file.txt'

    with open(file_loc, 'a') as file:
        if file.tell() != 0:
            file.write('\n')
        file.write(state)

def TF(rotation, transition):
    print("tf ............")
    homeDir = os.getenv("HOME")
    if homeDir is None:
        sys.stderr.write("Failed to get the home directory.\n")

    if rotation is None or transition is None:
        print("Error: Rotation or translation is None.")
        return

    # file_loc = homeDir + '/PSP/files/TF.txt'
    file_loc = '/home/honglang/PSP/files/TF.txt'

    tf = []
    for i in range(3):
        for j in range(3):
            tf.append(rotation[i][j])
    for i in range(3):
        tf.append(transition[0][i])

    with open(file_loc, 'w') as file:
        for i in range(len(tf)):
            file.write(str(tf[i]))  # Convert to string before writing
            file.write('\n')

def center_point_cloud(point_cloud):
    global centroid
    print(centroid)
    centroid = np.mean(np.asarray(point_cloud.points), axis=0)
    translated_pcd = point_cloud.translate(-centroid)
    return translated_pcd

def save_point_cloud_as_pcd(rotation, translation):
    global cloud_data
    if cloud_data is not None:
        pc_np = pc2.read_points(cloud_data, field_names=("x", "y", "z"), skip_nans=True)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_np)

        pcd.rotate(np.linalg.inv(rotation), center=(0, 0, 0))
        pcd.translate(-1 * np.linalg.inv(rotation) @ translation)

        bounding_box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=[-0.35, 0, -0.1],
            max_bound=[0, 0.22, -0.01]
        )

        cropped_pcd = pcd.crop(bounding_box)

        downsampled_pcd = cropped_pcd.voxel_down_sample(voxel_size=0.005)

        centroid_pcd = center_point_cloud(downsampled_pcd)

        print("tag.transition", tag_translation)
        print("centroid", centroid)
        camera_to_centroid = tag_translation.T + centroid
        # for i in range(3):
        #     camera_to_centroid[i] =  tag_translation[i] + centroid[i]
        print("camera_to_centroid", camera_to_centroid)
        TF(tag_rotation, camera_to_centroid)

        # find the max_x & min_x
        max_x = float('-inf')
        min_x = float('inf')
        for point in centroid_pcd.points:
            x = point[0]  # 第一个元素是 X 座标
            if x > max_x:
                max_x = x
            if x < min_x:
                min_x = x

        x_resolution = 0.005 # resolution = 1cm
        current_x = min_x

        while current_x < max_x:
            loop_min_x = 100
            for tmp_points in centroid_pcd.points:
                if tmp_points[0] < current_x + x_resolution and tmp_points[0] >= current_x:
                    if tmp_points[2] < loop_min_x:
                        loop_min_x = tmp_points[2]
            for tmp_points in centroid_pcd.points:
                if tmp_points[0] < current_x + x_resolution and tmp_points[0] >= current_x:
                    tmp_points[2] = loop_min_x 

            current_x = current_x + x_resolution



        # o3d.io.write_point_cloud( homeDir + "/PSP/files/point_cloud.pcd", centroid_pcd )
        o3d.io.write_point_cloud("/home/honglang/PSP/files/point_cloud.pcd", centroid_pcd)

def capture(req):
    if req.scan == True:
        rotation, translation = get_pose()
        save_point_cloud_as_pcd(rotation, translation)
        return VisionCaptureResponse(True)

def ros_server():
    rospy.init_node('vision_capture')
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, cloud_callback)
    s = rospy.Service('/vision_capture', VisionCapture, capture)
    rospy.spin()

if __name__ == '__main__':
    ros_server()
