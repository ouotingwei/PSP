import cv2
import numpy as np
from pupil_apriltags import Detector
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl
import open3d as o3d

class Detection: 
    def __init__(self):
        self.camera_params = (623.6932761844039, 624.1571561352914, 329.01981432231025, 235.02669920374768)
        self.tag_size = 0.059
        self.tag_rotation = None
        self.tag_translation = None

        
        self.loop_rate = rospy.Rate(1)
        self.bridge = CvBridge()
        self.image = None
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            rospy.logerr("CvBridgeError:")

    def get_pose(self):
        if self.image is not None:
            at_detector = Detector(families='tag36h11')
            gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            tags = at_detector.detect(gray_image, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size)

            for tag in tags:
                if 0 == tag.tag_id:
                    print("Pose Rotation Matrix:\n", tag.pose_R)
                    print("Pose Translation Vector:\n", tag.pose_t)
                    self.tag_rotation = tag.pose_R
                    self.tag_translation = tag.pose_t

        return self.tag_rotation, self.tag_translation

class CloudFilter():
    def __init__(self, rotation, translation):
        self.R = rotation
        self.T = translation

        self.cloud_subscriber = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)
        self.cloud_data = None
    
    def callback(self, msg):
        self.cloud_data = msg
        self.save_point_cloud_as_pcd()
        
    def save_point_cloud_as_pcd(self):
        if self.cloud_data is not None:
            # Convert PointCloud2 to numpy array
            pc_np = pc2.read_points(self.cloud_data, field_names=("x", "y", "z"), skip_nans=True)
            
            # Create PointCloud from numpy array
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pc_np)

            # Save PointCloud as PCD file
            o3d.io.write_point_cloud("/home/wei/PSP/files/point_cloud.pcd", pcd)
            rospy.loginfo("Point cloud saved as point_cloud.pcd")
    

if __name__ == '__main__':
    rospy.init_node("vision_capture")
    pose_detect = Detection()

    while not rospy.is_shutdown():
        rotation, translation = pose_detect.get_pose()
        cloud_filter = CloudFilter(rotation, translation)
        pose_detect.loop_rate.sleep()
