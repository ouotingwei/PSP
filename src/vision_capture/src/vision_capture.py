import cv2
import numpy as np
from pupil_apriltags import Detector
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

class Detection: 
    def __init__(self):
        self.camera_params = (604.373076, 608.179768, 317.741819, 247.792564)
        self.tag_size = 0.0596
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
        sample = 0

        if self.image is not None:
            at_detector = Detector(families='tag36h11')
            gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            tags = at_detector.detect(gray_image, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size)

            if len(tags) == 0:
                print('\033[91m' + " [-] POSE_ERROR: No APRILTAG detected." +  '\033[0m')
                self.LOGGING(state="   [e]No APRILTAG detected")
                return None, None

            else:
                for tag in tags:
                    if 1 == tag.tag_id:
                        sample = 1

                for tag in tags:
                    if 0 == tag.tag_id and sample == 1:
                        self.tag_rotation = tag.pose_R
                        self.tag_translation = tag.pose_t
                        print('\033[92m' + " [-] POSE OK." +  '\033[0m')
                    elif 0 == tag.tag_id and sample == 0:
                        print('\033[91m' + " [-] POSE_ERROR: No APRILTAG detected." +  '\033[0m')
                        self.LOGGING(state="   [e]No ")
        
        self.TF(self.tag_rotation, self.tag_translation)
        return self.tag_rotation, self.tag_translation

    def LOGGING(self, state):
        file_loc = '/home/wei/PSP/files/logging_file.txt'
        
        with open(file_loc, 'a') as file: 
            if file.tell() != 0: 
                file.write('\n') 
            file.write(state)
        
    def TF(self, rotation, transition):
        if rotation is None or transition is None:
            print("Error: Rotation or translation is None.")
            return
        
        file_loc = '/home/wei/PSP/files/TF.txt'
        tf = []
        for i in range(3):
            for j in range(3):
                tf.append(rotation[i][j])
        
        for i in range(3):
            tf.append(transition[i][0])
        
        with open(file_loc, 'w') as file: 
            for i in range(len(tf)):
                file.write(str(tf[i]))  # Convert to string before writing
                file.write('\n')



class CloudFilter():
    def __init__(self):
        self.loop_rate = rospy.Rate(1)
        self.cloud_subscriber = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)
        self.cloud_data = None
    
    def callback(self, msg):
        self.cloud_data = msg

    def center_point_cloud(self, point_cloud):
        centroid = np.mean(np.asarray(point_cloud.points), axis=0)
        translated_pcd = point_cloud.translate(-centroid)

        return translated_pcd
        
    def save_point_cloud_as_pcd(self, rotation, translation):
        if self.cloud_data is not None:

            pc_np = pc2.read_points(self.cloud_data, field_names=("x", "y", "z"), skip_nans=True)
            
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pc_np)

            pcd.rotate(np.linalg.inv(rotation), center=(0, 0, 0))
            pcd.translate(-1*np.linalg.inv(rotation)@translation)

            bounding_box = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=[0, 0, -0.1],
                max_bound=[0.22, 0.35, -0.03]
            )

            cropped_pcd = pcd.crop(bounding_box)

            downsampled_pcd = cropped_pcd.voxel_down_sample(voxel_size=0.005)

            centroid_pcd = self.center_point_cloud(downsampled_pcd)

            o3d.io.write_point_cloud("/home/wei/PSP/files/point_cloud.pcd", centroid_pcd)
            rospy.loginfo("Point cloud saved as point_cloud.pcd")


if __name__ == '__main__':
    rospy.init_node("vision_capture")
    pose_detect = Detection()
    cloud_filter = CloudFilter()

    while not rospy.is_shutdown():
        rotation, translation  = pose_detect.get_pose()
        cloud_filter.save_point_cloud_as_pcd(rotation, translation)
        pose_detect.loop_rate.sleep()