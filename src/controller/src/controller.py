import rospy
from vision_capture.srv import VC
from path_planning_ver1.srv import PP

def capture(req):
    rospy.wait_for_service('vision_capture')

    try:
        response = 

def planning(req):