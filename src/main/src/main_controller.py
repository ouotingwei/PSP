#!/usr/bin/env python3
from getpcd.srv import REQU_PCD, RESP_PCD
from dsam.srv import REQU_DSAM, RESP_DSAM
from path_planning_ver1 import REQU_PP, RESP_PP
import rospy

class main_controller():
    def __init__(self):
        rospy.init_node('main')

        self.getpcd_proxy = rospy.ServiceProxy('getpcd_service', REQU_PCD)
        self.dsam_proxy = rospy.ServiceProxy('dsam_service', REQU_DSAM)
        self.pp_proxy = rospy.ServiceProxy('path_planning_ver1_service', REQU_PP)

        rospy.wait_for_service('getpcd_service')
        rospy.wait_for_service('dsam_service')
        rospy.wait_for_service('path_planning_ver1_service')
        
    def getpcd_fn(self, request):
        try:
            response = self.getpcd_proxy(request)
            if response:
                rospy.loginfo("getpcd_successfully completed !")
                request = False
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Error sending getpcd service request: {e}")

    def dsam_fn(self, request):
        try:
            response = self.dsam_proxy(request)
            if response:
                rospy.loginfo("down sample and merge successfully completed !")
                request = False
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Error sending dsam service request: {e}")

    def pp_fn(self, request):
        try:
            response = self.pp_proxy(request)
            if response:
                rospy.loginfo("path planning successfullly completed !")
                request = False
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Error sending pp service request: {e}")
        ...

    def plc_fn(self):  
        ...

    def fanuc_fn(self):
        ...

    def run(self):
        request = True
        self.getpcd_fn(request)
        self.dsam_fn(request)
        self.pp_fn(request)