#!/usr/bin/env python3
import rospy
from dsam.srv import dsam
from path_planning_ver1.srv import path_planning_ver1
from getpcd.srv import getpcd
from communication.srv import ModbusPLC, ModbusRobot, Ftp

request = False

class main_controller():
    def __init__(self):
        rospy.init_node('main')

        self.getpcd_proxy = rospy.ServiceProxy('getpcd', getpcd)
        self.dsam_proxy = rospy.ServiceProxy('dsam', dsam)
        self.pp_proxy = rospy.ServiceProxy('path_planning_ver1', path_planning_ver1)
        self.ftp_proxy = rospy.ServiceProxy('ftp_transfer', Ftp)
        self.funuc_proxy = rospy.ServiceProxy('modbus_robot_control', ModbusRobot)
        self.plc_proxy = rospy.ServiceProxy('modbus_plc_control', ModbusPLC)
        

    
    def getpcd_fn(self, request):
        rospy.wait_for_service('getpcd')

        try:
            response = self.getpcd_proxy(request)
            if response:
                rospy.loginfo("[!] getpcd_successfully completed !")
                request = False
        
        except rospy.ServiceException as e:
            rospy.logerr(f"[!] Error sending getpcd service request: {e}")

    def dsam_fn(self, request):
        rospy.wait_for_service('dsam')

        try:
            response = self.dsam_proxy(request)
            if response:
                rospy.loginfo("[!] down sample and merge successfully completed !")
                request = False
        
        except rospy.ServiceException as e:
            rospy.logerr(f"[!] rror sending dsam service request: {e}")

    def pp_fn(self, request):
        rospy.wait_for_service('path_planning_ver1')

        try:
            response = self.pp_proxy(request)
            if response:
                rospy.loginfo("[!] path planning successfullly completed !")
                request = False
        
        except rospy.ServiceException as e:
            rospy.logerr(f"[!] Error sending pp service request: {e}")
        
    
    def tfFile_fn(self, host, path):  
        rospy.wait_for_service('ftp_transfer')

        try:
            response = self.ftp_proxy(host, path)
            if response:
                rospy.loginfo("file transfer successfullly completed !")
                request = False
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Error sending file transfer service request: {e}")

    def fanuc_fn(self, request):
        rospy.wait_for_service('modbus_robot_control')

        try:
            response = self.funuc_proxy(request)
            if response:
                rospy.loginfo("executing robot successfullly completed !")
                request = False
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Error sending robot control service request: {e}")

    def plc_fn(self, setnum):  
        rospy.wait_for_service('modbus_plc_control')

        try:
            response = self.plc_proxy(setnum)
            if response:
                rospy.loginfo("writing plc successfullly completed !")
                request = False
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Error sending plc control service request: {e}")
        

    
    

    def run(self):
        self.getpcd_fn(True)
        self.dsam_fn(True)
        self.pp_fn(True)
        self.tfFile_fn('192.168.255.200', '/home/honglang/PSP/testingFile/S003.LS')
        self.fanuc_fn(True)

        