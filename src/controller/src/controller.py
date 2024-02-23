import rospy
import sys
import os
import datetime
from vision_capture.srv import VC
from path_planning_ver1.srv import PP
from communication.srv import ModbusPLC, ModbusRobot, Ftp

request = False

class controller ():
    
    def __init__ ( self ):
        rospy.init_node ( 'controller' )
        
        self.capture_proxy = rospy.ServiceProxy ( 'vision_capture', VC )
        self.pp_proxy = rospy.ServiceProxy ( 'path_planning_ver1', PP )
        self.ftp_proxy = rospy.ServiceProxy('ftp_transfer', Ftp)
        self.funuc_proxy = rospy.ServiceProxy('modbus_robot_control', ModbusRobot)
        self.plc_proxy = rospy.ServiceProxy('modbus_plc_control', ModbusPLC)


    def capture( self, request ):
        rospy.wait_for_service( 'vision_capture' )

        try:
            response = self.capture_proxy( request )
            if response:
                print( '\033[94m' + "[SERVER] Vision Capture Successfully Completed." + '\033[0m' )
                request = False
        
        except rospy.ServiceException as e:
            print( '\033[91m' + " [SERVER] Error Sending Vision Capture Request." +  '\033[0m' )
            self.LOGGING(   "   [SERVER] Error Sending Vision Capture Request" )

    def planning(self, request):
        rospy.wait_for_service('path_planning_ver1')

        try:
            response = self.pp_proxy( request )
            if response:
                print( '\033[94m' + "[SERVER] Path Planning Successfully Completed." + '\033[0m' )
                request = False
        
        except rospy.ServiceException as e:
            print( '\033[91m' + " [SERVER] Error Sending Path Planning Request." +  '\033[0m' )
            self.LOGGING(   "   [SERVER] Error Sending Path Planning Request" )

    def fileTf(self, host, path):  
        rospy.wait_for_service('ftp_transfer')

        try:
            response = self.ftp_proxy(host, path)
            if response:
                print( '\033[94m' + "[SERVER] Transfer LS File to funuc Successfully Completed." + '\033[0m' )

        except rospy.ServiceException as e:
            print( '\033[91m' + " [SERVER] Error Transfer LS File Request." +  '\033[0m' )
            self.LOGGING(   "   [SERVER] Error Transfer LS File Request" )

    def fanucStart(self, request):
        rospy.wait_for_service('modbus_robot_control')

        try:
            response = self.funuc_proxy(request)
            if response:
                print( '\033[94m' + "[SERVER] Execute FUNUC Successfully Completed." + '\033[0m' )
        
        except rospy.ServiceException as e:
            print( '\033[91m' + " [SERVER] Error Execute FUNUC Request." +  '\033[0m' )
            self.LOGGING(   "   [SERVER] Error Execute FUNUC Request" )


    def plcControl(self, setnum):  
        rospy.wait_for_service('modbus_plc_control')

        try:
            response = self.plc_proxy(setnum)
            if response:
                print( '\033[94m' + "[SERVER] Writing PLC Successfully Completed." + '\033[0m' )
        
        except rospy.ServiceException as e:
            print( '\033[91m' + " [SERVER] Error Writing PLC Request." +  '\033[0m' )
            self.LOGGING(   "   [SERVER] Error Writing PLC Request" )

    def run(self):
        self.capture(True)
        self.planning(True)

    def LOGGING ( self, state ):
        homeDir = os.getenv( "HOME" )
        if homeDir is None:
            sys.stderr.write( "Failed to get the home directory.\n" )

        current_datetime = datetime.datetime.now()
        file_loc = homeDir + "/PSP/logfile/" + "logging_file.txt"
        
        with open( file_loc, 'a' ) as file: 
            if file.tell() != 0: 
                file.write( '\n' ) 
            file.write( state )


if __name__ == '__main__':
    homeDir = os.getenv( "HOME" )
    if homeDir is None:
        sys.stderr.write( "Failed to get the home directory.\n" )

    current_datetime = datetime.datetime.now()
    file_loc = homeDir + "/PSP/logfile/" + "logging_file.txt"

    current_datetime = datetime.datetime.now()

    with open( file_loc, 'a' ) as file: 
        if file.tell() != 0: 
            file.write( '\n' ) 
        file.write( "[Time] " + current_datetime.strftime("%Y-%m-%d_%H-%M-%S") )
        file.write( "[Start] ")

    ###
    c = controller()
    c.run()
    ###

    with open( file_loc, 'a' ) as file: 
        if file.tell() != 0: 
            file.write( '\n' ) 

        file.write( "[End] ")