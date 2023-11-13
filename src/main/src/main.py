#!/usr/bin/env python3
import rospy
from main_controller import main_controller
import sys
print(sys.path)

if __name__ == "__main__":
    mc = main_controller()
    mc.run()
