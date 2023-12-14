#!/usr/bin/env python3
import rospy
from main_controller import main_controller
import sys
print(sys.path)

if __name__ == "__main__":
    print("main fn have been called")

    mc = main_controller()
    print(" run fn have been called")
    mc.run()
