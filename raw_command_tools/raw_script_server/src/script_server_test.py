#!/usr/bin/python
import time

import roslib
roslib.load_manifest('raw_script_server')
import rospy
import actionlib

from raw_script_server.msg import *
from simple_script_server import *

sss = simple_script_server()

## Main routine for running the script server
#
if __name__ == '__main__':
	rospy.init_node('script_server_test')
	
	
	#sss.move("base", "intro")
	
	sss.move("arm", "zeroposition")
	
	sss.move("gripper", "open")
	
	rospy.sleep(2)
	
	sss.move("gripper", "close")

	rospy.sleep(2)
	
	sss.move("arm", [0.3, 0.1, 0.15, 0, ((math.pi/2) + (math.pi/4)), 0, "/arm_link_0"])
	
	rospy.sleep(2)
	
	sss.move("arm", [0.4, 0.0, 0.2, 0, ((math.pi/2) + (math.pi/4)), 0, "/base_link"])
	
	rospy.sleep(2)
	
	sss.move("arm", "initposition")

