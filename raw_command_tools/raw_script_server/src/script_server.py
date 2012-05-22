#!/usr/bin/python
import time

import roslib
roslib.load_manifest('raw_script_server')
import rospy
import actionlib

from raw_script_server.msg import *
from simple_script_server import *

sss = simple_script_server()

## Script server class which inherits from script class.
#
# Implements actionlib interface for the script server.
#
class script_server():
	## Initializes the actionlib interface of the script server.
	#
	def __init__(self):
		self.ns_global_prefix = "/script_server"
		self.script_action_server = actionlib.SimpleActionServer(self.ns_global_prefix, ScriptAction, self.execute_cb, False)
		self.script_action_server.start()
	
#------------------- Actionlib section -------------------#
	## Executes actionlib callbacks.
	#
	# \param server_goal ScriptActionGoal
	#
	def execute_cb(self, server_goal):
		server_result = ScriptActionResult().result
		if server_goal.function_name == "stop":
			handle01 = sss.stop(server_goal.component_name)
		elif server_goal.function_name == "move":
			handle01 = sss.move(server_goal.component_name,server_goal.parameter_name,mode=server_goal.mode)
		else:
			rospy.logerr("function <<%s>> not supported", server_goal.function_name)
			self.script_action_server.set_aborted(server_result)
			return
		
		server_result.error_code = handle01.get_error_code()
		if server_result.error_code == 0:
			rospy.logdebug("action result success")
			self.script_action_server.set_succeeded(server_result)
		else:
			rospy.logerr("action result error")
			self.script_action_server.set_aborted(server_result)

## Main routine for running the script server
#
if __name__ == '__main__':
	rospy.init_node('script_server')
	script_server()
	rospy.loginfo("script_server is running")
	rospy.spin()
