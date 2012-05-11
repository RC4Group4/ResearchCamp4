#!/usr/bin/env python
import roslib; roslib.load_manifest('vb_ptu46')
import rospy
import vb_ptu46.msg
import actionlib
import sys

def ptu_action_test():
	client = actionlib.SimpleActionClient('/vb_ptu46/SetPTUPosition', vb_ptu46.msg.PtuGotoAction)
	client.wait_for_server()
	goal = vb_ptu46.msg.PtuGotoGoal(pan=float(sys.argv[1]), tilt=float(sys.argv[2]))
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()
	
if __name__ == '__main__':
	rospy.init_node('action_test')
	result = ptu_action_test()
	print result
