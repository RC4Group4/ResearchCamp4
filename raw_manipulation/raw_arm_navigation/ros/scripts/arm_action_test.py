#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_arm_navigation')
import rospy
import raw_arm_navigation.msg
import actionlib
import sys


def arm_action_test():
	client = actionlib.SimpleActionClient('MoveToJointConfigurationDirect', raw_arm_navigation.msg.MoveToJointConfigurationAction)
	client.wait_for_server()
	goal = raw_navigation.msg.MoveToJointConfigurationGoal()

	joint_names = ['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5']

	for i in range(5):
		jv = brics_actuator.msg.JointValue()
		jv.joint_uri = joint_names[i]
		jv.value = 0.1
		jv.unit = "rad"
		goal.positions.append(jv)

	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()
	
if __name__ == '__main__':
	rospy.init_node('action_test')
	result = arm_action_test()
	print result
