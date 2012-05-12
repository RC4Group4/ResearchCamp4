#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_arm_navigation')
import rospy
import raw_arm_navigation.msg
import brics_actuator.msg
import actionlib
import sys


def arm_action_test():
    client = actionlib.SimpleActionClient('/arm_1/arm_controller/MoveToJointConfigurationDirect', raw_arm_navigation.msg.MoveToJointConfigurationAction)
    
    print 'wait for action server: MoveToJointConfigurationDirect'

    client.wait_for_server()
    goal = raw_arm_navigation.msg.MoveToJointConfigurationGoal()

    joint_names = ['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5']

    print 'establish action msg'
    for i in range(5):
        jv = brics_actuator.msg.JointValue()
        jv.joint_uri = joint_names[i]
        jv.value = 0.0
        jv.unit = "rad"
        goal.goal.positions.append(jv)

    print 'send goal'
    client.send_goal(goal)
    
    print 'wait for result'
    client.wait_for_result()
    return client.get_result()
    
if __name__ == '__main__':
	rospy.init_node('action_test')
	result = arm_action_test()
	print result
