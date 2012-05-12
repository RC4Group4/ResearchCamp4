#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_arm_navigation')
import rospy
import raw_arm_navigation.msg
import brics_actuator.msg
import actionlib
import sys
import tf
import math


def arm_action_test():
    client = actionlib.SimpleActionClient('/arm_1/arm_controller/MoveToCartesianPoseDirect', raw_arm_navigation.msg.MoveToCartesianPoseAction)
    
    print 'wait for action server: MoveToCartesianPoseDirect'

    client.wait_for_server()
    action_goal = raw_arm_navigation.msg.MoveToCartesianPoseGoal()

   
    print 'establish action msg'
    action_goal.goal.pose.position.x = 0.0
    action_goal.goal.pose.position.y = 0.0
    action_goal.goal.pose.position.z = 0.4  
    
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    
    (qx,qy,qz,qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    
    action_goal.goal.pose.orientation.x = qx
    action_goal.goal.pose.orientation.y = qy
    action_goal.goal.pose.orientation.z = qz
    action_goal.goal.pose.orientation.w = qw    


    print 'send goal'
    client.send_goal(action_goal)
    
    print 'wait for result'
    client.wait_for_result()
    return client.get_result()
    
if __name__ == '__main__':
	rospy.init_node('action_test')
	result = arm_action_test()
	print result
