#!/usr/bin/python
import roslib; roslib.load_manifest('youbot_grri_demo')
import rospy

import smach
import smach_ros

class select_pose_to_approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'failed'],
            input_keys=['pose_list'],
            output_keys=['pose_to_approach'])

    def execute(self, userdata):
        rospy.sleep(1)
        return 'succeeded'

class adjust_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        return 'succeeded'