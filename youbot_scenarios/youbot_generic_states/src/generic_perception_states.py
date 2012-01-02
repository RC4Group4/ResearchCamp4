#!/usr/bin/python
import roslib
roslib.load_manifest('youbot_generic_states')
import rospy
import smach
import smach_ros

class detect_object(smach.State):

    def __init__(self, pose = ""):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'])


    def execute(self, userdata):
        return 'succeeded'