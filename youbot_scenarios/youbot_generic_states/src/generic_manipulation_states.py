#!/usr/bin/python
import roslib
roslib.load_manifest('youbot_generic_states')
import rospy
import smach
import smach_ros
import tf

class grasp_object(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'])


    def execute(self, userdata):
        return 'succeeded'
    

class place_obj_on_rear_platform(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'])


    def execute(self, userdata):
        return 'succeeded'
    
    
    
    