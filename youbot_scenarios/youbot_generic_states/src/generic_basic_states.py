#!/usr/bin/python
import roslib
roslib.load_manifest('youbot_generic_states')
import rospy
import smach
import smach_ros


class init_robot(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.sleep(1)
        return 'succeeded'
    
    
class announce_failure(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded'])

    def execute(self, userdata):
        return 'succeeded'
    
        
    