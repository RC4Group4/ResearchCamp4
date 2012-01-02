#!/usr/bin/python
import roslib; roslib.load_manifest('youbot_grri_demo')
import rospy

import smach
import smach_ros

class select_pose_to_approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['pose_list', 'pose_to_approach'],
            output_keys=['pose_list', 'pose_to_approach'])

    def execute(self, userdata):
        if(userdata.pose_to_approach == -1):
            userdata.pose_to_approach = userdata.pose_list[0]
        else:
            count = 0
            for item in userdata.pose_list:
                if userdata.pose_to_approach == item:
                    userdata.pose_to_approach = userdata.pose_list[((count+1) % len(userdata.pose_list))]
                    break;
                count = count + 1 
        
        rospy.loginfo("selected pose: %s", userdata.pose_to_approach)
        rospy.sleep(1)
                
        return 'succeeded'

class adjust_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        return 'succeeded'