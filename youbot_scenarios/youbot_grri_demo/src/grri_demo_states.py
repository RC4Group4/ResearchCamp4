#!/usr/bin/python
import roslib; roslib.load_manifest('youbot_grri_demo')
import rospy

import smach
import smach_ros

class select_pose_to_approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['base_pose_list', 'base_pose_to_approach'],
            output_keys=['base_pose_list', 'base_pose_to_approach'])

    def execute(self, userdata):
        if(userdata.base_pose_to_approach == -1):
            userdata.base_pose_to_approach = userdata.base_pose_list[0]
        else:
            count = 0
            for item in userdata.base_pose_list:
                if userdata.base_pose_to_approach == item:
                    userdata.base_pose_to_approach = userdata.base_pose_list[((count+1) % len(userdata.base_pose_list))]
                    break;
                count = count + 1 
        
        rospy.loginfo("selected pose: %s", userdata.base_pose_to_approach)
        rospy.sleep(1)
                
        return 'succeeded'

class adjust_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        return 'succeeded'
    

class check_platform_occupancy(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['platform_full', 'platform_has_free_slots'],
            input_keys=['rear_platform_free_poses'])

    def execute(self, userdata):
        if(len(userdata.platform_free_slots) > 0):
            return 'platform_has_free_slots'
        else:
            return 'platform_full'

class announce_full_platform(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'pending'],
            input_keys=['rear_platform_free_poses'],
            output_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'])

    def execute(self, userdata):
        
        wav_path = commands.getoutput("rospack find youbot_generic_states")
        
        filename = wav_path + "/files/beep-3.wav"
        print 'file: ', filename
        
        #ToDo: send turn velocity
        
        for i in range(5):
            os.system("aplay -q " + filename)
            rospy.sleep(1)
            
        #ToDo: Stop turning
            
        print "The platform is full, please remove all blocks and press -- C -- to continue the scenario"
        ret = sss.wait_for_input()
        if ret == 'C':
            #all poses are free now
            for i in len(userdata.rear_platform_occupied_poses):
                userdata.rear_platform_free_poses.append(rear_platform_occupied_poses.pop())
            return 'succeeded'
        else:
            print "Wrong input key"
            return 'pending'
        
        