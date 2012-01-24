#!/usr/bin/python
import roslib
roslib.load_manifest('youbot_generic_states')
import rospy
import smach
import smach_ros
import tf
import arm_configuration
import math

tf_listener = 0

class grasp_random_object(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['object_list'])
        
        global tf_listener
        self.move_arm = arm_configuration.ArmConfiguration(tf_listener)

    def execute(self, userdata):
        self.move_arm.moveGripperOpen()
        self.move_arm.moveToConfiguration("zeroposition")
        rospy.sleep(2.0)
        
        for object in userdata.object_list:         
            # ToDo: need to be adjusted to correct stuff
            if object.z <= 0.05 and object.z >= 0.15:
                continue
            
            target_pose = self.move_arm._createPose(object.x + 0.01, object.y + 0.015, object.z + 0.06, 0, math.pi, 0)
            print 'before grasp send'
            self.move_arm.moveToPose(target_pose)
            print 'before grasp send'
            rospy.sleep(1.0)
            break;
        
        self.move_arm.moveGripperClose()
        rospy.sleep(2.0)
        self.move_arm.moveToConfiguration("zeroposition")
        
        print 'grasp done'
        
        return 'succeeded'
    

class place_obj_on_rear_platform(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'],
            output_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses'])
        
        global tf_listener
        self.move_arm = arm_configuration.ArmConfiguration(tf_listener)

    def execute(self, userdata):   
        
        #ToDo: get next free rear slot
        
        self.move_arm.moveToConfiguration("zeroposition")
        self.move_arm.moveToConfiguration("pregrasp_back_init")
        self.move_arm.moveToConfiguration("pregrasp_back")
        
        target_pose = self.move_arm._createPose(0.033 + 0.024 - 0.28, 0.0, 0.14, 0, -math.pi + 0.3, 0, "arm_link_0")
        
        self.move_arm.moveToPose(target_pose)
        
        rospy.sleep(1.0)
        self.move_arm.moveGripperOpen()
        rospy.sleep(2.0)
        self.move_arm.moveToConfiguration("zeroposition")
   
        
        return 'succeeded'
    
    
class move_arm_out_of_view(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'])
        
        global tf_listener
        self.move_arm = arm_configuration.ArmConfiguration(tf_listener)

    def execute(self, userdata):   
        
        self.move_arm.moveToConfiguration("zeroposition")  
        self.move_arm.moveToConfiguration("kinect_left_init") 
        self.move_arm.moveToConfiguration("kinect_left") 
           
        return 'succeeded'
    
    