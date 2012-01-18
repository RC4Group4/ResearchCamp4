#!/usr/bin/python
import roslib
roslib.load_manifest('youbot_generic_states')
import rospy
import smach
import smach_ros
import tf
import arm_configuration

tf_listener = 0

class detect_object(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            output_keys=['object_list'])
        
        self.object_finder_srv = rospy.ServiceProxy('/youbot_object_finder/GetObjectCandidates3D', brsu_srvs.srv.GetObjectCandidateList3D)
        
        global tf_listener
        self.move_arm = arm_configuration.ArmConfiguration(tf_listener)

    def execute(self, userdata):
        # move arm out of the field of view of the kinect
        self.move_arm.moveToConfiguration("zeroposition")
        self.move_arm.moveToConfiguration("kinect_left_init")
        self.move_arm.moveToConfiguration("kinect_left")
        
        #get object pose list
        rospy.wait_for_service('/youbot_object_finder/GetObjectPoseList', 30)
        for i in range(20): 
            resp = self.object_finder_srv()

            if (len(resp.pointCloudCentroids) <= 0):
                rospy.sleep(1);
            else:    
                rospy.info('found {0} objects'.format(len(resp.pointCloudCentroids)))
                break
        
        if (len(resp.pointCloudCentroids) <= 0):
            rospy.error("no graspable objects found");
            
            self.move_arm.moveToConfiguration("kinect_left_init")
            self.move_arm.moveToConfiguration("zeroposition")
                    
            return 'failed'
        
        userdata.object_list = pointCloudCentroids
        
        return 'succeeded'