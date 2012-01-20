#!/usr/bin/python
import roslib
roslib.load_manifest('youbot_generic_states')
import rospy
import smach
import smach_ros
import tf
import arm_configuration
import brsu_srvs.srv

import geometry_msgs.msg    ### ONLY FOR TESTING

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
        rospy.wait_for_service('/youbot_object_finder/GetObjectCandidates3D', 30)
        for i in range(20): 
            print "find object try: ", i
            resp = self.object_finder_srv()
             
            ## ONLY TESTING ToDo: remove
            resp.pointCloudCentroids = []
            point = geometry_msgs.msg.Point()
            point.header.frame_id = "/base_link"
            point.header.stamp = rospy.Time.now()
            point.pose.position.x = 0.4
            point.pose.position.y = 0
            point.pose.position.z = 0.20
            point.pose.orientation.x = 0
            point.pose.orientation.y = 0
            point.pose.orientation.z = 0
            point.pose.orientation.w = 1
            resp.pointCloudCentroids.append(point)
            ##### END TESTING
             
            if (len(resp.pointCloudCentroids) <= 0):
                rospy.loginfo('found no objects')
                rospy.sleep(1);
            else:    
                rospy.loginfo('found {0} objects'.format(len(resp.pointCloudCentroids)))
                break
        
        if (len(resp.pointCloudCentroids) <= 0):
            rospy.logerror("no graspable objects found");
            
            self.move_arm.moveToConfiguration("kinect_left_init")
            self.move_arm.moveToConfiguration("zeroposition")
                    
            return 'failed'
        
        userdata.object_list = resp.pointCloudCentroids
        
        self.move_arm.moveToConfiguration("kinect_left_init") 
        self.move_arm.moveToConfiguration("zeroposition") 
        
        return 'succeeded'