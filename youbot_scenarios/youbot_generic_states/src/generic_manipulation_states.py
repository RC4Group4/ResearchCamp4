#!/usr/bin/python
import roslib
roslib.load_manifest('youbot_generic_states')
import rospy
import smach
import smach_ros
import tf

class grasp_nearest_object(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['object_list'])

    def execute(self, userdata):
        #find nearest object
        object_found = False
        min_dist = 1000.0
        min_index = -1
        
        for object in userdata.object_list:
            x = object.x
            y = object.y
            z = object.z
            
            dist = math.sqrt(x * x + y * y + z * z)
            
            if ((dist <= min_dist) and (dist >= 0.6)):
                min_dist = dist
                min_index = k
                object_found = True
        
        if (not object_found):
            return "failed"
             
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "/base_link"
        pose.header.stamp = rospy.Time.now() - rospy.Duration(0.5)
        pose.pose.position.x = coord[min_index][0]
        pose.pose.position.y = coord[min_index][1]
        pose.pose.position.z = coord[min_index][2]
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        
        base_pose = tf_listener.transformPose("base_link", pose)
        userdata.grasp_position = base_pose
        
        
        return 'succeeded'
    

class place_obj_on_rear_platform(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'])


    def execute(self, userdata):
        return 'succeeded'
    
    
    
    