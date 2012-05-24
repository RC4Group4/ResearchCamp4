#!/usr/bin/python
import roslib; roslib.load_manifest('raw_kinect_calibration')
import rospy
import tf  
  
from geometry_msgs.msg import TransformStamped
from simple_script_server import *



pattern_tf_name = "/kinect_calibration_pattern"
last_pattern_transform = TransformStamped()

def cb_tf(data):
    for item in data.transforms:
        if item.child_frame_id == pattern_tf_name:
            rospy.loginfo("found kinect pattern:")
            last_pattern_transform = item

def main():
    rospy.init_node('tf_position_retrieval')
    
    tf_listener = tf.TransformListener()
    
    #subscribe to tf message where the pattern information are published
    rospy.Subscriber("/tf", tf.msg.tfMessage, cb_tf)
    
    #todo: read in pose list from parameter server
    if (not rospy.has_param("/script_server/arm/calibration_poses")):
        rospy.logerr("No arm calibration poses given.")
        exit(0)
    else:
        calib_poses = sorted(rospy.get_param("/script_server/arm/calibration_poses"))
        rospy.loginfo("arm calibration poses: %s", calib_poses)
        
    kinect_transforms = []
    
    #create instance of script server
    sss = simple_script_server()
    
    for item in calib_poses:
        #move arm to the given position (blocking call
        print "start move arm"
        sss.move("arm", item)
        print "arm arrived"
        
        rospy.sleep(2.0)
        
        time_after_movement = rospy.Time.now()
        
        # only take pose if time stamp is new than the time directly after the arm movement 
        while((last_pattern_transform.header.stamp - time_after_movement) <= 0):   
            print "wait for newest time stamp"   
            rospy.spin()
            
        #current_transform = last_pattern_transform.transform
        print "loopup transform"
        (trans, rot) = tf_listener.lookupTransform('/base_link', '/openni_camera', rospy.Time(0))
        
        current_transform.transform.translation = trans
        current_transform.transform.rotation = rot
        
        kinect_transforms.append(current_transform)
        
    print "calc mean"
        
    #calculate the mean 
    mean_x = 0
    mean_y = 0
    mean_z = 0
    mean_roll = 0
    mean_pitch = 0
    mean_raw = 0
    
    for item in kinect_transforms:
        mean_x += item.transform.translation.x
        mean_y += item.transform.translation.y
        mean_z += item.transform.translation.z
        
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([item.transform.rotation.x, item.transform.rotation.y, item.transform.rotation.z, item.transform.rotation.w])
        mean_roll += roll
        mean_pitch += pitch
        mean_yaw += yaw
        
    mean_x /= len(kinect_transforms)
    mean_y /= len(kinect_transforms)
    mean_z /= len(kinect_transforms)
    mean_roll /= len(kinect_transforms)
    mean_pitch /= len(kinect_transforms)
    mean_raw /= len(kinect_transforms)
            
    rospy.loginfo("%s %s %s %s %s %s", mean_x, mean_y, mean_z, mean_roll, mean_pitch, mean_yaw)
    
    
if __name__ == '__main__':
    main()
