#!/usr/bin/python
import roslib; roslib.load_manifest('raw_grri_demo')
import rospy
import arm_configuration

# main
def main():
    rospy.init_node('test_arm')

    tf_listener = 0
    move_arm = arm_configuration.ArmConfiguration(tf_listener)
    
    rospy.sleep(1.0)
    
    print '1'
    
    move_arm.moveGripperOpen()
    
    move_arm.moveToConfiguration("initposition")    
    #rospy.sleep(0.5)
    
    move_arm.moveToConfiguration("zeroposition")
    #rospy.sleep(0.5)

    move_arm.moveToConfiguration("pregrasp_back_init")    
    #rospy.sleep(0.5) 
    
    move_arm.moveToConfiguration("pregrasp_back")
    #rospy.sleep(0.5)
    
    move_arm.moveToConfiguration("pregrasp_back_init")
    #rospy.sleep(0.5)
    
    move_arm.moveToConfiguration("zeroposition")
    #rospy.sleep(0.5)   
    
    move_arm.moveToConfiguration("kinect_left_init")
    #rospy.sleep(0.5)
     
    move_arm.moveToConfiguration("kinect_left")
    #rospy.sleep(0.5)
    
    move_arm.moveToConfiguration("kinect_left_init")
    #rospy.sleep(0.5) 
    
    move_arm.moveToConfiguration("zeroposition")
    #rospy.sleep(0.5)
    
    move_arm.moveToConfiguration("initposition")    
    
    move_arm.moveGripperClose()
    
    
    
    print '2'
        
if __name__ == '__main__':
    main()
