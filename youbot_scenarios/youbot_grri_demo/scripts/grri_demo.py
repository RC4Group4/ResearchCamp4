#!/usr/bin/python
import roslib; roslib.load_manifest('youbot_grri_demo')
import rospy

import smach
import smach_ros

# generic states
from generic_basic_states import *
from generic_navigation_states import *
from generic_state_machines import *

# scenario specific states
from grri_demo_states import *

# main
def main():
    rospy.init_node('grri_demo')

    SM = smach.StateMachine(outcomes=['overall_failed'])
    
    # world knowledge
    SM.userdata.pose_list = ["grasp_blue_box", "grasp_red_box"]
    SM.userdata.pose_to_approach = -1; 

    # open the container
    with SM:
        # add states to the container
        smach.StateMachine.add('INIT_ROBOT', init_robot(),
            transitions={'succeeded':'SELECT_POSE_TO_APPROACH', 
                         'failed':'ANNOUNCE_FAILURE'})
        
        smach.StateMachine.add('SELECT_POSE_TO_APPROACH', select_pose_to_approach(),
            transitions={'succeeded':'MOVE_TO_GRASP_POSE'})
        
        smach.StateMachine.add('MOVE_TO_GRASP_POSE', approach_pose(),
            transitions={'succeeded':'ADJUST_POSE', 
                        'failed':'ANNOUNCE_FAILURE'})
                
        smach.StateMachine.add('ADJUST_POSE', adjust_pose(),
            transitions={'succeeded':'SM_GRASP_OBJECT', 
                        'failed':'ANNOUNCE_FAILURE'})

        smach.StateMachine.add('SM_GRASP_OBJECT', sm_grasp_object(),
            transitions={'object_grasped':'PLACE_OBJECT_ON_REAR_PLATFORM', 
                         'failed':'ANNOUNCE_FAILURE'})
                        
        smach.StateMachine.add('PLACE_OBJECT_ON_REAR_PLATFORM', place_obj_on_rear_platform(),
            transitions={'succeeded':'SELECT_POSE_TO_APPROACH', 
                        'failed':'ANNOUNCE_FAILURE'})
        
        smach.StateMachine.add('ANNOUNCE_FAILURE', announce_failure(),
            transitions={'succeeded':'overall_failed'})
            
    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('GRRI_DEMO', SM, 'GRRI_DEMO')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()

if __name__ == '__main__':
    main()
