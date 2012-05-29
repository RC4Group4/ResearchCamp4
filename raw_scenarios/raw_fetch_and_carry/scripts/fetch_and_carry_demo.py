#!/usr/bin/python
import roslib; roslib.load_manifest('raw_fetch_and_carry')
import rospy

import smach
import smach_ros

# generic states
from generic_basic_states import *
from generic_navigation_states import *
from generic_state_machines import *

# scenario specific states
from fetch_and_carry_demo_states import *

# main
def main():
    rospy.init_node('fetch_and_carry_demo')

    SM = smach.StateMachine(outcomes=['overall_failed'])
    
    # world knowledge
    SM.userdata.base_pose_list = ["S2", "D1"]
    SM.userdata.base_pose_to_approach = -1; 
    SM.userdata.object_list = [];
                                            # x, y, z, roll, pitch, yaw
    SM.userdata.rear_platform_free_poses = [[0.033 + 0.024 - 0.32,  0.0, 0.14, 0, -math.pi + 0.2, 0, "/arm_link_0"],   #front pos
                                            [0.033 + 0.024 - 0.28,  0.0, 0.14, 0, -math.pi + 0.3, 0, "/arm_link_0"],    #rear pos
                                            [0.033 + 0.024 - 0.235, 0.0, 0.14, 0, -math.pi + 0.3, 0, "/arm_link_0"]]
    
    SM.userdata.rear_platform_occupied_poses = []

    # open the container
    with SM:
        # add states to the container
        
        smach.StateMachine.add('INIT_ROBOT', init_robot(),
            transitions={'succeeded':'SELECT_POSE_TO_APPROACH', 
                         'failed':'overall_failed'})
        
        smach.StateMachine.add('SELECT_POSE_TO_APPROACH', select_pose_to_approach(),
            transitions={'succeeded':'MOVE_TO_GRASP_POSE'})
        
        smach.StateMachine.add('MOVE_TO_GRASP_POSE', approach_pose(),
            transitions={'succeeded':'SM_GRASP_OBJECT', 
                        'failed':'overall_failed'})
                      
        smach.StateMachine.add('SM_GRASP_OBJECT', sm_grasp_random_object(),
            transitions={'object_grasped':'PLACE_OBJECT_ON_REAR_PLATFORM', 
                         'failed':'overall_failed'})
                                
        smach.StateMachine.add('PLACE_OBJECT_ON_REAR_PLATFORM', place_obj_on_rear_platform(),
            transitions={'succeeded':'SELECT_POSE_TO_APPROACH', 
                        'failed':'overall_failed'})
              
            
    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('FETCH_AND_CARRY_DEMO', SM, 'FETCH_AND_CARRY_DEMO')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()

if __name__ == '__main__':
    main()
