#!/usr/bin/python
import roslib
roslib.load_manifest('youbot_generic_states')
import rospy
import smach
import smach_ros
import actionlib
import move_base_msgs.msg
import tf

class approach_pose(smach.State):

    def __init__(self, pose = ""):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['base_pose_to_approach'])
        
        self.move_base = actionlib.SimpleActionClient("/move_base", move_base_msgs.msg.MoveBaseAction)
        self.pose = pose;    

    def execute(self, userdata):
        
        if(self.pose == ""):
            self.pose = userdata.base_pose_to_approach 
        
        if type(self.pose) is str:
            goal_pose = rospy.get_param('/script_server/base/' + self.pose)
            rospy.loginfo("moving to location: %s at %s", self.pose, goal_pose) 
            
        elif type(self.pose) is list and len(self.pose) == 3:
            rospy.loginfo("moving to pose: %s", self.pose)
            goal_pose = self.pose
        else: # this should never happen
            rospy.logerr("Invalid pose format")
            return 'failed'  
                   
        self.move_base.wait_for_server()
                             
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_pose[0]
        goal.target_pose.pose.position.y = goal_pose[1]
        goal.target_pose.pose.orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, goal_pose[2])
        
        self.move_base.send_goal(goal)
                
        while True:                
            rospy.sleep(1)
            base_state = self.move_base.get_state()
            if (base_state == actionlib.simple_action_client.GoalStatus.SUCCEEDED):
                return "succeeded"
            elif (base_state == actionlib.simple_action_client.GoalStatus.ACTIVE):
                continue
            else:
                print 'last state: ',base_state
                return "failed"
            
            
            