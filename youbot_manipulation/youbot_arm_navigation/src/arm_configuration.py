#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_arm_navigation')

import rospy
import threading
import tf
import time
import math
import geometry_msgs.msg
import kinematics_msgs.srv
import kinematics_msgs.msg
import sensor_msgs.msg
import motion_planning_msgs.msg
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

from brics_actuator.msg import JointPositions, JointValue, Poison


class ArmConfiguration:
    def __init__(self, tfl):
        
        self.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        self.configuration = [0, 0, 0, 0, 0]
        self.received_state = False

        rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self._joint_states_callback)

        rospy.loginfo("Waiting for 'get_constraint_aware_ik' service")
        rospy.wait_for_service('/youbot_arm_kinematics/get_constraint_aware_ik')
        self.ciks = rospy.ServiceProxy('/youbot_arm_kinematics/get_constraint_aware_ik', kinematics_msgs.srv.GetConstraintAwarePositionIK)
        rospy.loginfo("Service 'get_constraint_aware_ik' is ready")

        #self.tfl = tf.TransformListener()
        self.tfl = tfl
        
        self.gripper_publisher = rospy.Publisher('/arm_1/gripper_controller/position_command', JointPositions)
        
        self.arm_publisher = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions)
    
        self.armclient = actionlib.SimpleActionClient("/arm_1/arm_controller/joint_trajectory_action", 
                                          control_msgs.msg.FollowJointTrajectoryAction)
        print "waiting for action server"
        self.armclient.wait_for_server()
    
        
    #callback function: when a joint_states message arrives, save the values
    def _joint_states_callback(self, msg):
        for k in range(5):
            for i in range(len(msg.name)):
                joint_name = "arm_joint_" + str(k + 1)
                if(msg.name[i] == joint_name):
                    self.configuration[k] = msg.position[i]
        self.received_state = True


    def _call_constraint_aware_ik_solver(self, goal_pose):
        
       # while (not self.received_state):
        #    time.sleep(0.1)
            
        req = kinematics_msgs.srv.GetConstraintAwarePositionIKRequest()
        
        req.timeout = rospy.Duration(0.5)
        req.ik_request.ik_link_name = "arm_link_5"
        req.ik_request.ik_seed_state.joint_state.name = self.joint_names
        req.ik_request.ik_seed_state.joint_state.position = self.configuration
        req.ik_request.pose_stamped = goal_pose
        
        try:
            resp = self.ciks(req)
        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: %s", str(e))
        return (resp.solution.joint_state.position, resp.error_code.val == motion_planning_msgs.msg.ArmNavigationErrorCodes.SUCCESS)
        
    def _createPose(self, x, y, z, roll, pitch, yaw, frame = "base_link"):
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time.now()  
        return pose
       
    def moveToPosition(self, joint_config):
            goal = control_msgs.msg.FollowJointTrajectoryGoal()
        
            goal.trajectory.joint_names = self.joint_names
        
            jtp = trajectory_msgs.msg.JointTrajectoryPoint()
                
            for i in range(5):
                jtp.positions.append(joint_config[i])
            
            goal.trajectory.points.append(jtp)
            print "sending goal"
            self.armclient.send_goal(goal)
        
            self.armclient.wait_for_result()
                    
    def moveToPose(self, pose):
        print "moveToPose: ", pose.pose.position.x ,", ", pose.pose.position.y, ", ", pose.pose.position.z
        
        print pose
        #pose.header.frame_id = "/arm_link_5"
        
        print 'try to find a solution'
        (conf, success) = self._call_constraint_aware_ik_solver(pose)
        print 'processing done'
        if (success):
            # publish solution directly as joint positions
            print 'found solution and try to send it'
            self.moveToPosition(conf)
            print 'send successfull'
            
            return True
        else:
            print 'IK solver did not find a solution'
            return False
    
    def _copyPose(self, pose):
        newpose = pose = geometry_msgs.msg.PoseStamped()
        newpose.pose.position.x = pose.pose.position.x
        newpose.pose.position.y = pose.pose.position.y
        newpose.pose.position.z = pose.pose.position.z
        
        newpose.pose.orientation.x = pose.pose.orientation.x
        newpose.pose.orientation.y = pose.pose.orientation.y
        newpose.pose.orientation.z = pose.pose.orientation.z
        newpose.pose.orientation.w = pose.pose.orientation.w
        
        return newpose
  
    def moveToPoseAndGraspFromTop(self, pose):
        tmpPose = self._copyWithOffset(pose, 0,0,0.05)
        
        self.moveGripperOpen()
        rospy.sleep(1.0)
        
        self.moveToPoseStepwise(tmpPose, 20)
        self.moveToPoseStepwise(pose, 20)
    
        self.moveGripperClose()
        
        rospy.sleep(2.0)
        
        self.moveToPoseStepwise(tmpPose, 20)
    
    def moveToPoseAndReleaseFromTop(self, pose):
        tmpPose = self._copyWithOffset(pose, 0,0,0.05)
        
        self.moveToPoseStepwise(tmpPose, 20)
        self.moveToPoseStepwise(pose, 20)
    
        self.moveGripperOpen()
        
        rospy.sleep(2.0)
        
        self.moveToPoseStepwise(tmpPose, 20)
                
    def _copyWithOffset(self, origin, x, y, z):
        ntarget = geometry_msgs.msg.PoseStamped()
        ntarget.pose.position.x = origin.pose.position.x + x
        ntarget.pose.position.y = origin.pose.position.y + y
        ntarget.pose.position.z = origin.pose.position.z + z
        ntarget.header.stamp = rospy.Time.now()
        
        ntarget.pose.orientation.x = origin.pose.orientation.x
        ntarget.pose.orientation.y = origin.pose.orientation.y
        ntarget.pose.orientation.z = origin.pose.orientation.z
        ntarget.pose.orientation.w = origin.pose.orientation.w
        
        return ntarget
    '''
    def _copyWithoutXYPosition(self, origin, target):
        ntarget = geometry_msgs.msg.PoseStamped()
        ntarget.pose.position.x = origin.pose.position.x
        ntarget.pose.position.y = origin.pose.position.y
        ntarget.pose.position.z = target.pose.position.z
        ntarget.header.stamp = rospy.Time.now()
        
        ntarget.pose.orientation.x = target.pose.orientation.x
        ntarget.pose.orientation.y = target.pose.orientation.y
        ntarget.pose.orientation.z = target.pose.orientation.z
        ntarget.pose.orientation.w = target.pose.orientation.w
        
        return ntarget  
    '''
    def moveToPoseStepwise(self, pose, steps):
        cpose = self._currentPose()
        
        print "moveToPoseGrasping: ", pose
        print "currentPose: ", cpose
        
        epose = geometry_msgs.msg.PoseStamped()
        epose.pose.position.x = pose.pose.position.x - cpose.pose.position.x
        epose.pose.position.y = pose.pose.position.y - cpose.pose.position.y
        epose.pose.position.z = pose.pose.position.z - cpose.pose.position.z
        
        
        epose.pose.orientation.x = pose.pose.orientation.x - cpose.pose.orientation.x
        epose.pose.orientation.y = pose.pose.orientation.y - cpose.pose.orientation.y
        epose.pose.orientation.z = pose.pose.orientation.z - cpose.pose.orientation.z
        epose.pose.orientation.w = pose.pose.orientation.w - cpose.pose.orientation.w
        
        print "error: "
        print epose.pose
        
        running = True
        
        #now in Z direction
        for i in range(steps):
            tmppose = geometry_msgs.msg.PoseStamped()
            tmppose.pose.position.x = cpose.pose.position.x + i * epose.pose.position.x / steps
            tmppose.pose.position.y = cpose.pose.position.y + i * epose.pose.position.y / steps
            tmppose.pose.position.z = cpose.pose.position.z + i * epose.pose.position.z / steps
            tmppose.header.stamp = rospy.Time.now()
            
            tmppose.pose.orientation.x = pose.pose.orientation.x
            tmppose.pose.orientation.y = pose.pose.orientation.y
            tmppose.pose.orientation.z = pose.pose.orientation.z
            tmppose.pose.orientation.w = pose.pose.orientation.w
            
            tmppose.header.frame_id = "/arm_link_0"

            self.moveToPose(tmppose)
            
                        
    def _currentPose(self):
        base_frame = "/arm_link_0"
        ctl_frame = "/arm_link_5"
        
        now = rospy.Time.now()
        
        self.tfl.waitForTransform(base_frame, ctl_frame,
                                  now, rospy.Duration(1.0));
                                  
                       
        (trans,rot) = self.tfl.lookupTransform(base_frame, ctl_frame, now)
        cpose = geometry_msgs.msg.PoseStamped()
        cpose.pose.position.x = trans[0]
        cpose.pose.position.y = trans[1]
        cpose.pose.position.z = trans[2]
        
        cpose.pose.orientation.x = rot[0]
        cpose.pose.orientation.y = rot[1]
        cpose.pose.orientation.z = rot[2]
        cpose.pose.orientation.w = rot[3]
        cpose.header.frame_id = "/arm_link_0"
        
        print "currentPose: ", cpose
        
        return cpose
    
    
    
    def moveToLocation(self, x,y,z, roll, pitch, yaw):
        return self.moveToPose(self._createPose(x,y,z, roll, pitch, yaw));
    
    def moveToConfiguration(self, arm_configuration_name):
        configname = "/script_server/arm/" + arm_configuration_name
        print "moveToConfiguration ", configname
        if rospy.has_param(configname):
            config = rospy.get_param(configname);
            self.moveToPosition(config)
            return True
        else:
            print "Configuration ", arm_configuration_name, " does not exist"
            return False
    
    #### publishing joint commands ####
    
    def _createJointPositions(self, joint_config):
        jp = JointPositions()
        
        for i in range(5):
            jv = JointValue()
            jv.joint_uri = self.joint_names[i]
            jv.value = joint_config[i]
            jv.unit = "rad"
            jp.positions.append(jv)
        
        return jp
    
    def moveToPositionPublish(self, joint_config):
            jp = self._createJointPositions(joint_config)
            self.arm_publisher.publish(jp)
            return True
    
    def moveToPosePublish(self, pose):
        print "moveToPosePublish: ", pose.pose.position.x, ", " , pose.pose.position.y, ", ", pose.pose.position.z
        
        (conf, success) = self._call_constraint_aware_ik_solver(pose)
        if (success):
            # publish solution directly as joint positions
            self.moveToPositionPublish(conf)
            
            return True
        else:
            print("IK solver didn't find a solution")
            return False
    
    def moveToLocationPublish(self, x,y,z, roll, pitch, yaw):
        return self.moveToPosePublish(self._createPose(x,y,z, roll, pitch, yaw));
    
    def moveToConfigurationPublish(self, arm_configuration_name):
        configname = "/arm_configurations/" + arm_configuration_name
        print "moveToConfiguration ", configname
        if rospy.has_param(configname):
            config = rospy.get_param(configname);
            self.moveToPositionPublish(config)
            return True
        else:
            print "Configuration ", arm_configuration_name, " does not exist"
            return False
    
    
    ##### Gripper #####
    def _createGripperJointPositions(self, left, right):
        jp = JointPositions()

        jv1 = JointValue()
        jv1.joint_uri = "gripper_finger_joint_l"
        jv1.value = left
        jv1.unit = "m"

        jv2 = JointValue()
        jv2.joint_uri = "gripper_finger_joint_r"
        jv2.value = right
        jv2.unit = "m"

        p = Poison()
        jp.poisonStamp = p

        jp.positions = [jv1, jv2] #list

        return jp
    
    def _createGripperJointPositionsSym(self, value):
        return self._createGripperJointPositions(value, value)
        
    def moveGripperOpen(self):
        print "moveGripperOpen"
        jp = self._createGripperJointPositionsSym(0.0115)
        self.gripper_publisher.publish(jp)
     
    def moveGripperClose(self):
        print "moveGripperClose"
        jp = self._createGripperJointPositionsSym(0.00)
        self.gripper_publisher.publish(jp)
        
    '''
    percentave=0.0 => gripper close
    percentage=1.0 => gripper open  
    '''
    def moveGripper(self, percentage):
        print "moveGripper(", percentage,")"
        jp = self._createGripperJointPositionsSym(0.0115 * percentage)
        self.gripper_publisher.publish(jp)   
        
    def moveGripperOpeningDistance(self, distance):
        print "moveGripperOpeningDistance(", distance,")"
        jp = self._createGripperJointPositionsSym(distance / 2.0)
        self.gripper_publisher.publish(jp)       
    
    def moveGripperDirect(self, left, right):
        print "moveGripperOpeningDistance(", left,",", right,")"
        jp = self._createGripperJointPositions(left, right)
        self.gripper_publisher.publish(jp) 
