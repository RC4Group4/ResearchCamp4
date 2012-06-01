#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_arm_navigation')

import rospy
import sensor_msgs.msg
import actionlib
import brics_actuator.msg
import raw_arm_navigation.msg
import arm_navigation_msgs.msg
import tf

from simple_ik_solver_wrapper import SimpleIkSolver


class ArmActionServer:
	def __init__(self):
		self.received_state = False
					
		if (not rospy.has_param("joints")):
			rospy.logerr("No arm joints given.")
			exit(0)
		else:
			self.joint_names = sorted(rospy.get_param("joints"))
			rospy.loginfo("arm joints: %s", self.joint_names)
			
		# read joint limits
		self.joint_limits = []
		for joint in self.joint_names:
			if ((not rospy.has_param("limits/" + joint + "/min")) or (not rospy.has_param("limits/" + joint + "/min"))):
				rospy.logerr("No arm joint limits given.")
				exit(0)
			else:
				limit = arm_navigation_msgs.msg.JointLimits()
				limit.joint_name = joint 
				limit.min_position = rospy.get_param("limits/" + joint + "/min")
				limit.max_position = rospy.get_param("limits/" + joint + "/max")
				self.joint_limits.append(limit)
		
		self.current_joint_configuration = [0 for i in range(len(self.joint_names))]
		self.unit = "rad"
		
		# subscriptions
		rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_states_callback)
		
		# publications
		self.pub_joint_positions = rospy.Publisher("position_command", brics_actuator.msg.JointPositions)
		
		# action server
		self.as_move_cart_direct = actionlib.SimpleActionServer("MoveToCartesianPoseDirect", raw_arm_navigation.msg.MoveToCartesianPoseAction, execute_cb = self.execute_cb_move_cartesian_direct)
		self.as_move_joint_direct = actionlib.SimpleActionServer("MoveToJointConfigurationDirect", raw_arm_navigation.msg.MoveToJointConfigurationAction, execute_cb = self.execute_cb_move_joint_config_direct)
	
		# additional classes
		self.iks = SimpleIkSolver()
	
	
	def joint_states_callback(self, msg):
		for k in range(len(self.joint_names)):
			for i in range(len(msg.name)):
				if (msg.name[i] == self.joint_names[k]):
					#rospy.loginfo("%s: %f", msg.name[i], msg.position[i])
					self.current_joint_configuration[k] = msg.position[i]
					
		#print 'joint states received'
		self.received_state = True
		
	
	def is_joint_configuration_not_in_limits(self, goal_configuration):
		for goal_joint in goal_configuration.positions:
			for joint_limit in self.joint_limits:
				if ((goal_joint.joint_uri == joint_limit.joint_name) and ((goal_joint.value < joint_limit.min_position) or (goal_joint.value > joint_limit.max_position))):
					rospy.logerr("goal configuration has <<%s>> in joint limit: %lf", goal_joint.joint_uri, goal_joint.value)
					return False
		
		return True
	
	
	def execute_cb_move_joint_config_direct(self, action_msgs):
		rospy.loginfo("move arm to joint configuration")
		
		if not self.is_joint_configuration_not_in_limits(action_msgs.goal):
			result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
			result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED
			self.as_move_joint_direct.set_aborted(result)
			return
			
		self.pub_joint_positions.publish(action_msgs.goal)
		
		#wait to reach the goal position
		while (not rospy.is_shutdown()):
			if (self.is_goal_reached(action_msgs.goal)):
				break
					
		result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
		result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
		
		self.as_move_joint_direct.set_succeeded(result)


	def execute_cb_move_cartesian_direct(self, action_msgs):
		rospy.loginfo("move arm to cartesian pose")
		
		joint_config = self.iks.call_constraint_aware_ik_solver(action_msgs.goal)
		
		result = raw_arm_navigation.msg.MoveToCartesianPoseResult()
				
		if (joint_config):
			rospy.loginfo("IK solution found")
			jp = brics_actuator.msg.JointPositions()
			
			for i in range(5):
				jv = brics_actuator.msg.JointValue()
				jv.joint_uri = self.iks.joint_names[i]
				jv.value = joint_config[i]
				jv.unit = self.unit
				jp.positions.append(jv)
				
		
			if not self.is_joint_configuration_not_in_limits(jp):
				result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
				result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED
				self.as_move_cart_direct.set_aborted(result)
				return
		
			
			self.pub_joint_positions.publish(jp)
			
			#wait to reach the goal position
			while (not rospy.is_shutdown()):
				if (self.is_goal_reached(jp)):
					break
			
			result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
			self.as_move_cart_direct.set_succeeded(result)
		
		else:
			rospy.logerr("NO IK solution found")
			result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.NO_IK_SOLUTION
			self.as_move_cart_direct.set_aborted(result)
				
		
	def is_goal_reached(self, goal_pose):
		for i in range(len(self.joint_names)):
			#rospy.loginfo("joint: %d -> curr_val: %f --- goal_val: %f", i, goal_pose.positions[i].value, self.current_joint_configuration[i])
			if (abs(goal_pose.positions[i].value - self.current_joint_configuration[i]) > 0.05):   #ToDo: threshold via parameter
				return False
		
					
		rospy.loginfo("arm goal pose reached")
		return True


if __name__ == "__main__":
	rospy.init_node("arm_action_server")
	
	action = ArmActionServer()
	
	rospy.loginfo("arm action server started")
	
	rospy.spin()
