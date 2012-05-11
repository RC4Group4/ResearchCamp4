#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_arm_navugation')

import rospy
import sensor_msgs.msg
import actionlib
import brics_actuator.msg
import control_msgs.msg


class ArmMoveSimpleActionServer:
	def __init__(self):
		self.received_state = False
		
		### get parameters
		if (not rospy.has_param("arm_move_simple_action_server/unit")):
			rospy.logerr("No unit given.")
			exit(0)
		
		if (not rospy.has_param("arm_move_simple_action_server/joints")):
			rospy.logerr("No joints given.")
			exit(0)
		
		self.joint_names = rospy.get_param("arm_move_simple_action_server/joints")
		rospy.loginfo("Joints: %s", self.joint_names)
		self.current_joint_configuration = [0 for i in range(len(self.joint_names))]
		
		self.unit = rospy.get_param("arm_move_simple_action_server/unit")
		rospy.loginfo("Unit: %s", self.unit)
		
		# subscriptions
		rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_states_callback)
		
		# publications
		self.pub_joint_positions = rospy.Publisher("position_command", brics_actuator.msg.JointPositions)
		
		# action server
		self.as_move_cart_direct = actionlib.SimpleActionServer("MoveToCartesianPoseDirect", raw_arm_navigation.msg.MoveToCartesianPoseAction, execute_cb = self.execute_cb_move_cartesian_direct)
		self.as_move_joint_direct = actionlib.SimpleActionServer("MoveToJointConfigurationDirect", raw_arm_navigation.msg.MoveToJointConfigurationAction, execute_cb = self.execute_cb_move_joint_config_direct)
	
		# additional classes
		iks = SimpleIkSolver()
	
	
	def joint_states_callback(self, msg):
		for k in range(len(self.joint_names)):
			for i in range(len(msg.name)):
				if (msg.name[i] == self.joint_names[k]):
					self.current_joint_configuration[k] = msg.position[i]
					
		
		self.received_state = True
		
	
	def execute_cb_move_joint_config_direct(self, action_msgs):

		self.pub_joint_positions.publish(action_msgs.goal)
			
		#wait to reach the goal position
		while (not rospy.is_shutdown()):
			if (self.is_goal_reached(action_msgs.goal)):
				break
			
		result = arm_navigation_msgs.msg.ArmNavigationErrorCodes()
		
		result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
		
		self.as_move_joint_direct.set_succeeded(result)


	def execute_cb_move_cartesian_direct(self, action_msgs):
	
		x = action_msgs.goal.pose.position.x
		y = action_msgs.goal.pose.position.y
		z = action_msgs.goal.pose.position.z
	
		(roll, pitch, jaw) = tf.transformations.euler_from_quaternion(action_msgs.goal.pose.orientation.x, 
																	action_msgs.goal.pose.orientation.y,
																	action_msgs.goal.pose.orientation.z,
																	action_msgs.goal.pose.orientation.w)
	
		pose = self.iks.create_pose(x, y, z, roll, pitch, yaw)
		
		joint_config = self.iks.call_constraint_aware_ik_solver(pose)
		
		result = arm_navigation_msgs.msg.ArmNavigationErrorCodes()
		
		if (joint_config):
			jp = brics_actuator.msg.JointPositions()
			
			for i in range(5):
				jv = brics_actuator.msg.JointValue()
				jv.joint_uri = self.iks.joint_names[i]
				jv.value = joint_config[i]
				jv.unit = "rad"
				jp.positions.append(jv)
			
			self.pub_joint_positions.publish(jp)
			
			#wait to reach the goal position
			while (not rospy.is_shutdown()):
				if (self.is_goal_reached(jp)):
					break
				
			result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
			self.as_move_joint_direct.set_succeeded(result)
		
		else:
			result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.NO_IK_SOLUTION
			self.as_move_joint_direct.set_aborted(result)
				
		
	def is_goal_reached(self, goal_pose):
		for i in range(len(self.joint_names)):
			if (abs(goal_pose.position[i] - self.configuration[i]) > 0.05):
				return False
		return True


if __name__ == "__main__":
	rospy.init_node("arm_move_simple_action_server")
	
	action = ArmMoveSimpleActionServer()
	
	rospy.spin()
