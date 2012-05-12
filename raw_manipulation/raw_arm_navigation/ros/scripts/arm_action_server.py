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


class ArmMoveSimpleActionServer:
	def __init__(self):
		self.received_state = False
		
		### get parameters
		if (not rospy.has_param("unit")):
			rospy.logerr("No unit given.")
			exit(0)
		
		if (not rospy.has_param("joints")):
			rospy.logerr("No joints given.")
			exit(0)
		
		self.joint_names = rospy.get_param("joints")
		rospy.loginfo("Joints: %s", self.joint_names)
		self.current_joint_configuration = [0 for i in range(len(self.joint_names))]
		
		self.unit = rospy.get_param("unit")
		rospy.loginfo("Unit: %s", self.unit)
		
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
					self.current_joint_configuration[k] = msg.position[i]
					
		#print 'joint states received'
		self.received_state = True
		
	
	def execute_cb_move_joint_config_direct(self, action_msgs):

		print 'publish data'
		self.pub_joint_positions.publish(action_msgs.goal)
		
		print 'wait for reaching goal'
		#wait to reach the goal position
		while (not rospy.is_shutdown()):
			if (self.is_goal_reached(action_msgs.goal)):
				break
			
		print 'goal reached'
			
		result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
		result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
		
		print 'return success'
		self.as_move_joint_direct.set_succeeded(result)


	def execute_cb_move_cartesian_direct(self, action_msgs):
	
		print 'pose transform'
		x = action_msgs.goal.pose.position.x
		y = action_msgs.goal.pose.position.y
		z = action_msgs.goal.pose.position.z
	
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([action_msgs.goal.pose.orientation.x, 
																	action_msgs.goal.pose.orientation.y,
																	action_msgs.goal.pose.orientation.z,
																	action_msgs.goal.pose.orientation.w])
	
		print 'create pose'
		pose = self.iks.create_pose(x, y, z, roll, pitch, yaw)

		print pose
		
		print 'get ik solution'
		joint_config = self.iks.call_constraint_aware_ik_solver(pose)
		
		print joint_config
				
		result = raw_arm_navigation.msg.MoveToCartesianPoseResult()
				
		if (joint_config):
			print 'solution found'
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
			
			result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
			self.as_move_cart_direct.set_succeeded(result)
		
		else:
			print 'no solution found'
			result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.NO_IK_SOLUTION
			self.as_move_cart_direct.set_aborted(result)
				
		
	def is_goal_reached(self, goal_pose):
		for i in range(len(self.joint_names)):
			if (abs(goal_pose.positions[i].value - self.current_joint_configuration[i]) > 0.05):
				print 'GOAL NOT REACHED ###########################'
				return False
					
		print 'GOAL REACHED ###########################'
		return True


if __name__ == "__main__":
	rospy.init_node("arm_move_simple_action_server")
	
	action = ArmMoveSimpleActionServer()
	
	rospy.spin()
