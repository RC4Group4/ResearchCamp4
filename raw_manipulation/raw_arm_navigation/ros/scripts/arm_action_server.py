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
		
		if (not rospy.has_param("arm_move_simple_action_server/unit")):
			rospy.logerr("No unit given.")
			exit(0)
		
		if (not rospy.has_param("arm_move_simple_action_server/joints")):
			rospy.logerr("No joints given.")
			exit(0)
		
		self.joint_names = rospy.get_param("arm_move_simple_action_server/joints")
		rospy.loginfo("Joints: %s", self.joint_names)
		self.current_joint_configuration = sensor_msgs.msg.JointState()
		
		self.unit = rospy.get_param("arm_move_simple_action_server/unit")
		rospy.loginfo("Unit: %s", self.unit)
		
		# subscriptions
		rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_states_callback)
		
		self.pub_joint_positions = rospy.Publisher("position_command", brics_actuator.msg.JointPositions)
		
		self.action = actionlib.SimpleActionServer("MoveToCartesianPoseDirect", raw_arm_navigation.msg.MoveToCartesianPoseAction, execute_cb = self.execute_cb_move_cartesian_direct)
		self.action = actionlib.SimpleActionServer("MoveToJointConfigurationDirect", raw_arm_navigation.msg.MoveToJointConfigurationAction, execute_cb = self.execute_cb_move_joint_config_direct)
	
	
	def joint_states_callback(self, msg):
		for k in range(len(self.joint_names)):
			for i in range(len(msg.name)):
				if (msg.name[i] == self.joint_names[k]):
					self.current_joint_configuration.position.append(msg.position[i])
					self.current_joint_configuration.name.append(msg.position[i])
		self.received_state = True

	
	def execute_cb_move_joint_config_direct(self, action_msgs):
		is_timed_out = False
		start = rospy.Time.now()
		duration = rospy.Duration(5.0)

		self.pub_joint_positions.publish(action_msgs.goal)
			
		#wait to reach the goal position
		while (not rospy.is_shutdown()):
			if (self.is_goal_reached(conf, self.configuration)):
				break
			if (rospy.Time.now() - start > duration):
				is_timed_out = True
				break
			if (is_timed_out):
				break
			
		result = control_msgs.msg.FollowJointTrajectoryResult()
		if (is_timed_out):
			result.error_code = control_msgs.msg.FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
			self.action.set_aborted(result)
		else:
			result.error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
			self.action.set_succeeded(result)


	def is_goal_reached(self, goal_pose):
		for i in range(len(self.joint_names)):
			if (abs(goal_pose[i] - self.configuration[i]) > 0.05):
				return False
		return True


if __name__ == "__main__":
	rospy.init_node("arm_move_simple_action_server")
	
	action = ArmMoveSimpleActionServer()
	
	rospy.spin()
