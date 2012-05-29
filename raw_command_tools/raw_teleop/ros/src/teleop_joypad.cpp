/*
 * teleop_joypad.cpp
 *
 *  Created on: May 27, 2012
 *      Author: fred
 */

#include "teleop_joypad.h"

TeleOpJoypad::TeleOpJoypad(ros::NodeHandle &nh)
{
	double max_speed = 0;
	ros::param::param<double>("~base_max_linear_x_vel", max_speed, 0.3);
	base_factor_.linear.x =  max_speed / MAX_JOYPAD;
	ros::param::param<double>("~base_max_linear_y_vel", max_speed, 0.3);
	base_factor_.linear.y =  max_speed / MAX_JOYPAD;
	ros::param::param<double>("~base_max_angular_vel", max_speed, 0.5);
	base_factor_.angular.z	 =  max_speed / MAX_JOYPAD;

	ros::param::param<double>("~arm_max_vel", max_speed, 0.2);
	arm_max_vel_ =  max_speed / MAX_JOYPAD;

	// ToDo: read from parameter server
	//ros::param::param<double>("/arm_1/arm_controller/joints", arm_joint_names_);
	arm_joint_names_.push_back("arm_link_0");
	arm_joint_names_.push_back("arm_link_1");
	arm_joint_names_.push_back("arm_link_2");
	arm_joint_names_.push_back("arm_link_3");
	arm_joint_names_.push_back("arm_link_4");

	arm_vel_.velocities.clear();
	for(unsigned int i=0; i < arm_joint_names_.size(); ++i)
	{
		brics_actuator::JointValue joint_value;

		joint_value.timeStamp = ros::Time::now();
		joint_value.joint_uri = arm_joint_names_[i];
		joint_value.unit = "rad";
		joint_value.value = 0.0;

		arm_vel_.velocities.push_back(joint_value);
	}

	sub_joy_ = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &TeleOpJoypad::cbJoy, this);
	pub_base_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	pub_arm_vel = nh.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command", 1);

	srv_arm_motors_on = nh.serviceClient<std_srvs::Empty>("arm_1/switchOnMotors");
	srv_arm_motors_off = nh.serviceClient<std_srvs::Empty>("arm_1/switchOffMotors");
}

void TeleOpJoypad::cbJoy(const sensor_msgs::Joy::ConstPtr& command)
{
	//general
	button_deadman_pressed_prev_ = button_deadman_pressed_;
	button_deadman_pressed_ = (bool)command->buttons[BUTTON_DEADMAN];
	button_run_pressed_ = (bool)command->buttons[BUTTON_RUN];

	//base
	if(!button_run_pressed_)
		speed_factor_ = 0.5;
	else
		speed_factor_ = 1.0;

	base_vel_.linear.x = command->axes[AXES_BASE_LINEAR_X_SPEED] * base_factor_.linear.x * speed_factor_;
	base_vel_.linear.y = command->axes[AXES_BASE_LINEAR_Y_SPEED] * base_factor_.linear.y * speed_factor_;
	base_vel_.angular.z = command->axes[AXES_BASE_ANGULAR_SPEED] * base_factor_.angular.z * speed_factor_;

	if(fabs(base_vel_.linear.x) < 0.05)
		base_vel_.linear.x = 0;
	if(fabs(base_vel_.linear.y) < 0.05)
		base_vel_.linear.y = 0;
	if(fabs(base_vel_.angular.z) < 0.01)
		base_vel_.angular.z = 0;

	//arm
	if((bool)command->buttons[BUTTON_ARM_MOTORS_ON])
		turnOnArmMotorsOn();
	else if((bool)command->buttons[BUTTON_ARM_MOTORS_OFF])
		turnOnArmMotorsOff();

	if((bool)command->buttons[BUTTON_ARM_MOTOR_1_2])
	{
		arm_vel_.velocities[0].value = command->axes[AXES_ARM_1] * arm_max_vel_ * speed_factor_;
		arm_vel_.velocities[1].value = command->axes[AXES_ARM_2] * arm_max_vel_ * speed_factor_;
	}
	else if((bool)command->buttons[BUTTON_ARM_MOTOR_3_4])
	{
		arm_vel_.velocities[2].value = command->axes[AXES_ARM_1] * arm_max_vel_ * speed_factor_;
		arm_vel_.velocities[3].value = command->axes[AXES_ARM_2] * arm_max_vel_ * speed_factor_;
	}
	else if((bool)command->buttons[BUTTON_ARM_MOTOR_5])
		arm_vel_.velocities[4].value = command->axes[AXES_ARM_1] * arm_max_vel_ * speed_factor_;

}

void TeleOpJoypad::turnOnArmMotorsOn()
{
	std_srvs::Empty empty;

	if(srv_arm_motors_on.call(empty))
		ROS_INFO("Turned ON arm motors");
	else
		ROS_ERROR("Could not turn ON arm motors");

	sleep(1);
}

void TeleOpJoypad::turnOnArmMotorsOff()
{
	std_srvs::Empty empty;

	if(srv_arm_motors_off.call(empty))
		ROS_INFO("Turned OFF arm motors");
	else
		ROS_ERROR("Could not turn OFF arm motors");

	sleep(1);
}

void TeleOpJoypad::setSingleJointVel(double motor_vel, std::string joint_name)
{
	for(unsigned int i=0; i < arm_vel_.velocities.size(); ++i)
	{
		if(arm_vel_.velocities[i].joint_uri == joint_name)
		{
			arm_vel_.velocities[i].timeStamp = ros::Time::now();
			arm_vel_.velocities[i].value = motor_vel;
		}
	}
}

void TeleOpJoypad::setAllJointVel(double motor_vel)
{
	for(unsigned int i=0; i < arm_joint_names_.size(); ++i)
		setSingleJointVel(motor_vel, arm_joint_names_[i]);
}

void TeleOpJoypad::publishCommands()
{
	if(button_deadman_pressed_)
	{
		pub_base_vel.publish(base_vel_);
		//pub_arm_vel.publish(arm_vel_);
	}

	else
	{
		if(button_deadman_pressed_prev_)
		{
			pub_base_vel.publish(base_zero_vel_);
			//setAllJointVel(0.0);
			//pub_arm_vel.publish(arm_vel_);
		}
	}
}

