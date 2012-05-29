/*
 * teleop_joypad.h
 *
 *  Created on: May 27, 2012
 *      Author: fred
 */

#ifndef TELEOP_JOYPAD_H_
#define TELEOP_JOYPAD_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <brics_actuator/JointVelocities.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <string>

#define MAX_JOYPAD						1.0

#define BUTTON_DEADMAN					5
#define BUTTON_RUN						7

#define BUTTON_ARM_MOTORS_ON			9
#define BUTTON_ARM_MOTORS_OFF			8
#define BUTTON_ARM_MOTOR_1_2			0
#define BUTTON_ARM_MOTOR_3_4			1
#define BUTTON_ARM_MOTOR_5				2

#define AXES_BASE_LINEAR_X_SPEED		1
#define AXES_BASE_LINEAR_Y_SPEED		0
#define AXES_BASE_ANGULAR_SPEED			2

#define AXES_ARM_1						4
#define AXES_ARM_2						5


class TeleOpJoypad
{
public:
	TeleOpJoypad(ros::NodeHandle &nh);
	void publishCommands();

private:
	void cbJoy(const sensor_msgs::Joy::ConstPtr& command);
	void setAllJointVel(double motor_vel);
	void setSingleJointVel(double motor_vel, std::string joint_name);
	void turnOnArmMotorsOn();
	void turnOnArmMotorsOff();

	double speed_factor_;

	bool button_deadman_pressed_;
	bool button_deadman_pressed_prev_;
	bool button_run_pressed_;

	std::vector<std::string> arm_joint_names_;
	brics_actuator::JointVelocities arm_vel_;
	double arm_max_vel_;

	geometry_msgs::Twist base_vel_;
	geometry_msgs::Twist base_zero_vel_;
	geometry_msgs::Twist base_factor_;

	ros::Subscriber sub_joy_;
	ros::Publisher pub_base_vel;
	ros::Publisher pub_arm_vel;

	ros::ServiceClient srv_arm_motors_on;
	ros::ServiceClient srv_arm_motors_off;
};

#endif /* TELEOP_JOYPAD_H_ */
