/*
 * teleop_joypad_node.cpp
 *
 *  Created on: Dec 15, 2011
 *      Author: Frederik Hegger
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define MAX_JOYPAD			1.0

#define BUTTON_DEADMAN			5
#define BUTTON_RUN				7
#define AXES_LINEAR_X_SPEED		1
#define AXES_LINEAR_Y_SPEED		0
#define AXES_ANGULAR_SPEED		2

using namespace std;

bool deadman_pressed = false;
bool deadman_pressed_prev = false;
bool run_pressed = false;
double linear_x_speed = 0;
double linear_y_speed = 0;
double angular_speed = 0;

double linear_x_factor = 0;
double linear_y_factor = 0;
double angular_factor = 0;


void joy_cmds(const sensor_msgs::Joy::ConstPtr& command)
{
	deadman_pressed_prev = deadman_pressed;
	deadman_pressed = (bool)command->buttons[BUTTON_DEADMAN];
	run_pressed = (bool)command->buttons[BUTTON_RUN];

	linear_x_speed = command->axes[AXES_LINEAR_X_SPEED] * linear_x_factor;
	linear_y_speed = command->axes[AXES_LINEAR_Y_SPEED] * linear_y_factor;
	angular_speed = command->axes[AXES_ANGULAR_SPEED] * angular_factor;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_teleop_joypad");
    ros::NodeHandle nh;

    /* get parameters from ros parameter server */
    double max_speed = 0;
    ros::param::param<double>("~max_linear_x_speed", max_speed, 0.3);
    linear_x_factor =  (2*max_speed) / (2*MAX_JOYPAD);
    ros::param::param<double>("~max_linear_y_speed", max_speed, 0.3);
    linear_y_factor =  (2*max_speed) / (2*MAX_JOYPAD);
    ros::param::param<double>("~max_angular_speed", max_speed, 0.5);
    angular_factor = (2*max_speed) / (2*MAX_JOYPAD);

    ros::Subscriber sub_joy = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joy_cmds);
    ros::Publisher pub_base = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.linear.z = 0.0;
    base_cmd.angular.x = base_cmd.angular.y = base_cmd.angular.z = 0.0;

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
    	if(deadman_pressed)
    	{
    		base_cmd.linear.x = linear_x_speed;
    		base_cmd.linear.y = linear_y_speed;
    		base_cmd.angular.z = angular_speed;

    		if(!run_pressed)
    		{
				base_cmd.linear.x /= 2;
				base_cmd.linear.y /= 2;
				base_cmd.angular.z /= 2;
    		}

    		pub_base.publish(base_cmd);
    	}

		else
		{
			if(deadman_pressed_prev)
			{
				base_cmd.linear.x = 0;
				base_cmd.linear.y = 0;
				base_cmd.angular.z = 0;

				pub_base.publish(base_cmd);
			}
		}

    	ros::spinOnce();
    	loop_rate.sleep();
    }
}

