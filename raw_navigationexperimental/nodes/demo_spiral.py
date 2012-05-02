# A demo application for youbot rotating while moving
# Author : Praveen Ramanujam
# Status : Needs improvization in terms of having better arguments and a little control over keyboard, refactoring is required

import roslib; roslib.load_manifest('navexp') 
import rospy
import re
import math
import time
import numpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

''' A class for moving the base '''
class MoveBase:
	''' Initialization member function to initialize the nodes and variables as demanded by ROS'''
	def __init__(self):
		rospy.init_node('omni', anonymous=True)
		self.cmd_vel_pub = rospy.Publisher('/base_controller/command', Twist)
		rospy.Subscriber('odom',Odometry,self.odom_callback)
		self.z = 0
		self.retangle= 0
		self.count = 0

        ''' Callback for reading Odometry Data. specially the orientation '''
	def odom_callback(self,odom):
		start = odom.pose.pose
		quaternion = start.orientation
		q0 = quaternion.x
        	q1 = quaternion.y
        	q2 = quaternion.z
       		q3 = quaternion.w
		self.z = math.atan2(2.*(q0*q1 + q2*q3), 1. - 2.*(q1**2 + q2**2))

	
       
	def action(self,vel):
		cmd_vel = Twist()
		cmd_vel.linear.x = vel[0]  #Set the x velocity
	        cmd_vel.linear.y = vel[1]  # Set the y velocity
		cmd_vel.angular.z = 1      # Set the z velocity .. constant
		self.cmd_vel_pub.publish(cmd_vel)
	
	
	def readz(self):
		return self.z	# Read angle from Callback
	
	'''Rotation Matrix Calculation for generating the velocities from world frame to local frame'''
	def rotmatz(self,z):
		
		R = numpy.matrix([[math.cos(z),-math.sin(z),0],[ math.sin(z),math.cos(z),0],[0,0,1]])	
		Rinv = numpy.linalg.pinv(R)
		wv = numpy.matrix([[1],[1],[0]])   #User changeable ... Should be passed as arguments as future.
		vel = Rinv*wv
		return vel
		

if  __name__ == '__main__':
	
	
	mb = MoveBase()    #Defining an object for move base
	
	while (True):
		z = mb.readz()  # Read values.
		vel = mb.rotmatz(z)  #Call the rotation matrix equation to calculate the local frame velocities.
		mb.action(vel)  # Call action to make the robot move
		
			
		
	
						
