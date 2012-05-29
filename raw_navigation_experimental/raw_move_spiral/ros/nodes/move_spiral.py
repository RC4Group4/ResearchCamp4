#!/usr/bin/env python

#Author : Praveen Ramanujam


import roslib; roslib.load_manifest('navexp') 
import rospy
import re
import math
import time
import numpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MoveBase:
	
	def __init__(self):
		rospy.init_node('omni', anonymous=True)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)
		rospy.Subscriber('odom',Odometry,self.odom_callback)
		self.firstx = 0
		self.firsty = 0
		self.trigger = 0
		self.z = 0
		self.retangle= 0
		self.count = 0
		self.distance = 0
	
	def read(self):
		return self.retangle

	def action(self,vel,angvl,stoprotating):
		cmd_vel = Twist()
		cmd_vel.linear.x = vel[0]
	        cmd_vel.linear.y = vel[1]
		if (stoprotating != 0):
			cmd_vel.angular.z = angvl
		else :
			cmd_vel.angular.z = 0
		self.cmd_vel_pub.publish(cmd_vel)
	
	def odom_callback(self,odom):
		start = odom.pose.pose
		if (self.trigger == 0):
			self.firstx = start.position.x
			self.firsty = start.position.y
			self.trigger = 1
		self.distance = math.sqrt(abs(((self.firstx - start.position.x)*(self.firstx - start.position.x)) - ((self.firsty - start.position.y)*(self.firsty - start.position.y))))
		quaternion = start.orientation
		q0 = quaternion.x
        	q1 = quaternion.y
        	q2 = quaternion.z
       		q3 = quaternion.w
		self.z = math.atan2(2.*(q0*q1 + q2*q3), 1. - 2.*(q1**2 + q2**2))
		
	
	def readz(self):
		if self.z < 0 :
			self.z = 2*math.pi + self.z
		return self.z	
	
	def readd(self):
		return self.distance
	
	def rotmatz(self,z,x,y):
		R = numpy.matrix([[math.cos(z),-math.sin(z),0],[math.sin(z),math.cos(z),0],[0,0,1]])	
		Rinv = numpy.linalg.pinv(R)
		wv = numpy.matrix([[x],[y],[0]])
		vel = Rinv*wv
		return vel
		

if  __name__ == '__main__':
	
	
	mb = MoveBase()
	
	case = 1
	while (not rospy.is_shutdown()):
		
		z = mb.readz()
		if (mb.readd() <= 1.99):
                       vel = mb.rotmatz(z,0.2,0)
		else :
			vel = [0,0]
                z = mb.readz()
		
		if (z<= (math.pi/2)):	
			angvel = 0.15
		else :
			angvel = 0
		print "angle:",z
		print "distance",mb.readd()		
		mb.action(vel,angvel,1)
		vel = [0,0]
		angvel = 0
		
		
			
		
	
						
