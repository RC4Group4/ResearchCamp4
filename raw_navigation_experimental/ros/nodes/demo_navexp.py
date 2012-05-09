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
		self.cmd_vel_pub = rospy.Publisher('/base_controller/command', Twist)
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

	def action(self,vel):
		cmd_vel = Twist()
		cmd_vel.linear.x = vel[0]
	        cmd_vel.linear.y = vel[1]
		cmd_vel.angular.z = 1
		self.cmd_vel_pub.publish(cmd_vel)
	
	def odom_callback(self,odom):
		start = odom.pose.pose
		if (self.trigger == 0):
			self.firstx = start.position.x
			self.firsty = start.position.y
		self.distance = math.sqrt(abs(((self.firstx - start.position.x)*(self.firstx - start.position.x)) - ((self.firsty - start.position.y)*(self.firsty - start.position.y))))
		quaternion = start.orientation
		q0 = quaternion.x
        	q1 = quaternion.y
        	q2 = quaternion.z
       		q3 = quaternion.w
		self.z = math.atan2(2.*(q0*q1 + q2*q3), 1. - 2.*(q1**2 + q2**2))
		
	
	def readz(self):
		print self.distance
		return self.z	
	
	
	def rotmatz(self,z):
		
		R = numpy.matrix([[math.cos(z),-math.sin(z),0],[ math.sin(z),math.cos(z),0],[0,0,1]])	
		Rinv = numpy.linalg.pinv(R)
		wv = numpy.matrix([[0],[0],[0]])
		vel = Rinv*wv
		return vel
		

if  __name__ == '__main__':
	
	
	mb = MoveBase()
	n = 0.5
	zstart = abs(mb.readz())
	while (True):
		z = mb.readz()
		vel = mb.rotmatz(z)
		mb.action(vel)
		
			
		
	
						
