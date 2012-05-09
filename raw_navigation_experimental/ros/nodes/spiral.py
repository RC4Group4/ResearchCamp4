import roslib; roslib.load_manifest('navexp') 
import rospy
import re
import math
import time
import numpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from pr2_mechanism_controllers.msg import Odometer
"""/base_controller/command"""

class MoveBase:
	
	def __init__(self):
		rospy.init_node('omni', anonymous=True)
		self.cmd_vel_pub = rospy.Publisher('/base_controller/command', Twist)
		rospy.Subscriber('odom',Odometry,self.odom_callback)
		rospy.Subscriber('/base_odometry/odometer',Odometer,self.odometer_callback)
		self.z = 0
		self.retangle= 0
		self.count = 0
	
	def odometer_callback(self,angle):
		
		a = angle.angle - self.count*(2*math.pi)
		if (a > 2*math.pi):
			self.count = self.count + 1
		else :
			pass
		self.retangle= a

	def read(self):
		return self.retangle

	def action(self,vel):
		cmd_vel = Twist()
		cmd_vel.linear.x = vel[0]
	        cmd_vel.linear.y = vel[1]
		cmd_vel.angular.z = 0.2
		self.cmd_vel_pub.publish(cmd_vel)
	
	def odom_callback(self,odom):
		#odom = Odometry()
		start = odom.pose.pose
		quaternion = start.orientation
		q0 = quaternion.x
        	q1 = quaternion.y
        	q2 = quaternion.z
       		q3 = quaternion.w
		self.z = math.atan2(2.*(q0*q1 + q2*q3), 1. - 2.*(q1**2 + q2**2))
			

	def readz(self):
		return self.z	
	
	def trn2deg(self,z):
		if ((z >0) or (z == 0)) :
			z = z/2
		else :
			z = (1 - abs(z))/2
		return ((z*2)*math.pi)	
	
	def rotmatz(self,z):
		print z
		R = numpy.matrix([[math.cos(z),-math.sin(z),0],[ math.sin(z),math.cos(z),0],[0,0,1]])	
		Rinv = numpy.linalg.pinv(R)
		wv = numpy.matrix([[0.5],[0],[0]])
		vel = Rinv*wv
		return vel
		

if  __name__ == '__main__':
	
	print "Main Entered"
	mb = MoveBase()
	n = 0.5
	zstart = abs(mb.readz())
	while (True):
		z = mb.readz()
		print z		
		#z = mb.trn2deg(zend)
		#print z
	        vel = mb.rotmatz(z)
		mb.action(vel)
		#print "x:",vel[0]
		#print "y:",vel[1]
			
		
	rospy.spin()
						
