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
		rospy.Subscriber('odom',Odometry,self.readstate)
		rospy.Subscriber('scan_front', LaserScan,self.filter_scan)
		rospy.Subscriber('/base_odometry/odometer',Odometer,self.readangle)
		self.z = 0
		self.retangle= 0
		self.count = 0
	
	def readangle(self,angle):
		
		a = angle.angle - self.count*(2*math.pi)
		if (a > 2*math.pi):
			self.count = self.count + 1
		else :
			pass
		self.retangle= a

	def read(self):
		return self.retangle

	def filter_scan(self, scan):
        	center_scan_index = (len(scan.ranges) + 1) / 2
		min_idx = center_scan_index - 50
		max_idx = center_scan_index + 50

		for dist in scan.ranges[min_idx:max_idx]:
		    if dist < self.min_front_dist and dist > 0.05:
		        self.min_front_dist = dist

	def action(self,vel):
		cmd_vel = Twist()
		cmd_vel.linear.x = vel[0]
	        cmd_vel.linear.y = vel[1]
		cmd_vel.angular.z = 0.05
		self.cmd_vel_pub.publish(cmd_vel)
	
	def readstate(self,odom):
		#odom = Odometry()
		start = odom.pose.pose
		self.z = start.orientation.z
			

	def readz(self):
		return self.z	
	
	def trn2deg(self,z):
		if z >0 :
			z = z/2
		else :
			z = (1 - abs(z))/2
		return (z*360)	
	
	def rotmatz(self,z):
		R = numpy.matrix([[math.cos(z),-math.sin(z),0],[ math.sin(z),math.cos(z),0],[0,0,1]])	
		Rinv = numpy.linalg.inv(R)
		wv = numpy.matrix([[0.1],[0],[0]])
		vel = Rinv*wv
		return vel
		

if  __name__ == '__main__':
	
	print "Main Entered"
	mb = MoveBase()
	n = 0.5
	zstart = abs(mb.readz())
	while (True):
		zend = mb.read()
		#print zend		
		z = mb.trn2deg(zend)
		#print z
	        vel = mb.rotmatz(z)
		mb.action(vel)
		print "x:",vel[0]
		print "y:",vel[1]
			
		
	rospy.spin()
						
