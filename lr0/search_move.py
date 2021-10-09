#! /usr/bin/python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import math

class SearchAndMove:
	def __init__(self):
		rospy.Subscriber('/turtle1/pose', Pose, self.callback1)
		rospy.Subscriber('/turtle2/pose', Pose, self.callback2)
		self.my_y = 0
		self.my_x = 0
		self.theta = 0
		self.pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size = 1)
	def callback1(self, msg):
		rospy.logerr(msg)
		newMsg = Twist()
		if self.is_close(msg):
			return
		newMsg.linear.x = math.sqrt((msg.x - self.my_x) ** 2 + (msg.y - self.my_y) ** 2) / 15
		newMsg.angular.z = 1.5 * (math.atan2(msg.y - self.my_y, msg.x - self.my_x) - self.theta)
		self.pub2.publish(newMsg)
	def callback2(self, msg):
		self.my_x = msg.x
		self.my_y = msg.y
		self.theta = msg.theta
	def is_close(self, msg):
		return abs(self.my_x - msg.x) < 0.1 and abs(self.my_y - msg.y) < 0.1

rospy.init_node('turtle_0')
SearchAndMove()
rospy.spin()
