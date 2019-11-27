#! /usr/bin/env python

#import statements
import numpy
import cv2
import cv_bridge
import rospy
import os
#specific imports
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from random import randint, random
from time import sleep, time
from datetime import timedelta
from sys import argv

class Follower:
	def __init__(self, ring_id):
		self.ring_id = ring_id
		
		if ring_id == "0":
			self.distavg = 5.9
		
		
		self.scanSub = rospy.Subscriber('thorvald_001/vel/scan', LaserScan, self.scanning)
		self.ring_pub = rospy.Publisher('thorvald_001/vel/scan_filtered', LaserScan,queue_size=1)

	def scanning(self, data):
		r = numpy.array(data.ranges)
		print(self.distavg)
		
		X = 0.2
		for i in range (0,len(r)):
			if r[i]+X > self.distavg:
				r[i] = numpy.Inf
		data.ranges =r
		self.ring_pub.publish(data)


if __name__ == '__main__':
	cv2.startWindowThread()
	rospy.init_node('follower')
	
	follower = Follower(argv[1])
	rospy.spin()

	cv2.destroyAllWindows()
