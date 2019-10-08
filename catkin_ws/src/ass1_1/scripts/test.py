#!/usr/bin/env python

#>>> green = np.uint8([[[0,255,0 ]]])
#>>> hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
#>>> print hsv_green

# USE ABOVE TO CALCULATE COLOUR VALUES IN HSV
# lower green = 50, 100, 100
# upper green = 70, 255, 255

#lower blue = 110, 100, 100
#upper blue = 130, 255, 255

#lower red = 0, 100, 100
#upper red = 10, 255, 255

#lower yellow = 20, 100, 100
#upper yellow = 40, 255, 255

import numpy

import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

kernel = numpy.ones((10, 10), numpy.uint8)

class Follower:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('thorvald_002/kinect2_camera/hd/image_color_rect', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('thorvald_002/teleop_joy/cmd_vel', Twist,
                                           queue_size=1)
        #self.maskPub = rospy.Publisher('/13488071/images/mask', Image, queue_size = 1)
        self.twist = Twist()
	
    def image_callback(self, msg):
    	print("Hi")
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        gre = cv2.inRange(hsv, numpy.array([30, 0, 0]), numpy.array([130, 255, 255]))

        mask = gre
        h, w, d = image.shape
        #search_top = 0
        #search_bot = h
        #mask[0:search_top, 0:w] = 0
        #mask[search_bot:h, 0:w] = 0

        mask = cv2.erode(mask, kernel)
        #M = cv2.moments(mask)
	#found = 3

	# if M['m00'] == 0:
	# 	self.twist.angular.z = 0.5
	# 	self.twist.linear.x = 0
	# 	self.cmd_vel_pub.publish(self.twist)
	# 	print('twisting')
	# else:
	#     cx = int(M['m10']/M['m00'])
	#     cy = int(M['m01']/M['m00'])
	#     cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
	#     err = cx - w/2

	#     if mask[h-1, w/2] != 0:
	# 	self.twist.linear.x = 0
	# 	#stop 1m before cylinder if the bottom middle of the mask is white
	#     else:
	# 	self.twist.linear.x = 0.5
    
	#     self.twist.angular.z = -float(err) / 100
	#     print self.twist.angular.z
	#     #print(found)

 #    	    self.cmd_vel_pub.publish(self.twist)

        masked_image = cv2.bitwise_and(image, image, mask=mask)

        cv2.imshow("image", image)
        cv2.imshow("mask", masked_image)
        cv2.waitKey(3)

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()