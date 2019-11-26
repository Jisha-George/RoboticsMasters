#>>> green = numpy.uint8([[[0,255,0 ]]])
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
from matplotlib import pyplot

kernel = numpy.ones((1,1), numpy.uint8)
kernel2 = numpy.ones((3,3), numpy.uint8)
kernel10 = numpy.ones((25,25), numpy.uint8)

class Follower:

	def __init__(self):

		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('thorvald_001/kinect2_camera/hd/image_color_rect', Image,
										  self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('thorvald_001/teleop_joy/cmd_vel', Twist,
										   queue_size=1)
		#self.maskPub = rospy.Publisher('/13488071/images/mask', Image, queue_size = 1)
		self.twist = Twist()


	def image_callback(self, msg):

		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		image = cv2.resize(image,(480, 270))

		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		h, w, d = image.shape

		_, thresh = cv2.threshold(hsv[:,:,0].astype(float)/180, 0.2,1, cv2.THRESH_BINARY)
		
		
		BFill = numpy.array(self.imfill(thresh), dtype='uint8')
		BWE = cv2.erode(BFill, kernel10, iterations=1)
		BRe = self.imreconstruct(BWE, BFill, kernel)		
		
		WeedMask = (BFill - BRe)
		
		
		
		basil = cv2.inRange(hsv, numpy.array([45, 0, 0]), numpy.array([130, 255, 255]))
		
		mask = basil
#		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


		mask = numpy.array(self.imfill(mask), dtype='uint8')

		masked_image = cv2.bitwise_and(image, image, mask=mask)
		Re = cv2.bitwise_and(image, image, mask=WeedMask)

		cv2.imshow("image", cv2.resize(image,(720, 450)))
		cv2.imshow("Basil", cv2.resize(masked_image,(720, 450)))
		cv2.imshow("Cabbage", cv2.resize(Re,(720, 450)))
		cv2.waitKey(3)

	def imreconstruct(self, img, mask, st): #img is dilated

		_,mask = cv2.threshold(mask,0.5,1, cv2.THRESH_BINARY)

		img_new = img.copy()

		#i=0
		while True:
			#i+=1;
			#print(i)
			img = mask*cv2.dilate(img, st, iterations=1)
			#if i%100==0:
				#cv2.imwrite("out/IMG_"+str(i)+".png", img*255)
			if (numpy.array_equal(img_new,img)):
				#cv2.imwrite("out/IMG_"+str(i)+".png", img*255)
				return img
			img_new = img
			

	def imfill(self, im_in): #swap this out to only accept binary images
		h, w = im_in.shape[:2]
	
		#invert im_in (padd by 2)
		in2 = numpy.ones((h+2, w+2), numpy.uint8)
		in2[1:h+1,1:w+1] = im_in==0
	
		#make empty array (padd by 2)
		emptyMask = numpy.zeros((h+2, w+2), numpy.uint8)
		emptyMask[0,0] = 1
	
		#imreconstruct(emptyMask, im_in)
		rec = self.imreconstruct(emptyMask, in2, numpy.ones((3,3),numpy.uint8))
	
		RET = (rec[1:h+1,1:w+1]==0)

		#return not(reconstructed)
		return RET




cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
