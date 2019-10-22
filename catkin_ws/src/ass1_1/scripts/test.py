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

kernel = numpy.ones((2,2), numpy.uint8)
kernel2 = numpy.ones((5,5), numpy.uint8)

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
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        gre = cv2.inRange(hsv, numpy.array([45, 0, 0]), numpy.array([130, 255, 255]))

        mask = gre
        h, w, d = image.shape

        #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.erode(mask, kernel)
        mask = cv2.dilate(mask, kernel2)

	 	# Threshold.
        # Set values equal to or above 220 to 0.
        # Set values below 220 to 255.
         
        im_th = mask.copy();
         
        # Copy the thresholded image.
        im_floodfill = im_th.copy()
         
        # Mask used to flood filling.
        # Notice the size needs to be 2 pixels than the image.
        h, w = im_th.shape[:2]
        mask = numpy.zeros((h+2, w+2), numpy.uint8)
         
        # Floodfill from point (0, 0)
        cv2.floodFill(im_floodfill, mask, (0,0), 255);
         
        # Invert floodfilled image
        im_floodfill_inv = cv2.bitwise_not(im_floodfill)
         
        # Combine the two images to get the foreground.
        mask = im_th | im_floodfill_inv
	 


        masked_image = cv2.bitwise_and(image, image, mask=mask)

        cv2.imshow("image", image)
        cv2.imshow("mask", masked_image)
        cv2.waitKey(3)

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()