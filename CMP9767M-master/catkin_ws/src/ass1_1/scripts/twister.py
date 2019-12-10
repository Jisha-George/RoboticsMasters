# USE ABOVE TO CALCULATE COLOUR VALUES IN HSV
# lower green = 50, 100, 100
# upper green = 70, 255, 255

#lower blue = 110, 100, 100
#upper blue = 130, 255, 255

#lower red = 0, 100, 100
#upper red = 10, 255, 255

#lower yellow = 20, 100, 100
#upper yellow = 40, 255, 255

#import statements
import numpy
import cv2
import cv_bridge
import rospy

#specific imports
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatusArray, GoalID
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from std_srvs.srv import Empty
from time import sleep, time

######################################################################################################

#kernel size for eroding the mask (to get rid of the small pixels in the mask)
kernel = numpy.ones((1,1), numpy.uint8)
kernel2 = numpy.ones((3,3), numpy.uint8)
kernel10 = numpy.ones((25,25), numpy.uint8)

######################################################################################################

class Weed_Killer:

	def __init__(self):

		self.statSub = rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, self.odom)
		
		#Publishers
		self.velPub = rospy.Publisher('thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=10)
		self.movePub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
		self.basPub = rospy.Publisher('/images/basil', Image, queue_size = 1)
		self.cabPub = rospy.Publisher('/images/cabbage', Image, queue_size = 1)
		self.imgPub = rospy.Publisher('/images/image', Image, queue_size = 1)
		self.canPub = rospy.Publisher('/move_base/cancel',GoalID, queue_size = 1)
		
		#Variables
		self.seq_id = 0
		self.spray = rospy.ServiceProxy('/thorvald_001/spray', Empty)
		self.bridge = cv_bridge.CvBridge()
		self.cGoal = [0,0,0,1]
		self.ab = True
		self.stat = -1
		self.yaw = 0

		o = 6.5 #
		self.path = [[o,0.25,1,0],[-o,0.25,1,0],[-o,-0.75,0,1],[o,-0.75,0,1],[o,-2.75,1,0],[-o,-2.75,1,0],[-o,-3.75,0,1],[o,-3.75,0,1]]
		print(self.path[0][0],self.path[0][1],self.path[0][2],self.path[0][3])
		sleep(2)
		self.imgSub = rospy.Subscriber('thorvald_001/kinect2_camera/hd/image_color_rect', Image, self.image_callback)

		self.main()
#######################################################################################################################################
        #gets information of the position and orientation from move_base/feedback (more accurate and reliable than /odom)

	def odom(self, msg):
		self.pos = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

######################################################################################################
	
	def image_callback(self, msg):
		
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		image = cv2.resize(image,(480, 270))

		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)		
		h, w, d = image.shape

		_, thresh = cv2.threshold(hsv[:,:,0].astype(float)/180, 0.2,1, cv2.THRESH_BINARY)

		BFill = numpy.array(self.imfill(thresh), dtype='uint8')
		BWE = cv2.erode(BFill, kernel10, iterations=1)
		BRe = self.imreconstruct(BWE, BFill, kernel2)		

		Cabbage = (BFill - BRe)
		basil_filt = cv2.inRange(hsv, numpy.array([45, 0, 0]), numpy.array([130, 255, 255]))

		Basil = basil_filt
		Basil = numpy.array(self.imfill(Basil), dtype='uint8')
		Basil = cv2.erode(Basil,kernel)
		Cabbage = cv2.erode(Cabbage,kernel)

		bas_mask = cv2.bitwise_and(image, image, mask=Basil)
		cab_mas = cv2.bitwise_and(image, image, mask=Cabbage) 

		a = numpy.sum(Basil[((Basil.shape[0]/2)-50):((Basil.shape[0]/2)+50), (Basil.shape[1]/3):((Basil.shape[1]/3)*2)])
		b = numpy.sum(Cabbage[((Cabbage.shape[0]/2)-50):((Cabbage.shape[0]/2)+50), (Cabbage.shape[1]/3):((Cabbage.shape[1]/3)*2)])

		bas_mask[0:h/2-50,:]=0
		bas_mask[h/2+50:h,:]=0
		cab_mas[0:h/2-50,:]=0
		cab_mas[h/2+50:h,:]=0

		self.basPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(bas_mask,(720, 450)), encoding = 'bgr8'))
		self.cabPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(cab_mas,(720, 450)), encoding = 'bgr8'))
		self.imgPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(image,(720, 450)), encoding = 'bgr8'))

			
######################################################################################################

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
			
######################################################################################################

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

######################################################################################################
	
	def main(self):
		r = rospy.Rate(10) 
		t = Twist()	
		_,_,yaw = euler_from_quaternion([0,0,self.pos[2], self.pos[3]])
		_,_,tYaw = euler_from_quaternion([0,0,self.path[1][2],self.path[1][3]])
		err = 0.5 * (tYaw-yaw)
		while err > 0.01:
			t.angular.z = err
			if err >= numpy.pi/2:
				t.angular.z = -err
			self.velPub.publish(t)
			
			_,_,yaw = euler_from_quaternion([0,0,self.pos[2], self.pos[3]])
			_,_,tYaw = euler_from_quaternion([0,0,self.path[1][2],self.path[1][3]])
			err = 0.5 * (tYaw-yaw)
			print("yaw= {}  tYaw= {}  err= {}".format(yaw,tYaw, err))
			r.sleep()
######################################################################################################

if __name__ == '__main__':
	cv2.startWindowThread()
	rospy.init_node('start')

	start = Weed_Killer()
	rospy.spin()

	cv2.destroyAllWindows()
