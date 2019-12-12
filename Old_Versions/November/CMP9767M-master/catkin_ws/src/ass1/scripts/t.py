# USE ABOVE TO CALCULATE COLOUR VALUES IN HSV
# lower green = 50, 100, 100
# upper green = 70, 255, 255

#lower blue = 110, 100, 100
#upper blue = 130, 255, 255

#lower red = 0, 100, 100
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

		#Subscribers
		self.velSub = rospy.Subscriber('/thorvald_001/2/velscan_filtered_2', LaserScan, self.Vscan)
		self.scanSub = rospy.Subscriber('/thorvald_001/scan', LaserScan, self.scanning)
		self.statSub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status)
		self.OdomSub = rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, self.Odom)
		
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
		self.dist = 0.0		
		self.distF = 0.0		
		self.distB = 0.0
		self.yaw = 0

		#path of the robot can be cahnged to suit co-ordinates
		#could have been made dynamic by making the robot wander and have reactive behaviour
		self.path = [[3.5,-3.75,1,0],[-5.5,-3.75,1,0],[-5.5,-2.75,0,1],[4,-2.75,0,1],[4,-0.75,1,0],[-4,-0.75,1,0],[-4,0.25,0,1],[4,0.25,0,1]]
		sleep(2)
		#move to first co-ordinates
		print(self.path[0][0],self.path[0][1],self.path[0][2],self.path[0][3])
		self.move(self.path[0][0],self.path[0][1],self.path[0][2],self.path[0][3])
		self.wait()
		sleep(2)
		#due to the robot having a reactive behaviour then the camera subscriber has to be initalised after the robot has reached the first co-ordinate
		self.imgSub = rospy.Subscriber('thorvald_001/kinect2_camera/hd/image_color_rect', Image, self.image_callback)
		#call the main function
		self.main()

#######################################################################################################################################
 #gets information of the position and orientation from move_base/feedback (more accurate and reliable than /odom)
	
	def Odom(self, msg):
		self.ori = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
			
#######################################################################################################################################
        #checks the status message given by the move_base/status topic
	def status(self, msg):
		try: #check if status is publishin
			self.stat = msg.status_list[len(msg.status_list)-1].status
		except: 
			print("Error with status.. trying again")			
######################################################################################################################################
        #alternative function to sleep, waits for a success status, an aborted status or timeout
	def wait(self):
		timer = time()
		sleep(1)
		try:
			while True:
				if self.stat == 3:
					break;
				elif self.stat == 4:
					break;
				elif(time()-timer > 60):
					break;
				elif self.stat == -1:
					print("Status list empty")
		except:
			print("Status not published")

############################################################			
	def waiter(self):
		timer = time()
		sleep(1)
		try:
			while True:
				if self.stat == 3:
					break;
				elif self.stat == 4:
					break;
				elif(time()-timer > 250):
					print("timer")
					break;
				elif self.stat == -1:
					print("Status list empty")
		except:
			print("Status not published")

######################################################################################################
	#move base function which moves the robot to the given co-ordinates based on the map
	def moveg(self, cGoal):
		self.move(cGoal[0],cGoal[1],cGoal[2],cGoal[3])
		
	def move(self, x, y, z, w):
		co_move = MoveBaseActionGoal()
		co_move.goal.target_pose.pose.position.x = x
		co_move.goal.target_pose.pose.position.y = y
		co_move.goal.target_pose.pose.orientation.z = z
		co_move.goal.target_pose.pose.orientation.w = w
		co_move.goal.target_pose.header.stamp = rospy.Time.now()
		co_move.goal.target_pose.header.frame_id = 'map'
		co_move.goal.target_pose.header.seq = 102
		self.seq_id = 102
		self.movePub.publish(co_move)

######################################################################################################
#scanning function for velodyne laserscan
	def Vscan(self, msg):
		self.distF = msg.ranges[210:630]
		self.distB = msg.ranges[631:896]
######################################################################################################
#scanning function of laserscan
	def scanning(self, msg):
		self.dist = msg.ranges[200:550]

######################################################################################################
	#image callback function to update the image everytime something changes, creates all the image object such as the mask
	def image_callback(self, msg):
		#create the image by reading from the image subcriber
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		image = cv2.resize(image,(480, 270))
		#change to hsv
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)		
		h, w, d = image.shape
		#perform image procession to extract cabbage and ground from the image
		_, thresh = cv2.threshold(hsv[:,:,0].astype(float)/180, 0.2,1, cv2.THRESH_BINARY)
		BFill = numpy.array(self.imfill(thresh), dtype='uint8')
		BWE = cv2.erode(BFill, kernel10, iterations=1)
		BRe = self.imreconstruct(BWE, BFill, kernel2)		
		Cabbage = (BFill - BRe)
		#perform image procession to extract basil and ground from the image
		Basil = cv2.inRange(hsv, numpy.array([45, 0, 0]), numpy.array([130, 255, 255]))
		Basil = numpy.array(self.imfill(Basil), dtype='uint8')
		Basil = cv2.erode(Basil,kernel)
		Cabbage = cv2.erode(Cabbage,kernel)
		#masks created for basil and cabbage
		bas_mask = cv2.bitwise_and(image, image, mask=Basil)
		cab_mas = cv2.bitwise_and(image, image, mask=Cabbage) 
		
		#the sum of the binary image for basil
		a = numpy.sum(Basil[((Basil.shape[0]/2)-50):((Basil.shape[0]/2)+50), (Basil.shape[1]/3):((Basil.shape[1]/3)*2)])
		#the sum of the binary image for cabbage
		b = numpy.sum(Cabbage[((Cabbage.shape[0]/2)-50):((Cabbage.shape[0]/2)+50), (Cabbage.shape[1]/3):((Cabbage.shape[1]/3)*2)])
		
		bas_mask[0:h/2-50,:]=0
		bas_mask[h/2+50:h,:]=0
		cab_mas[0:h/2-50,:]=0
		cab_mas[h/2+50:h,:]=0

		#publish images
		self.basPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(bas_mask,(720, 450)), encoding = 'bgr8'))
		self.cabPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(cab_mas,(720, 450)), encoding = 'bgr8'))
		self.imgPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(image,(720, 450)), encoding = 'bgr8'))
		
		#if sum of basil is greater than cabbage
		if a > b:
			self.finder(Cabbage)
		else:
			self.finder(Basil)
			
######################################################################################################
	#function to reconstruct binary image given the original and filtered masks
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
	#function to fill holes in binary images
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
 #function for when weed is found in the mask
	def finder(self, mask):	
		t = Twist()		
		r = rospy.Rate(10)
		#if the sum of the mask is more than 150 (if there is weed in the middle of the mask)
		if numpy.sum(mask[((mask.shape[0]/2)-50):((mask.shape[0]/2)+50), (mask.shape[1]/3):((mask.shape[1]/3)*2)]) > 150:
			
			#unregister the subscriber
			self.imgSub.unregister()
			
			#cancel the move base
			self.canPub.publish(GoalID())
			sleep(2)		
			
			#if there is nothing the laserscan that is less that 1 meter
			if numpy.amin(self.dist) > 1.1 or numpy.min(self.distF) > 1.1: #
				#move forwards
				t.linear.x = 1
				self.velPub.publish(t)
				sleep(1)
				t.linear.x = 0.75
				self.velPub.publish(t)
				sleep(1)
				t.linear.x = 0
				self.velPub.publish(t)
				sleep(1)
				#spray
				self.spray()
				sleep(2)

			#if there is nothing the velodyne
			if numpy.amin(self.distB) > 1.1:
				#move backwards
				t.linear.x = -1
				self.velPub.publish(t)
				sleep(1)
				t.linear.x = -0.45
				self.velPub.publish(t)
				sleep(1)
				t.linear.x = 0
				self.velPub.publish(t)
				sleep(1)
			
			#align the robot to face forwards
			#code adapted from ther construct how to rotate robot using feedback from odometery
			_,_,yaw = euler_from_quaternion([0,0,self.ori[2], self.ori[3]])
			_,_,tYaw = euler_from_quaternion([0,0,self.cGoal[2],self.cGoal[3]])
			err = 0.5*(tYaw-yaw)
			while err > 0.005:
				t.angular.z = err
				if err >= numpy.pi/2:
					t.angular.z = -err
				self.velPub.publish(t)
			
				_,_,yaw = euler_from_quaternion([0,0,self.ori[2], self.ori[3]])
				_,_,tYaw = euler_from_quaternion([0,0,self.cGoal[2],self.cGoal[3]])
				err = 0.5*(tYaw-yaw)
				r.sleep()

			#stop the robot
			t.linear.x = 0
			t.angular.z = 0
			self.velPub.publish(t)
			sleep(1)

			#enable the subscriber
			self.imgSub = rospy.Subscriber('thorvald_001/kinect2_camera/hd/image_color_rect', Image, self.image_callback)
			
			#move to the co-ordinate
			self.moveg(self.cGoal)
		
######################################################################################################
	
	def main(self):
		while not(rospy.is_shutdown()):
			i = 1
			#while i is less than or equal to the length of path
			while i <= len(self.path):
				#initalise current goal
				self.cGoal = self.path[i]
				#move to current goal
				self.moveg(self.cGoal)
				#wait until the robot reaches the current goal
				self.waiter()
				#go to next goal
				i = i+1
				print(self.cGoal)

######################################################################################################

if __name__ == '__main__':
	cv2.startWindowThread()
	rospy.init_node('start')

	start = Weed_Killer()
	rospy.spin()

	cv2.destroyAllWindows()

