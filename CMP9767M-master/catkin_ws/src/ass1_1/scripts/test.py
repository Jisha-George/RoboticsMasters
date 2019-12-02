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
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from nav_msgs.msg import Odometry, OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray, GoalID
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from time import sleep, time
from std_srvs.srv import Empty

######################################################################################################

#kernel size for eroding the mask (to get rid of the small pixels in the mask)
kernel = numpy.ones((1,1), numpy.uint8)
kernel2 = numpy.ones((3,3), numpy.uint8)
kernel10 = numpy.ones((25,25), numpy.uint8)

######################################################################################################

class Weed_Killer:

	def __init__(self):

		self.bridge = cv_bridge.CvBridge()
		self.cGoal = 0
		self.ab = True
		#Subscribers
		self.imgSub = rospy.Subscriber('thorvald_001/kinect2_camera/hd/image_color_rect', Image, self.image_callback)
		self.scanSub = rospy.Subscriber('/thorvald_001/scan', LaserScan, self.scanning)
	# self.odomSub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.odom)
		self.statSub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status)
		
		#Publishers
		self.velPub = rospy.Publisher('thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=1)
		self.movePub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
		self.basPub = rospy.Publisher('/images/basil', Image, queue_size = 1)
		self.cabPub = rospy.Publisher('/images/cabbage', Image, queue_size = 1)
		self.imgPub = rospy.Publisher('/images/image', Image, queue_size = 1)
		self.canPub = rospy.Publisher('/move_base/cancel',GoalID, queue_size = 1)
		
		self.spray = rospy.ServiceProxy('/thorvald_001/spray', Empty)
		
		#Variables
		self.stat = -1
		o = 6.5
		d = 3.25
		self.path = [[o,0.25,0,1],[-o,0.25,0,1],[-o,-0.75,0,1],[o,-0.75,0,1],[o,-2.75,0,1],[-o,-2.75,0,1],[-o,-3.75,0,1],[o,-3.75,0,1]]
		sleep(2)
		self.main()
				
#######################################################################################################################################
        #checks the status message given by the move_base/status topic
	def status(self, msg):
		try:
			self.stat = msg.status_list[len(msg.status_list)-1].status
		except: 
			print("Error with status")
			print(msg)			
	def statu2s(self, msg):
		try:
			self.stat = msg.status_list[len(msg.status_list)-1].status
			self.ab = True
		except: 
			if self.ab == True:
				self.ab = False
######################################################################################################################################
        #alternative function to sleep, waits for a success status, an aborted status or timeout
	def wait(self):
		timer = time()
		sleep(1)
		try:
			while True:
				cv2.waitKey(1)
				if self.stat == 3:
					break;
				elif self.stat == 4:
					break;
				elif(time()-timer > 150):
					break;
				elif self.stat == -1:
					print("Status list empty")
		except:
			print("Status not published")
############################################################			
	def waiter(self):
		try:
			while True:
				cv2.waitKey(1)
				if self.stat == 3:
					break;
		except:
			print("Status not published")
######################################################################################################

	def moveg(self, cGoal):
		self.move(cGoal[0],cGoal[1],cGoal[2],cGoal[3])
		
	def move(self, x, y, z, w):
		co_move = MoveBaseActionGoal()
		co_move.goal.target_pose.pose.position.x = x
		co_move.goal.target_pose.pose.position.y = y
		co_move.goal.target_pose.pose.orientation.z = z
		co_move.goal.target_pose.pose.orientation.w = w
		#co_move.goal.target_pose.header.stamp = rospy.Time.now()
		co_move.goal.target_pose.header.frame_id = 'map'
		#co_move.goal.target_pose.header.seq = 102
		self.movePub.publish(co_move)
#######################################################################################################################################
        #relative move_base system... activated when a colour in mask is present
	def rel_move(self, x, y):
		colour_found = MoveBaseActionGoal()
		colour_found.goal.target_pose.pose.position.x = x
		colour_found.goal.target_pose.pose.position.y = y
		colour_found.goal.target_pose.pose.orientation.z = 0
		colour_found.goal.target_pose.pose.orientation.w = 1
		colour_found.goal.target_pose.header.stamp = rospy.Time.now()
		colour_found.goal.target_pose.header.frame_id = 'thorvald_001/base_link'
		colour_found.goal.target_pose.header.seq = 101		
		self.movePub.publish(colour_found)
######################################################################################################

	def scanning(self, msg):
		self.dist = msg.ranges[320]

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
		
		WeedMask = (BFill - BRe)
		
		basil = cv2.inRange(hsv, numpy.array([45, 0, 0]), numpy.array([130, 255, 255]))
		
		mask = basil
		mask = numpy.array(self.imfill(mask), dtype='uint8')
		mask = cv2.erode(mask,kernel)
		WeedMask = cv2.erode(WeedMask,kernel)
		
		maskd = cv2.bitwise_and(image, image, mask=mask)
		Re = cv2.bitwise_and(image, image, mask=WeedMask)
		
		mask[((mask.shape[0]/2)-10):((mask.shape[0]/2)+10), :] = 255
		
		
		self.basPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(maskd,(720, 450)), encoding = 'bgr8'))
		self.cabPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(Re,(720, 450)), encoding = 'bgr8'))
		self.imgPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(image,(720, 450)), encoding = 'bgr8'))

		a = numpy.sum(mask[((mask.shape[0]/2)-10):((mask.shape[0]/2)+10), (mask.shape[1]/3):((mask.shape[1]/3)*2)])
		b = numpy.sum(WeedMask[((WeedMask.shape[0]/2)-10):((WeedMask.shape[0]/2)+10), (WeedMask.shape[1]/3):((WeedMask.shape[1]/3)*2)])
		
		if a > b:
		#sum of cabbage is greater than basil:
			print('cabbage')
			self.finder(WeedMask)
		else:
			print('basil')
			self.finder(mask)
	#continue moving
			
		self.basPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(maskd,(720, 450)), encoding = 'bgr8'))
		self.cabPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(Re,(720, 450)), encoding = 'bgr8'))
		self.imgPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(image,(720, 450)), encoding = 'bgr8'))
#		cv2.imshow("image", cv2.resize(image,(720, 450)))
	#	cv2.imshow("Basil", cv2.resize(masked_image,(720, 450)))
#		cv2.imshow("Cabbage", cv2.resize(Re,(720, 450)))
	#	cv2.waitKey(3)
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

	def finder(self, mask):
		t = Twist()	
		#if the mask at center equals 1
		if numpy.any(mask[((mask.shape[0]/2)-10),((mask.shape[0]/2)+10)]) == 1:
			self.canPub.publish(GoalID())#	then move forward 2
			sleep(0.5)
			t.linear.x = 2
			self.velPub.publish(t)
			sleep(1)
			self.spray()#james said its this way
			t.linear.x = 0
			self.velPub.publish(t)
			sleep(0.5)
			t.linear.x = -1
			self.velPub.publish(t)
			sleep(1)
			#	spray 
			# move back 1
			#self.rel_move(-1,0)
			#sleep(1)
			self.moveg(self.cGoal)
			#	spray
			#	self.spray() #james said its this way
			# move back 1
			#	t.linear.x = -1
			#	self.velPub.publish(t)
######################################################################################################
	
	def main(self):
		i = 0
		while not(rospy.is_shutdown()):
			for i in range(len(self.path)):
				self.cGoal = self.path[i]
				self.moveg(self.cGoal)
				self.waiter()

######################################################################################################

if __name__ == '__main__':
	cv2.startWindowThread()
	rospy.init_node('start')

	start = Weed_Killer()
	rospy.spin()

	cv2.destroyAllWindows()
