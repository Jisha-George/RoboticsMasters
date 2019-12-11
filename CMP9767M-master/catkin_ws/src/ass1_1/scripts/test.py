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
		self.odomSub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.odom)
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
		o = 4.5
		self.path = [[o,-3.75,1,0],[-6.5,-3.75,1,0],[-4,-2.75,0,1],[6.5,-2.75,0,1],[6.5,-0.75,1,0],[-3,-0.75,1,0],[-4,0.25,0,1],[6.5,0.25,0,1]]
		sleep(2)
		
		print(self.path[0][0],self.path[0][1],self.path[0][2],self.path[0][3])
		self.move(self.path[0][0],self.path[0][1],self.path[0][2],self.path[0][3])
		self.wait()
		sleep(2)
		print("1")
		self.imgSub = rospy.Subscriber('thorvald_001/kinect2_camera/hd/image_color_rect', Image, self.image_callback)
		self.main()
#######################################################################################################################################
        #gets information of the position and orientation from move_base/feedback (more accurate and reliable than /odom)
	def odom(self, msg):
		self.pos = [msg.feedback.base_position.pose.position.x, msg.feedback.base_position.pose.position.y, msg.feedback.base_position.pose.orientation.z, msg.feedback.base_position.pose.orientation.w]
		
	def Odom(self, msg):
		self.ori = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
			
#######################################################################################################################################
        #checks the status message given by the move_base/status topic
	def status(self, msg):
		try:
			self.stat = msg.status_list[len(msg.status_list)-1].status
			
		except: 
			print("Error with status")			
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
				if self.stat == 3:
					print("stat3")
					break;
				elif self.stat == 4:
					print("stat4")
					break;
				elif(time()-timer > 60):
					print("timer")
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
#				elif (time()-timer > 60):
	#				if self.stat == 2:
		#				break;
				elif self.stat == 4:
					break;
				elif self.stat == -1:
					print("Status list empty")
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
		co_move.goal.target_pose.header.stamp = rospy.Time.now()
		co_move.goal.target_pose.header.frame_id = 'map'
		co_move.goal.target_pose.header.seq = 102
		self.seq_id = 102
		self.movePub.publish(co_move)
#######################################################################################################################################
        #relative move_base system... activated when a colour in mask is present
	def rel_move(self, x, y, seq=101):
		twister = MoveBaseActionGoal()
		twister.goal.target_pose.pose.position.x = x
		twister.goal.target_pose.pose.position.y = y
		twister.goal.target_pose.pose.orientation.z = 0
		twister.goal.target_pose.pose.orientation.w = 1
		twister.goal.target_pose.header.stamp = rospy.Time.now()
		twister.goal.target_pose.header.frame_id = '/thorvald_001/base_link'
		twister.goal.target_pose.header.seq = seq
		self.seq_id = seq
		self.movePub.publish(twister)
######################################################################################################

	def Vscan(self, msg):
		self.distF = msg.ranges[210:630]
		self.distB = msg.ranges[631:896]
		######################################################################################################

	def scanning(self, msg):
		self.dist = msg.ranges[200:550]
		
######################################################################################################
	
	def image_callback(self, msg):
		print("2")
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		image = cv2.resize(image,(480, 270))

		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)		
		h, w, d = image.shape

		_, thresh = cv2.threshold(hsv[:,:,0].astype(float)/180, 0.2,1, cv2.THRESH_BINARY)
		print("3")
		BFill = numpy.array(self.imfill(thresh), dtype='uint8')
		BWE = cv2.erode(BFill, kernel10, iterations=1)
		BRe = self.imreconstruct(BWE, BFill, kernel2)		

		Cabbage = (BFill - BRe)
		basil_filt = cv2.inRange(hsv, numpy.array([45, 0, 0]), numpy.array([130, 255, 255]))
		print("4")
		Basil = basil_filt
		Basil = numpy.array(self.imfill(Basil), dtype='uint8')
		Basil = cv2.erode(Basil,kernel)
		Cabbage = cv2.erode(Cabbage,kernel)

		bas_mask = cv2.bitwise_and(image, image, mask=Basil)
		cab_mas = cv2.bitwise_and(image, image, mask=Cabbage) 
		print("5")
		a = numpy.sum(Basil[((Basil.shape[0]/2)-50):((Basil.shape[0]/2)+50), (Basil.shape[1]/3):((Basil.shape[1]/3)*2)])
		b = numpy.sum(Cabbage[((Cabbage.shape[0]/2)-50):((Cabbage.shape[0]/2)+50), (Cabbage.shape[1]/3):((Cabbage.shape[1]/3)*2)])

		bas_mask[0:h/2-50,:]=0
		bas_mask[h/2+50:h,:]=0
		cab_mas[0:h/2-50,:]=0
		cab_mas[h/2+50:h,:]=0
		print("6")
		self.basPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(bas_mask,(720, 450)), encoding = 'bgr8'))
		self.cabPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(cab_mas,(720, 450)), encoding = 'bgr8'))
		self.imgPub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(image,(720, 450)), encoding = 'bgr8'))
		
		#sum of cabbage is greater than basil
		if a > b:
			self.finder(Cabbage)
		else:
			self.finder(Basil)
		
		print("7")
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
		#print("12")		
		t = Twist()		
		r = rospy.Rate(10)
		print("8")
		
		if numpy.sum(mask[((mask.shape[0]/2)-50):((mask.shape[0]/2)+50), (mask.shape[1]/3):((mask.shape[1]/3)*2)]) > 150:
			
			print("9")
			print("disanebla")
			self.imgSub.unregister()
			print("cancle mov")
			self.canPub.publish(GoalID())
			sleep(2)		
			print(numpy.min(self.dist))
			
			if numpy.amin(self.dist) > 1.1 or numpy.min(self.distF) > 1.1: #
				print("forwards")	
				t.linear.x = 1
				self.velPub.publish(t)
				sleep(1)
				t.linear.x = 0.75
				self.velPub.publish(t)
				sleep(1)
				t.linear.x = 0
				self.velPub.publish(t)
				sleep(1)
				print("11")	
				print("spray")
				self.spray() #
				sleep(2)
						
			print(numpy.min(self.distB))
			
			if numpy.min(self.distB) > 1.1:
				print("backwards")
				t.linear.x = -1
				self.velPub.publish(t)
				sleep(1)
				t.linear.x = -0.45
				self.velPub.publish(t)
				sleep(1)
				t.linear.x = 0
				self.velPub.publish(t)
				sleep(1)
			
			print("13")
			print("align")
			_,_,yaw = euler_from_quaternion([0,0,self.ori[2], self.ori[3]])
#			_,_,yaw = euler_from_quaternion([0,0,self.pos[2], self.pos[3]])
			_,_,tYaw = euler_from_quaternion([0,0,self.cGoal[2],self.cGoal[3]])
			err = 0.5*(tYaw-yaw)
			print(err)
			print("14")
			
			while err > 0.01:
				t.angular.z = err
				if err >= numpy.pi/2:
					t.angular.z = -err
				self.velPub.publish(t)

				_,_,yaw = euler_from_quaternion([0,0,self.ori[2], self.ori[3]])
				_,_,tYaw = euler_from_quaternion([0,0,self.cGoal[2],self.cGoal[3]])
				err = 0.5*(tYaw-yaw)
				print("yaw= {}  tYaw= {}  err= {}".format(yaw,tYaw, err))
				r.sleep()
			
			print("16")
			print("stop")
			t.linear.x = 0
			t.angular.z = 0
			self.velPub.publish(t)
			sleep(1)
			print("17")
			
			print("reanebla")
			self.imgSub = rospy.Subscriber('thorvald_001/kinect2_camera/hd/image_color_rect', Image, self.image_callback)
			
			print("18")
			print("move")
			self.moveg(self.cGoal)
			self.wait()
#			
			if self.stat == 4:
				if numpy.min(self.distF) > 1.1:
					t.linear.x = 1
					self.velPub.publish(t)
					
				elif numpy.min(self.distB) > 1.1:
					t.linear.x = -1
					self.velPub.publish(t)
			
		print("19")
		print(self.cGoal)
		self.moveg(self.cGoal)
		sleep(5)
######################################################################################################
	
	def main(self):
		while not(rospy.is_shutdown()):
			i = 1
			print i
			while i < len(self.path):
				self.cGoal = self.path[i]
				self.moveg(self.cGoal)
				self.waiter()
				print self.cGoal
				print("ghhh")
				if self.seq_id == 101:
					print(self.seq_id, self.cGoal)
					self.canPub.publish(GoalID())
					print("po")
					continue
							
				elif self.seq_id == 103:
					print(self.seq_id, self.cGoal)
					self.canPub.publish(GoalID())
					print("mjn")
					continue
					
				i = i+1
				print("1")

######################################################################################################

if __name__ == '__main__':
	cv2.startWindowThread()
	rospy.init_node('start')

	start = Weed_Killer()
	rospy.spin()

	cv2.destroyAllWindows()
