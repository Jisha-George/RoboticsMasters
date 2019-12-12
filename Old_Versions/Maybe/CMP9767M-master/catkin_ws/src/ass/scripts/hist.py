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

#######################################################################################################################################

#kernel size for eroding the mask (to get rid of the small pixels in the mask)
kernel = numpy.ones((5, 5), numpy.uint8)

#######################################################################################################################################

class Follower:
	
	def __init__(self):
		
		self.cvBridge = cv_bridge.CvBridge()
		#Subscribers
		self.imgSub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
		self.scanSub = rospy.Subscriber('/scan', LaserScan, self.scanning)
		self.mapSub = rospy.Subscriber('/map', OccupancyGrid, self.mapp)
		self.odomSub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.odom)
		self.statSub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status)
		#Publisher		
		self.movePub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
		self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size =1)
		self.maskPub = rospy.Publisher('/13488071/images/mask', Image, queue_size = 1)
		#Variables
		self.found = {"red": False, "yellow": False, "green": False, "blue": False}
		self.dist = 0.0
		self.imageobj = Image()
		self.path = []
		self.odomObj = Odometry()
		self.ab = True
		self.path = []
		self.nodehist = []
		self.plot = int(argv[1])
		self.mindy = float(argv[2])
		global mask, red, blu, yel, gre, M
                #start at 0,0
		sleep(2)
		self.move(1,0)
		self.wait()
		self.move(0,0)
		self.wait()
		self.main()
		
#######################################################################################################################################
        #checks the status message given by the move_base/status topic
	def status(self, msg):
		try:
			self.stat = msg.status_list[len(msg.status_list)-1].status
			self.ab = True
		except: 
			if self.ab == True:
				self.ab = False
			
######################################################################################################################################
        #alternative function to sleep, waits for a success status, an aborted status or timeout
	#saves time
	def wait(self):
		timer = time()
		sleep(1)
		
		while True:
			cv2.waitKey(1)
			if self.stat == 3:
				break;
			elif self.stat == 4:
				break;
			elif(time()-timer > 30):
				break;
	
#######################################################################################################################################
        #gets the data of the middle of the LaserScan from the /scan topic
	def scanning(self, msg):
		self.dist = msg.ranges[320]
		
#######################################################################################################################################
	#image callback fucntion to update the image everytime something changes, creates all the image object such as the mask	
	def image_callback(self, msg):
		self.imageobj = msg
		image = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')		
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		red = cv2.inRange(hsv, numpy.array([0, 100, 100]), numpy.array([6, 255, 255]))
		yel = cv2.inRange(hsv, numpy.array([20, 100, 100]), numpy.array([40, 255, 255]))
		gre = cv2.inRange(hsv, numpy.array([50, 100, 100]), numpy.array([70, 255, 255]))
		blu = cv2.inRange(hsv, numpy.array([110, 100, 100]), numpy.array([140, 255, 255]))

		mask = red * 0
		
		if not (self.found["red"]):
			mask += red
		if not (self.found["yellow"]):
			mask += yel
		if not (self.found["green"]):
			mask += gre
		if not (self.found["blue"]):
			mask += blu		

		mask = cv2.erode(mask, kernel)
		mask = cv2.medianBlur(mask,5)
		
		self.maskPub.publish(self.cvBridge.cv2_to_imgmsg(mask, encoding = 'mono8'))
		
		return (image, mask, red, yel, gre, blu, self.found)
	
#######################################################################################################################################
        #gets information of the position and orientation from move_base/feedback (more accurate and reliable than /odom)
	def odom(self, msg):
		self.pos = [msg.feedback.base_position.pose.position.x, msg.feedback.base_position.pose.position.y, msg.feedback.base_position.pose.orientation.z, msg.feedback.base_position.pose.orientation.w]
		self.path.append([self.pos[0], self.pos[1]])
	
#######################################################################################################################################
        #create random co-ordinates		
	def node_gen(self, quantity):
		return [[float(randint(-8,8))/2, float(randint(-10,10))/02, False] for i in range(quantity)]
		
#######################################################################################################################################		
        #create random nodes that have no been genereated yet but is also a set minimum distance away from all the other nodes
	def new_node(self, quant, minD):
		genstartnode = self.node_gen(1)
		 
		#if a history exists
		if len(self.nodehist) > 0:
			print "Re-Generation of Nodes"
			#keep finding new nodes until  a new space is found
			xval3 = numpy.array([x[0] for x in self.nodehist])
			yval3 = numpy.array([y[1] for y in self.nodehist])
			i = 0
			#keep trying new nodes until a space is found, or 50 attempts have failed
			while min(numpy.sqrt((xval3-genstartnode[0][0])**2 + (yval3-genstartnode[0][1])**2)) < minD:
				cv2.waitKey(1)
				genstartnode = self.node_gen(1)
				#if the 50 attempts fail, then reset the history
				i+=1
				if i > 50:
					print "Refreshing Node History" 
					self.nodehist = [genstartnode[0]]
					xval3 = numpy.array([x[0] for x in self.nodehist])
					yval3 = numpy.array([y[1] for y in self.nodehist])
					i = 0
		
		#Add new node
		self.nodehist.append(genstartnode[0])
		c = [genstartnode[0]]
		print("---")		
		print(c)
		print(self.nodehist)
		
		for i in range(quant-1): #add nodes until all nadded 
			for j in range(50): #timeout system
				next = self.node_gen(1)		

				xval2 = numpy.array([x[0] for x in self.nodehist])
				yval2 = numpy.array([y[1] for y in self.nodehist])

				if min(numpy.sqrt((xval2-next[0][0])**2 + (yval2-next[0][1])**2)) > minD:
				
					if not next in c:
						c.append(next[0])
						self.nodehist.append(next[0])
						break
		print("---")
		print(c)
		print(self.nodehist)
		print("###")
		return c

#######################################################################################################################################
        #global moving system that uses move_base
	def move(self, x, y):
		co_move = MoveBaseActionGoal()
		co_move.goal.target_pose.pose.position.x = x
		co_move.goal.target_pose.pose.position.y = y
		co_move.goal.target_pose.pose.orientation.w = 1
		co_move.goal.target_pose.header.stamp = rospy.Time.now()
		co_move.goal.target_pose.header.frame_id = 'map'
		co_move.goal.target_pose.header.seq = 102
		self.movePub.publish(co_move)
		
#######################################################################################################################################
        #relative move_base system... activated when a colour in mask is present
	def colour_move(self, x, z):
		colour_found = MoveBaseActionGoal()
		colour_found.goal.target_pose.pose.position.x = x
		colour_found.goal.target_pose.pose.orientation.z = z
		colour_found.goal.target_pose.pose.orientation.w = 1
		colour_found.goal.target_pose.header.stamp = rospy.Time.now()
		colour_found.goal.target_pose.header.frame_id = 'base_link'
		colour_found.goal.target_pose.header.seq = 101		
		self.movePub.publish(colour_found)
			
#######################################################################################################################################
	#function that writes the map (courtesy of James Heselden)	
	def mapp(self,msg):
		cv2.imwrite('map.png', cv2.flip(cv2.rotate(numpy.reshape(msg.data, newshape = (msg.info.height, msg.info.width)),cv2.ROTATE_90_COUNTERCLOCKWISE),1))	
		
#######################################################################################################################################

	def main(self):

		co = self.new_node(int(argv[1]), float(argv[2]))
		print ("Test " + str(argv[3]))
		print co
		t = Twist()			
		hn = 10
		wn = 12		
		start = time()
	
		#while there are unvisited co-ordinates or there are unfound poles
		while not([self.found[k] == True for k in self.found] == [True]*4):
			
			mapy = cv2.imread('map.png')
			
			#create array containing unvisited co-ordinates
			temp = [dist for dist in co if dist[2] == False]

			#add pixel co-ordinates for map
			xP2 = numpy.round((numpy.array([x[0] for x in self.nodehist]) - (hn/2)) * mapy.shape[0]/-hn)
			yP2 = numpy.round((numpy.array([y[1] for y in self.nodehist]) - (wn/2)) * mapy.shape[1]/-wn)
			for i in range(len(xP2)):
				cv2.circle(mapy,(int(yP2[i]),int(xP2[i])),4,(0, 255, 255),-1)

			xP = numpy.round((numpy.array([x[0] for x in temp]) - (hn/2)) * mapy.shape[0]/-hn)
			yP = numpy.round((numpy.array([y[1] for y in temp]) - (wn/2)) * mapy.shape[1]/-wn)
			for i in range(len(xP)):
				cv2.circle(mapy,(int(yP[i]),int(xP[i])),4,(255,255,0),-1)

			cv2.imshow("map", mapy)
			cv2.waitKey(5)

			#move to nearest co:
			xval = numpy.array([x[0] for x in temp])
			yval = numpy.array([y[1] for y in temp])
			xstart = self.pos[0]
			ystart = self.pos[1]
			#calculate the euclidian distance from current goal to each node
			edist = numpy.sqrt((xval-xstart)**2+(yval-ystart)**2)
			next_id = numpy.argmin(edist)
			self.move(temp[next_id][0], temp[next_id][1])
#			print"checking: " + str(temp[next_id][0]) + ", " + str(temp[next_id][1]) + " ..."
			self.wait()
			
			#spin 360 at current co (x)
			t0 = time()
			t1 = time()
			t2 = time()
			t3 = time()
#			print self.found
			
			while ((t1-t0 < 7) and (t2-t0 < 20) and (t3-t0 < 25)):
				(img, mask, r, yel, g, b, self.found) = self.image_callback(self.imageobj)
			
				M = cv2.moments(mask)
				h, w, _ = img.shape

			#if there are lost nodes
				if (not([self.found[k] == True for k in self.found] == [True]*4)):
					
				#if the mask is empty then spin
					if numpy.all(mask[240, :] == 0):
						t1 = time()
						t.angular.z = -1
						self.velPub.publish(t)			
					
					#if an object is focused
					elif mask[239,319]:
						t2 = time()
					
						#if dist < 0.5
						if numpy.isnan(self.dist):
							self.colour_move(-0.25, self.pos[2])
							sleep(0.5)
					
						#if the object is  0.5 <dist <1m  away
						elif self.dist < 1.1 or numpy.isnan(self.dist):
				
							#find the colour of the object and set it to found
							if r[239, 319] and not self.found["red"]:
								print "Found Red"	
								self.found["red"] = True
								Rend = timedelta(seconds=int(time()-start)) 
								print "   Red Time | " + str(Rend)
								break	
							elif yel[239, 319] and not self.found["yellow"]:
								print "Found Yellow"	
								self.found["yellow"] = True
								Yend = timedelta(seconds=int(time()-start)) 
								print "Yellow Time | " + str(Yend)	
								break		
							elif g[239, 319] and not self.found["green"]:
								print "Found Green"	
								self.found["green"] = True
								Gend = timedelta(seconds=int(time()-start)) 
								print " Green Time | " + str(Gend)	
								break	
							elif b[239, 319] and not self.found["blue"]:
								print "Found Blue"	
								self.found["blue"] = True	
								Bend = timedelta(seconds=int(time()-start)) 
								print "  Blue Time | " + str(Bend)
								break

#						if object is >1m away
						else:
							self.colour_move(0.5,0)
							sleep(0.5)
					
#					otherwise spin till the object is in the middle of the screen & move forward
					else:
							t3 = time()
				
							maskL = numpy.sum(mask[239, 0:250])
							maskC = numpy.sum(mask[239, 250:400])
							maskR = numpy.sum(mask[239, 400:650])
					
							if numpy.argmax([maskL, maskC, maskR]) == 0:
								self.colour_move(0,0.2)
								sleep(0.3)
								cv2.waitKey(3)
							elif numpy.argmax([maskL, maskC, maskR]) == 1:
								self.colour_move(0.5,0)
								sleep(0.5)
								cv2.waitKey(3)
							elif numpy.argmax([maskL, maskC, maskR]) == 2:

								self.colour_move(0,-0.2)
								sleep(0.3)
								cv2.waitKey(3)
					
#				if spin finished, current node complete
			else:
#				print("spin finished")
				temp[next_id][2] = True
			
				if (not([self.found[k] == True for k in self.found] == [True]*4)) and numpy.all(numpy.array(co)[:,2] == True):
					self.plot = self.plot * 1.8
					self.mindy = self.mindy * 0.8
					#co = self.new_node(int(plot), float(mindy))
					co = self.new_node(int(argv[1]), float(argv[2]))
					print " || " + str(timedelta(seconds=int(time()-start)))
				
		else:
			end = timedelta(seconds=int(time()-start)) 
			print " Total Time | " + str(end)
			print("___________________________________________________________________________________________________________________")
			

			#OUTPUT PATH TO mapy
			xP2 = numpy.round((numpy.array([x[0] for x in self.path]) - (hn/2)) * mapy.shape[0]/-hn)
			yP2 = numpy.round((numpy.array([y[1] for y in self.path]) - (wn/2)) * mapy.shape[1]/-wn)
			for i in range(len(self.path)):
				cv2.circle(mapy,(int(yP2[i]),int(xP2[i])),1,(150,255,70),-1)
			if not os.path.exists('routes/'): os.mkdir('routes/')
			cv2.imwrite(str("routes/path_" + str(argv[3]) + ".png"), mapy)

			exit()

#######################################################################################################################################
cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()

#######################################################################################################################################
