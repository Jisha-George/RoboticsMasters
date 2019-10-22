#!/usr/bin/env python


import rospy
import operator
from std_msgs.msg import String
from std_msgs.msg import Float32
from rfid_node.msg import TagReading



# Node class.
class tag_discovery():


    def loadROSParams(self):
        self.detected_tag_topic_name = rospy.get_param(
            '~detected_tag_topic_name', '/lastTag')
        self.tag_discovery_coverage_topic_name = rospy.get_param(
            'tag_discovery_coverage_topic_name', '/tag_coverage')

    def initROS(self):
        # topic publishers
        self.tag_discovery_coverage_pub = rospy.Publisher(self.tag_discovery_coverage_topic_name, Float32, queue_size=1)

        # service clients
        
        # service servers
        
        # topic subscribers and other listeners
        self.detected_tag_sub = rospy.Subscriber(self.detected_tag_topic_name, TagReading, self.detected_tag_callback, queue_size=1)
   
    def initTagIDSet(self):
        self.tagIDSet=set()
        # Monitored tags have each one a prob topic and hence a topic which follows the convention:
        # /grid_[TAG ID HERE]/rfid_grid_map_node/probs

        rospy.Rate(1.0/8.0).sleep() # Sleep for 8 secs before running

        while (len(self.tagIDSet)==0):
            listOfTopics = rospy.get_published_topics()
            for tup in listOfTopics:
                if ('probs' in tup[0]) and ('std_msgs/String' in tup[1]):
                    foundTopic=tup[0]
                    tag_id = foundTopic[6:-25]
                    self.tagIDSet.add(tag_id)
            
            if (len(self.tagIDSet)==0):
                rospy.logerr("Node [" + rospy.get_name() + "] There are no tag probs topics... (/grid_[TAG ID HERE]/rfid_grid_map_node/probs) waiting 5 secs.!!")
                rospy.sleep(5.)
        

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params and atributes
        self.initTagIDSet()        
        self.detectedTagIDSet=set()

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] entering spin...")

        rospy.spin()
    
    def detected_tag_callback(self, tag_msg):
        tag_id = tag_msg.ID
        if tag_id in self.tagIDSet:
            self.detectedTagIDSet.add(tag_id)
        else:
            rospy.logwarn("Node [" + rospy.get_name() + "] Detected tag not under survey ("+ tag_id +")")
        
        self.tag_discovery_coverage_pub.publish( float(len(self.detectedTagIDSet)) / float(len(self.tagIDSet))  )


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('tag_discovery_node')#, log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        r_s = tag_discovery()
    except rospy.ROSInterruptException: pass
