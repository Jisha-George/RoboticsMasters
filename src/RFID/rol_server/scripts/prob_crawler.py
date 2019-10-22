#!/usr/bin/env python
''' Gathers ROL server data for further analysis'''


import rospy
import operator
from std_msgs.msg import String
import pickle
from hmi_bridge.srv import findObject, findObjectRequest, findObjectResponse
from rfid_node.msg import TagReading


class object_prob():
    def __init__(self, _objectName, _tagID,_fileName):
        self.objectName=_objectName
        self.tagID=_tagID
        # /grid_390200010000000000000000/rfid_grid_map_node/probs
        probTopicName = '/grid_' + self.tagID+ '/rfid_grid_map_node/probs'
        self.subscriber = rospy.Subscriber(probTopicName, String, self.objectProbCallback)

        detectTagTopic='/lastTag'
        self.numDetects=0
        self.tag_subscriber = rospy.Subscriber(detectTagTopic, TagReading, self.tagCallback)


        self.entryList=list()
        self.file= open(_fileName, "wb")
        rospy.on_shutdown(self.shutdown)


    def tagCallback(self,msg):
        if msg.ID == self.tagID:
            self.numDetects=self.numDetects+1

    def objectProbCallback(self, data):
        elements = data.data.split(',')
        locations = elements[::2]
        strProbs = elements[1::2]

        # REMOVE LAST TAG DETECTION
        locations.pop(strProbs.index("-1"))
        strProbs.pop(strProbs.index("-1"))

        elapsedTime = rospy.get_time()
        numProbs = [float(s) for s in strProbs]

        result=dict(zip(locations, numProbs))
        result["time"]=elapsedTime
        result["detects"]=self.numDetects
        #rospy.logdebug(self.objectName+': '+str(result))

        self.entryList.append(result)

    def shutdown(self):
        pickle.dump(self.entryList, self.file)


# Node class.
class prob_crawler():




    def rosSetup(self):
        try:
            self.objectsDict = rospy.get_param('~Objects')
            self.saveRoute = rospy.get_param('~saveRoute')
            self.filePrefix = rospy.get_param('~fileNamePrefix')
        except KeyError:
            rospy.logfatal('Cant find a parameter!!')

    def subscribeToObjects(self):
        self.objectList=list()
        for key, value in self.objectsDict.iteritems():
            pickleFileName=self.saveRoute+self.filePrefix+key+'.p'
            obj_prob = object_prob(key,value,pickleFileName)
            self.objectList.append(obj_prob)


    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        rospy.loginfo("Advertising data crawler ")

        #create class atributes

        #wait for rol_server to be running

        # get parameters from ros
        self.rosSetup()

        # create topic subscribers
        self.subscribeToObjects()

        rospy.loginfo("["+rospy.get_name()+"] Ready...")
        rospy.spin()


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('prob_crawler')#, log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        pc = prob_crawler()
    except rospy.ROSInterruptException: pass
