#!/usr/bin/env python

import yaml
import rospy
import operator
from std_msgs.msg import String
#from rol_server.srv import findObject, findObjectRequest, findObjectResponse
from hmi_bridge.srv import findObject, findObjectRequest, findObjectResponse


# Node class.
class ProbHandler():
    def __init__(self,top,obj):
        self.object=obj
        self.topic=top
        self.lastLoc=' '
        rospy.Subscriber(self.topic, String, self.probCallback)

    def probCallback(self,data):
        self.rawData=data.data

    def setLocations(self, locs):
        self.locs=locs

    def getLastLoc(self):
        sublocSeparator='_'
        if sublocSeparator in self.lastLoc:
            temp=self.lastLoc.split(sublocSeparator)
            self.lastLoc=temp[0]

        return self.lastLoc

    def getProbs(self):
        try:
            splitData=self.rawData.split(',')
            self.probs = splitData[3::2]
            self.locs = splitData[2::2]
            self.lastLoc=splitData[0]
        except AttributeError:
            self.probs=[0]*len(self.locs)


        probDict=dict(zip(self.locs, self.probs))

        return probDict

class rol_server():

    def percentFormat(self, strNum):

        return  '{:.2f}'.format(100. *float( strNum))


    def performLoc(self,findObjectReq):
        receivedAction = str.lower(findObjectReq.action)
        receivedPayload = str.lower(findObjectReq.payload)
        rospy.logdebug("Received service request.")
        rospy.logdebug("Action: %s", receivedAction)
        rospy.logdebug("Payload: %s",receivedPayload)

        if   receivedAction== 'list':
            ans=self.performListAct(receivedPayload )
        elif receivedAction== 'find':
            ans=self.performFindAct(receivedPayload )
        elif receivedAction== 'accurate_find':
            ans=self.performAcFindAct(receivedPayload )
        elif receivedAction== 'accurate_prob':
            ans=self.performAcProbAct( receivedPayload )
        else:
            ans=self.createErrorResponse('Unknown action: '+ findObjectReq.action)

        self.rol_pub.publish(receivedAction+":"+receivedPayload)

        return ans



    def getRegion(self,subregion):
        sublocSeparator='_'
        ans=subregion
        if sublocSeparator in subregion:
            temp=subregion.split(sublocSeparator)
            ans=temp[0] 
        return ans



    def createFlatList(self):
        '''
        Creates a list of sublocations and locations without sublocations
        '''
        subLocResp=[]
        subLocResp=subLocResp+(self.locationsList)

        for sub in self.sublocationsList:
            reg =  self.getRegion(sub) 
            if reg in subLocResp:
                subLocResp.remove(reg)
            subLocResp.append(sub)
        return subLocResp



    def performListAct(self,payload):
        '''
        Returns a list of objects, locations or sublocations available
        :param payload: 'objects' , 'locations' or 'sublocations'
        :return: filled srv response with proper list
        '''

        if   payload == 'objects':
            ans=self.createOkResponse(self.objectsList)
        elif payload == 'locations':
            ans=self.createOkResponse(self.locationsList)
        elif payload == 'sublocations':
            ans=self.createOkResponse(self.createFlatList())
        else:
            ans=self.createErrorResponse('Unknown payload for list action:'+ payload)
        return ans

    def performAcFindAct(self, obj):
        if obj in self.objectsList:
            probs = self.getAcProbs(obj)
            ans = self.createOkResponse(probs)
        else:
            ans = self.createErrorResponse('Unknown object to accurately find:' + obj)
        return ans

    def performAcProbAct(self, obj):
        if obj in self.objectsList:
            probs = self.getAllProbs(obj)
            ans = self.createOkResponse(probs)
        else:
            ans = self.createErrorResponse('Unknown object to accurately probs:' + obj)
        return ans



    def performFindAct(self, obj):

        if obj in self.objectsList:
            probs = self.getObjectProbs(obj)
            ans=self.createOkResponse(probs)
        else:
            ans = self.createErrorResponse('Unknown object to find:' + obj)

        return ans

    def createOkResponse(self, data):
        '''
        Embeds a sequence of strings to a ROL srv response
        :param data: sequence to be joined and stored in srv response
        :return: filled srv response
        '''

        separator = ","

        ans = findObjectResponse()
        ans.response = separator.join(data)
        ans.wasOk = True
        ans.feedback = ''

        return ans

    def createErrorResponse(self, data):
        '''
        Returns a srv response describing an error
        :param data: error description
        :return: filled srv response
        '''

        ans = findObjectResponse()
        ans.response = ''
        ans.wasOk = False
        ans.feedback = data

        return ans


    def getObjectProbs(self,obj):
        ans = []

        pH=self.probHandlerList[obj]
        probDict=pH.getProbs()
        fullProbs = sorted(probDict.items(), key=operator.itemgetter(1), reverse=True)

        for z,p in fullProbs:
            #if z!=lastL:
                if z in self.locationsList:
                    if (float(p)>=(self.minProb/100.0)):
                        ans.append(z)
                        ans.append(self.percentFormat(p))
                else:
                    rospy.logdebug('Region is:   ' )


        lastL=pH.getLastLoc()
        if (lastL != ' '):
            ans.append(lastL)
            #ans.append('kitchen')
        else:
            ans.append(self.locationsList[0])
            #ans.append('kitchen')
        ans.append('-1')

        return ans

    def getAcProbs(self,obj):
        ans=[]

        #get prob dict
        probDict = (self.probHandlerList[obj]).getProbs()

        #get most probable location
        fullProbs = sorted(probDict.items(), key=operator.itemgetter(1), reverse=True)

        bestRegion,bestProb = fullProbs[0]

        #rospy.logdebug('Region is:   ' + bestRegion)
        #rospy.logdebug('Probability: ' + str(bestProb))

        #get a probabilities dict from this location
        bestSublocationsDict=dict()
        for subReg in self.sublocationsList:
                if bestRegion in subReg:
                        rospy.logdebug('Subregion is: '+ subReg)
                        rospy.logdebug('Probability:  ' + probDict[subReg])
                        rospy.logdebug('Relative Pr:  ' + probDict[subReg])
                        if (float(bestProb)>0.0):
                           bestSublocationsDict[subReg]=str(float(probDict[subReg])/float(bestProb))
                        else:
                           bestSublocationsDict[subReg]=str(0.0)
    
        #parse a list of relative probabilities
        if not bestSublocationsDict:
            if (float(bestProb) >= (self.minProb / 100.0)):
                ans.append(bestRegion)
                ans.append(self.percentFormat(bestProb))
                #ans=','.join(ans)
        else:
            sortedList = sorted(bestSublocationsDict.items(), key=operator.itemgetter(1),reverse=True)
            for bestRegion,bestProb in sortedList:
                if (float(bestProb) >= (self.minProb / 100.0)):
                    ans.append(bestRegion)
                    ans.append(self.percentFormat(bestProb))
                    #ans=[i for tup in sortedList for i in tup]


        return ans


    def getAllProbs(self,obj):
        ans=[]

        # get list of locations to be reported
        locations = self.createFlatList()
        
        # get prob dict
        probDict = (self.probHandlerList[obj]).getProbs()

        # sort by probability
        fullProbs = sorted(probDict.items(), key=operator.itemgetter(1), reverse=True)

        for location,prob in fullProbs:
                if (float(prob) >= (self.minProb / 100.0)):
                    if (location in locations):
                        ans.append(location)
                        ans.append(self.percentFormat(prob))
                    

        return ans

   
    def rosSetup(self):
        self.probHandlerList=dict()
        self.regions_file=''
        self.rolTopic=rospy.get_param('rolTopic','rol_requests')
        self.minProb = float(rospy.get_param('~minProb', 0.0))

        listOfTopics = rospy.get_published_topics()
        self.loadLocations()
        
        for tup in listOfTopics:
            if ('probs' in tup[0]) and ('std_msgs/String' in tup[1]):
                foundTopic=tup[0]
                nodeName=foundTopic[0:-5]
                
                objectName=rospy.get_param(nodeName+'object')
                self.objectsList.append(objectName)

                prH = ProbHandler(foundTopic,objectName)
                prH.setLocations(self.locationsList)
                self.probHandlerList[objectName] = (prH)

    def probCallback(self,data):
        probsString=data.data
                   
    #called by rossetup, to load locations list.
    def loadLocations(self):
        self.yDict = rospy.get_param('/mmap/zoi/submap_0/')
        for point in self.yDict:
            regionTmp = self.yDict[point][1]
            if not ('_' in regionTmp):            
               if not (regionTmp in self.locationsList):  
                   self.locationsList.append(regionTmp)
            else:
               if not (regionTmp in self.sublocationsList):
                   self.sublocationsList.append(regionTmp)

    def loadLocationsOLD(self):
        self.yDict = rospy.get_param('Regions')
        #print self.yDict
        for region in self.yDict:
            self.locationsList.append(region['name'])
            if region.has_key('subregions'):
                for subR in region['subregions']:
                    self.sublocationsList.append(subR['name'])

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        rospy.loginfo("Advertising RFID object location service ")


        #create some storage atributes
        self.objectsList = []
        self.locationsList=[]
        self.sublocationsList = []

	#sleep for 10 seconds to be sure tag nodes are running
	rospy.sleep(10.)

        # get parameters from ros: find out tracked objects, locations, subscribe to their probs trackers...
        self.rosSetup()

        # create publisher for service requests
        self.rol_pub = rospy.Publisher(self.rolTopic, String, queue_size=10)


        # start service callback
        self.s=rospy.Service('rol_server', findObject, self.performLoc)


        rospy.loginfo("Ready...")
        rospy.spin()


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('rol_server')#, log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        r_s = rol_server()
    except rospy.ROSInterruptException: pass
