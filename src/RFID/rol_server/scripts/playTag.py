#!/usr/bin/env python


#test tag was 300833B2DDD9014000000014

import rospy
import tf
import math

from std_msgs.msg import String
from rfid_node.msg import  TagReading

#import TagModel

class la_clase():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.tf = tf.TransformListener()


        rospy.Subscriber('/lastTag', TagReading, self.tagCallback)

        rospy.spin()


    def tagCallback(self, data):
        data2=TagReading ()
        txPower = 20.0
        txLoss = math.pow( 10.0,  (txPower - data.rssi + 30 ) / 10.0   )
        # 4 + data.rssi/10.0
        weight_inc=txLoss

        lowProb = 0.001 * weight_inc
        midProb = 0.01 * weight_inc
        highProb = 0.1 * weight_inc

        print "RSSI: ", data.rssi
        print "losses: ", txLoss


# Main function.
if __name__ == '__main__':


    rospy.init_node('lalala')

    #rospy.loginfo("")


    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        lalala=la_clase()
    except rospy.ROSInterruptException:
        pass



