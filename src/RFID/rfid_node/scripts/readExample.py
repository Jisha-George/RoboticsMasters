#!/usr/bin/env python

import rfid
import time
import sys

#  message is a string containing 
#           0:tagID:rssi:phase:frequency:timestampHigh:timestampLow
#     e.g.  0:300833B2DDD9014000000003:-75:104:911750:339:2908467775i
def rfidCallback(message):
	
	msg = str(message)
	fields = msg.split(':')

	tagID = fields[1]
	rssi = fields[2]
	phase = fields[3]

	print "Detected " + tagID +  " tag with RSSI "  + rssi  +  " and phase " + phase

if __name__ == "__main__":

	if (len(sys.argv)!=2):
		print 'Usage:'
		print  '%s [reader dev]' % sys.argv[0]
		print 'E.G.:'
		print  '%s /dev/rfid' % sys.argv[0]
		sys.exit()
	
	readerDev  = sys.argv[1]
	rfid.init()
	reader = rfid.startReader("tmr://%s" % readerDev, rfidCallback)

	_ = raw_input()

	rfid.stopReader(reader)
	rfid.close()
	
