#!/usr/bin/env python

import rfid
import time
import sys

def rfidCallback(message):
	return

if __name__ == "__main__":

    if (len(sys.argv)!=4):
	print 'Usage:'
        print  '%s [rfid device] [oldEPC] [newEPC]' % sys.argv[0]
	print 'E.G.:'
        print  '%s /dev/rfid 300833B2DDD9014100000000 39000001000000006b657973' % sys.argv[0]
	sys.exit()
    
    readerDev  = sys.argv[1]
    oldEpcData = sys.argv[2]
    newEpcData = sys.argv[3]

    if (len(oldEpcData)!=len(newEpcData)):
	print  'EPC ids have different length '
	print  'length(%s)=%d' % (oldEpcData,len(oldEpcData))
	print  'length(%s)=%d' % (newEpcData,len(newEpcData))
	sys.exit()

    rfid.init()


    reader = rfid.startReader("tmr://%s" % readerDev, rfidCallback)

    #Stop from reading more tags
    rfid.stopReader(reader)

    error = True
    # just write first tag you get!   
    print "about to rename %s as %s" % ( oldEpcData, newEpcData)
    print "\n\n "
    while error:
           error=(rfid.renameTag(reader, oldEpcData, newEpcData,len(oldEpcData))!=0)	
    print "bye!"
    rfid.close()
	
