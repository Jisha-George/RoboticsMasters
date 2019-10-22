#!/usr/bin/env python
import rfid 
import sys 
import time
#from ctypes import *

def readCallback(message):
    try:
        #print message.split(':')[1:3]
        pass
    except:
        pass


def doIt(reader):
    timeoutMili = 20
    data = rfid.readOnce(reader, timeoutMili,100)
    tagList = data.split(';')

    for tag in tagList[0:-1]: # tagList last entry is empty
        print tag.split(':')[1:3]
#    print data

        
if __name__ == "__main__":
    rfid.init()
    readerURI="tmr:///dev/ttyACM0"
    if len(sys.argv)>1:
        readerURI="tmr://"+sys.argv[1]
                
    reader = rfid.startReader(readerURI, readCallback)
    rfid.stopReader(reader)
    
    doIt(reader)
    time.sleep(1)
    doIt(reader)
    time.sleep(1.5)

    rfid.close()
