#!/usr/bin/env python

import amp1394
import time
import numpy as np
from random import randint
from amp1394 import bswap32

def TestWriteQuadletBroadcast(fw):
    """Write Quadlet Broadcast"""
    wval = randint(1, 1000)
    fw.WriteQuadletBroadcast(0xffffff001000, 0x03, wval)
    nnodes = fw.GetNumOfNodes()
        

def TestQRead(eth, fw):
    bid = 3
    wval = randint(1, 1000)
    fw.WriteQuadlet(bid, 0x03, wval)
    ret, rval = eth.ReadQuadlet(bid, 0x03)
    print "wval = " + str(wval) + "  rval = " + str(rval)
    if (wval == rval):
        return True
    else:
        return False

def TestQWrite(eth, fw):
    bid = 3
    wval = randint(1, 1000)
    eth.WriteQuadlet(bid, 0x03, wval)
    ret, rval = fw.ReadQuadlet(bid, 0x03)
    print "wval = " + str(wval) + "  rval = " + str(rval)
    if (wval == rval):
        return True
    else:
        return False

def TestBRead(eth, fw):
    bid = 3
    wval = [randint(1,1000), randint(1,1000), randint(1,1000)]
    fw.WriteBlock(bid, 0x1000, wval)
    ret, rval = eth.ReadBlock(bid, 0x1000, len(wval))
    if not ret:
        return ret
    print "wval = " + str(wval) + "  rval = " + str(rval)
    ispass = True
    for i in range(0, len(wval)):
        if (wval[i] != rval[i]):
            ispass = False
    return ispass

def TestBWrite(eth, fw):
    bid = 3
    wval = [randint(1,1000), randint(1,1000), randint(1,1000)]
    eth.WriteBlock(bid, 0x1000, wval)
    ret, rval = eth.ReadBlock(bid, 0x1000, len(wval))
    if not ret: return ret
    print "wval = " + str(wval) + "  rval = " + str(rval)
    ispass = True
    for i in range(0, len(wval)):
        if (wval[i] != rval[i]):
            ispass = False
    return ispass
    

def TestContinuousRead(eth, bid=3, len=100):
    print "Continuous read"
    
    hver = 0x514c4131
    success = 0
    readFailures = 0
    compareFailures = 0
    start = time.time()
    for i in range(1,len+1):
        ret = False
        rval = 0
        [ret, rval] = eth.ReadQuadlet(bid, 0x04)
        if not ret:
            print "failed to read quadlet"
            readFailures = readFailures + 1
        elif (rval != hver):
            print "Not a QLA1 board"
            compareFailures = compareFailures + 1
        else:
            success = success + 1

        # if (i%100 == 0):
        #     print "i = " + str(i) + "  success = " + str(success) + \
        #           "  read failures = " + str(readFailures) + \
        #           "  compare failures = " + str(compareFailures)

    print "Time elapsed for " + str(len) + " read = " + \
          str( time.time() - start ) + " sec"
            
    print "Done Total = " + str(len) + "  success = " + str(success) + \
          "  read failures = " + str(readFailures) + \
          "  compare failures = " + str(compareFailures)
    

def TestContinuousWrite(eth, bid=3, len=100):
    print "Continuous Write"

    start = time.time()
    # write
    for i in range(0,len):
        eth.WriteQuadlet(bid, 0x03, i)

    print "Time elapsed for " + str(len) + " write = " + \
          str( time.time() - start ) + " sec"

    # read
    [ret, rval] = eth.ReadQuadlet(bid, 0x03)
    print "i = " + str(i) + "  rval = " + str(rval)



# Test setup
#  - connect two boards (bid = 0 & 3)
#  - test quadlet read/write
#    - on directly connected board
#    - on firewire indirectly connected board
#  - test block read/write


bid = 3
fw = amp1394.FirewirePort(0)
bd1 = amp1394.AmpIO(bid)
fw.AddBoard(bd1)

eth = amp1394.Eth1394Port(0)
bd2 = amp1394.AmpIO(bid)
eth.AddBoard(bd2)


TestQRead(eth, fw)
TestQWrite(eth, fw)
TestBRead(eth, fw)
TestBWrite(eth, fw)


import ipdb; ipdb.set_trace()
