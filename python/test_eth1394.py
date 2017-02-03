#!/usr/bin/env python

import amp1394
import time
import numpy as np
import argparse
from random import randint
from amp1394 import bswap32, bswap16

def TestWriteQuadletBroadcast(fw):
    """Write Quadlet Broadcast"""
    wval = randint(1, 1000)
    fw.WriteQuadletBroadcast(0xffffff001000, 0x03, wval)
    nnodes = fw.GetNumOfNodes()
        

def TestQRead(eth, fw, bid):
    wval = randint(1, 1000)
    fw.WriteQuadlet(bid, 0x03, wval)
    ret, rval = eth.ReadQuadlet(bid, 0x03)
    print "wval = " + str(wval) + "  rval = " + str(rval)
    if (wval == rval):
        return True
    else:
        return False

def TestQWrite(eth, fw, bid):
    wval = randint(1, 1000)
    eth.WriteQuadlet(bid, 0x03, wval)
    ret, rval = fw.ReadQuadlet(bid, 0x03)
    print "wval = " + str(wval) + "  rval = " + str(rval)
    if (wval == rval):
        return True
    else:
        return False

def TestBRead(eth, fw, bid):
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

def TestBWrite(eth, fw, bid):
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
        time.sleep(0.001)
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

def TestContinuous(eth, bid=0, len=100):
    print "Continuous Write/Read"
    start = time.time()
    for i in range(0,len):
        twait = 0.00001
        wval = i % 65535
        wret = eth.WriteQuadlet(bid, 0x03, wval)
        # time.sleep(twait)
        [rret, val] = eth.ReadQuadlet(bid, 0x03)
        # time.sleep(twait)
        if not wret or not rret:
            print "write = " + str(wval) + "  read = " + str(val)
        elif wval != val:
            print "wval = " + str(wval) + "  rval = " + str(val)

    print "Time elapsed for " + str(len) + " W/R = " + \
          str( time.time() - start ) + " sec"

def ReceivePacket(bd):
    # bd: firewire board
    _, RXFCTR = bd.ReadKSZ8851Reg(0x9C)
    _, RXFHSR = bd.ReadKSZ8851Reg(0x7C)
    _, RXFHBCR = bd.ReadKSZ8851Reg(0x7E)
    bd.WriteKSZ8851Reg(0x86, 0x5000)
    _, RXQCR = bd.ReadKSZ8851Reg(0x82)
    bd.WriteKSZ8851Reg(0x82, RXQCR|0x0008)

    numWords = (((RXFHBCR&0x0FFF) + 3) & 0xFFFC) >> 1
    print "num bytes = " + str(RXFHBCR)
    print "num words = " + str(numWords)

    # skip
    dummy = bd.ReadKSZ8851DMA()
    status = bd.ReadKSZ8851DMA()
    length = bd.ReadKSZ8851DMA()
    
    dataStr = ''
    for i in range(0, numWords):
        _, data = bd.ReadKSZ8851DMA()
        dataStr = dataStr + format(bswap16(data), '04X') + ' '
        if i == 3 or i == 6:
            dataStr = dataStr + '\n'
        if (i > 7) and (i+2)%4==0:
            dataStr = dataStr + '\n'
    print dataStr

    _, RXQCR = bd.ReadKSZ8851Reg(0x82)
    bd.WriteKSZ8851Reg(0x82, RXQCR&0xFFF7)



# Test setup
#  - connect two boards (bid = 0 & 3)
#  - test quadlet read/write
#    - on directly connected board
#    - on firewire indirectly connected board
#  - test block read/write

def PrintEthDebug(fwbd):
    status = fwbd.ReadKSZ8851Status()
    dbgstr = ""
    if (status&0x4000): dbgstr = dbgstr + "error "
    if (status&0x2000): dbgstr = dbgstr + "initOK "
    if (status&0x1000): dbgstr = dbgstr + "initReq "
    if (status&0x0800): dbgstr = dbgstr + "ethIoErr "
    if (status&0x0400): dbgstr = dbgstr + "PacketErr "
    if (status&0x0200): dbgstr = dbgstr + "DestErr "
    if (status&0x0100): dbgstr = dbgstr + "qRead "
    if (status&0x0080): dbgstr = dbgstr + "qWrite "
    if (status&0x0040): dbgstr = dbgstr + "bRead "
    if (status&0x0020): dbgstr = dbgstr + "bWrite "
    if (status&0x0010): dbgstr = dbgstr + "multicast "
    if (status&0x0008): dbgstr = dbgstr + "KSZ-idle "
    if (status&0x0004): dbgstr = dbgstr + "ETH-idle "
    print "Eth dbg = " + dbgstr
    
    

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--board', default=0, type=int,
                        help="board id, default = 0")
    parser.add_argument('-f', '--firewire', default=False, help="firewire only mode")
    parser.add_argument('-t', '--test', default=False, help="enable basic test")
    args = parser.parse_args()

    print "===================="
    print "bid = " + str(args.board)
    bid = args.board

    if args.firewire:
        fw = amp1394.FirewirePort(0)
        bd1 = amp1394.AmpIO(bid)
        fw.AddBoard(bd1)
    else:
        fw = amp1394.FirewirePort(0)
        bd1 = amp1394.AmpIO(bid)
        fw.AddBoard(bd1)
        
        eth = amp1394.Eth1394Port(0)
        bd2 = amp1394.AmpIO(bid)
        eth.AddBoard(bd2)

    if args.test:
        TestQRead(eth, fw, bid)
        TestQWrite(eth, fw, bid)
        TestBRead(eth, fw, bid)
        TestBWrite(eth, fw, bid)

import ipdb; ipdb.set_trace()
