#!/usr/bin/env python

import Amp1394Python as amp1394
from Amp1394Python import bswap32

def TEST_ReadQuadlet(fw, eth):
    # bid = 1, addr = 0x04 (QLA1)
    bid = 1
    addr = 0x04
    ret_fw, data_fw = fw.ReadQuadlet(bid, addr)
    ret_eth, data_eth = eth.ReadQuadlet(bid, addr)
    if (not ret_fw):
        print 'Read Firewire failed'
    if (not ret_eth):
        print 'Read Ethernet failed'

    if (data_fw != data_eth):
        print 'Firewire & Ethernet read do not match'
        print 'Firewire data = ' + hex(data_fw)
        print 'Ethernet data = ' + hex(data_eth)
        return False
    else:
        print 'Passed test'
        return True
    pass


if __name__ == '__main__':
    bid = 3;
    eth = amp1394.EthUdpPort(0)
    bd1eth = amp1394.AmpIO(bid)
    eth.AddBoard(bd1eth)

    fw = amp1394.FirewirePort(0)
    bd1fw = amp1394.AmpIO(bid)
    fw.AddBoard(bd1fw)

    # Do something here

    fw.RemoveBoard(bid)
    eth.RemoveBoard(bid)

#try:
#    import ipdb; ipdb.set_trace()
#except ImportError:
#    print 'Failed to import ipdb'
