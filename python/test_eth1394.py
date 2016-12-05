#!/usr/bin/env python

import amp1394


eth = amp1394.Eth1394Port(0)
bd3 = amp1394.AmpIO(3)
eth.AddBoard(bd3)

import ipdb; ipdb.set_trace()
