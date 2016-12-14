#!/usr/bin/env python

import amp1394


eth = amp1394.Eth1394Port(0)
bd1 = amp1394.AmpIO(1)
eth.AddBoard(bd1)

import ipdb; ipdb.set_trace()
