# -*- coding: utf-8 -*-
"""
quick script for setting up the remote API for CopSim
#simRemoteApi.start(19999) should be sent in the CopSim console first
"""

import sim

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
    