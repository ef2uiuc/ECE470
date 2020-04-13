# -*- coding: utf-8 -*-
import sim
from remote import clientID

def getHandles(objectName):
    opm = sim.simx_opmode_blocking
    handle = sim.simxGetObjectHandle(clientID,objectName,opm)
    return handle[1]