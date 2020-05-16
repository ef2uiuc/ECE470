# -*- coding: utf-8 -*-
import sim
import numpy as np
import robotics
import greedy
from getHandles import getHandles
from remote import clientID
import time

#Initilization of values
arm = getHandles('UR3')
gripper = getHandles('BaxterVacuumCup')
joints = []
for i in range(1,7):
    joints.append(getHandles('UR3_joint'+str(i)))
ur3 = robotics.robot(clientID,arm,gripper,joints)

visSens = getHandles('Vision_sensor')
conveyor = getHandles('customizableConveyor')
noBins = 3
names = ['red', 'blue', 'green', 'purple', 'white','Empty']
costs = [100, 50, 20, 5, 200, 0]
blockMats = greedy.material(names,costs)
greedyCount = greedy.greedyAlgo(blockMats, noBins)

#CoppeliaSim returns colors in strange formats
colors = {'red':[-103,32,32], 'blue':[32, 32, -103], 'green':[32,-103,32], 'purple':[-103, 32, -103], 'white':[-103, -103, -103]}

#waits for a block to appear, reads its color, and stops the conveyor and more blocks from spawning
def acquireColor():
    error, res, image = sim.simxGetVisionSensorImage(clientID, visSens, 0, sim.simx_opmode_streaming)
    while sim.simxGetConnectionId(clientID) != -1:
        error, res, image = sim.simxGetVisionSensorImage(clientID, visSens, 0, sim.simx_opmode_buffer)
        if error == sim.simx_return_ok:     
            for i, color in enumerate(list(colors.values())):
                if image == color:
                    sim.simxSetFloatSignal(clientID,'conveyorVelocity',0,sim.simx_opmode_oneshot)
                    detectedColor = list(colors.keys())[i]
                    return detectedColor
                
#key positions for the ur3 arm to travel to
#home-y
home = [0]*6
#position is where a block should be
blockUp = [0, -.5, 0.22]
blockDown = [0,-.5, 0.15]
#the drop off bins respectively
binDrop = [[-.4, 0, .3],
            [.4, 0, .3],
            [0, .4, .3]]

def pickUp():
    detectedColor = acquireColor()
    ur3.move(ur3.invk(blockUp))
    time.sleep(1)
    ur3.move(ur3.invk(blockDown))
    time.sleep(1)
    sim.simxSetIntegerSignal(clientID,'BaxterVacuumCup_active',1,sim.simx_opmode_oneshot)
    time.sleep(1)
    sim.simxSetFloatSignal(clientID,'conveyorVelocity',0.2,sim.simx_opmode_oneshot)
    binToGo = greedyCount.update(detectedColor)
    ur3.move(ur3.invk(binDrop[binToGo]))
    time.sleep(2)
    sim.simxSetIntegerSignal(clientID,'BaxterVacuumCup_active',0,sim.simx_opmode_oneshot)
    time.sleep(1)
    ur3.move(home)
    return


n = 100
i = 0
while i < n:
    pickUp()
    i +=1
    