# -*- coding: utf-8 -*-
"""
Extremely basic hard-coded movement commands for testing 
"""


#Starting Up
import sim
import numpy as np
import time

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')

#Movement Parameters
tVel = 100


#Setting a random pose
randPose = np.radians(np.random.uniform(-90,90,(1,6)))
randPose = randPose.tolist()

#Setting up handles
j1 = sim.simxGetObjectHandle(clientID,'UR3_joint1',sim.simx_opmode_blocking )[1]
j2 = sim.simxGetObjectHandle(clientID,'UR3_joint2',sim.simx_opmode_blocking )[1]
j3 = sim.simxGetObjectHandle(clientID,'UR3_joint3',sim.simx_opmode_blocking )[1]
j4 = sim.simxGetObjectHandle(clientID,'UR3_joint4',sim.simx_opmode_blocking )[1]
j5 = sim.simxGetObjectHandle(clientID,'UR3_joint5',sim.simx_opmode_blocking )[1]
j6 = sim.simxGetObjectHandle(clientID,'UR3_joint6',sim.simx_opmode_blocking )[1]
sensor = sim.simxGetObjectHandle(clientID,'Proximity_sensor',sim.simx_opmode_blocking )[1]
prox = sim.simxReadProximitySensor(clientID,sensor,sim.simx_opmode_streaming)

#Send movement poses to robot
sim.simxPauseCommunication(clientID,True)
sim.simxSetJointTargetPosition(clientID,j1,randPose[0][0],sim.simx_opmode_oneshot)
sim.simxSetJointTargetPosition(clientID,j2,randPose[0][1],sim.simx_opmode_oneshot)
sim.simxSetJointTargetPosition(clientID,j3,randPose[0][2],sim.simx_opmode_oneshot)
sim.simxSetJointTargetPosition(clientID,j4,randPose[0][3],sim.simx_opmode_oneshot)
sim.simxSetJointTargetPosition(clientID,j5,randPose[0][4],sim.simx_opmode_oneshot)
sim.simxSetJointTargetPosition(clientID,j6,randPose[0][5],sim.simx_opmode_oneshot)
sim.simxPauseCommunication(clientID,False)

#Updates and reads Data from a Proximity Sensor Data
time.sleep(.5)
sim.simxPauseCommunication(clientID,True)
prox = sim.simxReadProximitySensor(clientID,sensor,sim.simx_opmode_streaming)
sim.simxPauseCommunication(clientID,False)
print('The sensor reads', prox[1])