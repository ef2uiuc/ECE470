# -*- coding: utf-8 -*-
import sim
import numpy as np
import robotics
from getHandles import getHandles
from remote import clientID
import time


arm = getHandles('UR3')
gripper = getHandles('BaxterVacuumCup')
joints = []
for i in range(1,7):
    joints.append(getHandles('UR3_joint'+str(i)))
ur3 = robotics.robot(clientID,arm,gripper,joints)


print('Please enter an xw, yw, and zw value respectivelt:')
print('xw = ')
xw = float(input())
print('yw = ')
yw = float(input())
print('zw = ')
zw = float(input())
thetas = ur3.invk(xw,yw,zw)
ur3.move(thetas)
print('The end-effector should be at')
d = ur3.fork(thetas)
print(d)

print('Please press enter to continue')
input()


ur3.move([0]*6)

above = ur3.invk(0, -.3 , .4)
ur3.move(above)
time.sleep(2)
target = ur3.invk(0, -.5, 0.170)
ur3.move(target)
time.sleep(2)
sim.simxSetIntegerSignal(clientID,'BaxterVacuumCup_active',1,sim.simx_opmode_oneshot)
time.sleep(2)
thetas = ur3.invk(0, .5, 0.170)
ur3.move(thetas)
time.sleep(2)
sim.simxSetIntegerSignal(clientID,'BaxterVacuumCup_active',0,sim.simx_opmode_oneshot)
print('Target dropped!')
