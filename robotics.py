# -*- coding: utf-8 -*-
import numpy as np
import sim
from scipy.linalg import expm
from remote import clientID

def getHandles(objectName):
    opm = sim.simx_opmode_blocking
    handle = sim.simxGetObjectHandle(clientID,objectName,opm)
    return handle[1]

arm = getHandles('UR3')
gripper = getHandles('BaxterVacuumCup')
joints = []
for i in range(1,7):
    joints.append(getHandles('UR3_joint'+str(i)))
  

class robot:
    def __init__(self, clientID, arm, gripper, joints):
        self.clientID = clientID
        self.arm = arm
        self.gripper = gripper
        self.joints = joints
        self.theta = [1.570796, 0, 0, 0, 0, 0]
        self.thetadot = []
        self.poseInit = False
        self.lenInit = False
        self.q = []
       
        
#   Updates current joint angles. If hasn't been called, uses streaming. Otherwise, buffer mode
    def curPos(self):
        pose = []
        if self.poseInit == False:
            opm = sim.simx_opmode_streaming
        if self.poseInit == True:
            opm = sim.simx_opmode_buffer
        for joint in self.joints:
            pose.append(sim.simxGetJointPosition(self.clientID,joint,opm)[1])
            self.poseInit = True
        self.theta = pose
        return
    
#   Direct movement of angles. Temporary, for testing until inverse kinematics    
    def move(self, angles):
        opm = sim.simx_opmode_oneshot
        try:
            sim.simxPauseCommunication(self.clientID,True)
            for i in range(len(angles)):
                sim.simxSetJointTargetPosition(self.clientID,self.joints[i],angles[i],opm)
            sim.simxPauseCommunication(self.clientID,False)
            return
        except:
            print('Something went wrong with your joints and angles lists')
            print('Check to see if both lists are the same size')
            return
 #  Retrieves distances of joints relative to base. Setting up for velocity/inverse kinematics   
    def linkLen(self):
        q = []
        if self.lenInit == False:
            opm = sim.simx_opmode_streaming
        if self.lenInit == True:
            opm = sim.simx_opmode_buffer 
        for joint in self.joints:
            q.append(sim.simxGetObjectPosition(self.clientID,joint,getHandles('UR3_link1_visible'),opm)[1])
            self.lenInit = True
        self.q = q
        return
        

ur3 = robot(clientID, arm, gripper, joints)

def MS():

    M = np.array([[0, 1, 0, -0.22939854860305786],
                  [0, 0, 1, 0],
                  [1, 0, 0, 0.5466200113296509],
                  [0, 0, 0, 1]]) #End-effector frame relative to base frame in zero-position
    w1 = np.array([0, 0, 1])
    w2 = np.array([0, 1, 0])
    w3 = np.array([0, 1, 0])
    w4 = np.array([0, 1, 0])
    w5 = np.array([0, 0, 1])
    w6 = np.array([0, 1, 0])
    q1 = np.array([-1.4901161193847656e-08, -5.960464477539063e-08, 0.10447296500205994])
    q2 = np.array([2.7164816856384277e-05, -0.11167824268341064, 0.10882195830345154])
    q3 = np.array([-0.03431794047355652, -0.11178913712501526, 0.35005152225494385])
    q4 = np.array([-0.0640682578086853, -0.11188384890556335, 0.5612159967422485])
    q5 = np.array([-0.08382469415664673, -0.11261004209518433, 0.6430813074111938])
    q6 = np.array([-0.08401036262512207, -0.11194229125976562, 0.6441996097564697])
    
    
    #calculate screw axis given omega and q
    def axis(w,q):
        v = np.cross(-w,q)
        axis = np.array([w[0],w[1],w[2],v[0],v[1],v[2]])
        return axis
    S1 = axis(w1,q1)
    S2 = axis(w2,q2)
    S3 = axis(w3,q3)
    S4 = axis(w4,q4)
    S5 = axis(w5,q5)
    S6 = axis(w6,q6)
    
    #arrange screw axis in skew symmetric array
    def skew(axis):
        skew = np.array([[0, -axis[2], axis[1], axis[3]],
					     [axis[2], 0, -axis[0], axis[4]],
					     [-axis[1], axis[0], 0, axis[5]],
					     [0, 0, 0, 0]])
        return skew
    Sb1 = skew(S1)
    Sb2 = skew(S2)
    Sb3 = skew(S3)
    Sb4 = skew(S4)
    Sb5 = skew(S5)
    Sb6 = skew(S6)
    
    #calculates the homogenous transformation matrix from robot base frame to end-effector frame
    T01 = np.matmul(expm(Sb1*ur3.theta[0]),expm(Sb2*ur3.theta[1]))
    T12 = np.matmul(T01,expm(Sb3*ur3.theta[2]))
    T23 = np.matmul(T12,expm(Sb4*ur3.theta[3]))
    T34 = np.matmul(T23,expm(Sb5*ur3.theta[4]))
    T45 = np.matmul(T34,expm(Sb6*ur3.theta[5]))
    T = np.round(np.matmul(T45,M),5)
    return T

def main():
    #initialize streaming
    ur3.linkLen()
    ur3.curPos()
    
    #now reads values
    #first go, just the zero-position
    ur3.move([np.pi/2, 0, 0, 0, 0, 0])
    ur3.linkLen()
    ur3.curPos()
    T = MS()
    print(T)
    input('Enter to continue')
    
    #second go
    ur3.move([-np.pi/2+2*np.pi, 3*np.pi/7, np.pi/6, np.pi/2, 0, np.pi/4])
    ur3.linkLen()
    ur3.curPos()
    T = MS()
    print(T)
    input('Enter to continue')
        
    #pickup block and return to home
    ur3.move([np.pi/4, -np.pi/2, 0, 0, 0, 0])
    ur3.linkLen()
    ur3.curPos()
    T = MS()
    print(T)
    
    return

main()