# -*- coding: utf-8 -*-
'''
Contains mostly everything regarding kinematics/dynamics of robot
'''
import numpy as np
import sim
import math
from getHandles import getHandles
from scipy.linalg import expm
from remote import clientID



arm = getHandles('UR3')
gripper = getHandles('BaxterVacuumCup')
joints = []
for i in range(1,7):
    joints.append(getHandles('UR3_joint'+str(i)))

  
#Some helper functions used within the robot, for forward kinematics
def axis(w,q):
    v = np.cross(-w,q)
    axis = np.array([w[0],w[1],w[2],v[0],v[1],v[2]])
    return axis

def skew(axis):
    skew = np.array([[0, -axis[2], axis[1], axis[3]],
				[axis[2], 0, -axis[0], axis[4]],
				[-axis[1], axis[0], 0, axis[5]],
				[0, 0, 0, 0]])
    return skew

def get_MS():
    M = np.array([[0, 1, 0, 0],
                  [0, 0, 1, -.2577],
                  [1, 0, 0, 0.6942],
                  [0, 0, 0, 1]]) #End-effector frame relative to base frame in zero-position, retrieved from simulation positions
    w1 = np.array([0, 0, 1])
    w2 = np.array([0, -1, 0])
    w3 = np.array([0, -1, 0])
    w4 = np.array([0, -1, 0])
    w5 = np.array([0, 0, 1])
    w6 = np.array([0, -1, 0])
    q1 = np.array([0, 0, 0.1519])
    q2 = np.array([0, -.1116, .1519])
    q3 = np.array([0, -.1116, .3955])
    q4 = np.array([0, -.1116, .6088])
    q5 = np.array([0, -.1123, .6930])
    q6 = np.array([0, -.1116, .6941])
    
    
    #calculate screw axis given omega and q
        
    S1 = axis(w1,q1)
    S2 = axis(w2,q2)
    S3 = axis(w3,q3)
    S4 = axis(w4,q4)
    S5 = axis(w5,q5)
    S6 = axis(w6,q6)
    
    #arrange screw axis in skew symmetric array
    S = np.zeros((6,6))
    S[:,0] = S1
    S[:,1] = S2
    S[:,2] = S3
    S[:,3] = S4
    S[:,4] = S5
    S[:,5] = S6
    
    return M, S



class robot:
    def __init__(self, clientID, arm, gripper, joints):
        self.clientID = clientID
        self.arm = arm
        self.gripper = gripper
        self.joints = joints
        self.theta = [0, 0, 0, 0, 0, 0]
       
        
##   Updates current joint angles. If hasn't been called, uses streaming. Otherwise, buffer mode
#    def curPos(self):
#        pose = []
#        if self.poseInit == False:
#            opm = sim.simx_opmode_streaming
#        if self.poseInit == True:
#            opm = sim.simx_opmode_buffer
#        for joint in self.joints:
#            pose.append(sim.simxGetJointPosition(self.clientID,joint,opm)[1])
#            self.poseInit = True
#        self.theta = pose
#        return
    def curPos(self,opm):
        pos = []
        for joint in self.joints:
            error, theta = sim.simxGetJointPosition(self.clientID,joint,opm)
            pos.append(theta)
        self.theta = pos
        return
    
#   Direct movement of allangles
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


#   Forward kinematics of UR3 arm
    def fork(self,thetas):
        M, S = get_MS()
    
        #calculates the homogenous transformation matrix from robot base frame to end-effector frame
        E1 = expm(skew(S[:,0])*thetas[0])
        E2 = expm(skew(S[:,1])*thetas[1])
        E3 = expm(skew(S[:,2])*thetas[2])
        E4 = expm(skew(S[:,3])*thetas[3])
        E5 = expm(skew(S[:,4])*thetas[4])
        E6 = expm(skew(S[:,5])*thetas[5])
        T12 = np.matmul(E1,E2)
        T23 = np.matmul(T12,E3)
        T34 = np.matmul(T23,E4)
        T45 = np.matmul(T34,E5)
        T56 = np.matmul(T45,E6)
        T = np.matmul(T56,M)
        
        d = T[:,3][0:3]
        return d
            
    def invk(self, coor):
        #the geometry in general is different than in lab5, must be careful
        L01 = 0.1519;
        L02 = 0.1115;
        L03 = 0.3955 - 0.1519;
        L04 = 0.1115 - 0.0291;
        L05 = 0.6088 - 0.3955;
        L06 = 0.1122 - 0.0291;
        L07 = 0.6941 - 0.6088;
        L08 = 0.1940 - 0.1122;  
        L09 = 0.2577 - 0.1940;
        thetas = [0.0, 0.0, 0.0, 0.0, -np.pi/2, 0.0]
        xw = coor[0]
        yw = coor[1]
        zw = coor[2]
        # theta1
        R = math.sqrt(yw**2+ xw**2)
        #frame is rotated pi/2 radians so quadrants are incorrect using atan2
        #taking the opposite values works for all cases
        alpha1 = math.atan2(-yw,-xw)
        alpha2 = math.asin(L02/R)
        thetas[0] = alpha1-alpha2
        
        #theta6 does not matter with baxter vacuum cup
        thetas[5] = 0


        # Projected end points
        z3_end = zw+L09+L08
        y3_end = yw+L07*math.sin(thetas[0])+(L06+(L02-L06))*math.cos(thetas[0])
        x3_end = xw+L07*math.cos(thetas[0])-(L06+(L02-L06))*math.sin(thetas[0])

        #theta2, theta3, theta4
        d = z3_end-L01
        R = math.sqrt(x3_end**2 + y3_end**2 + d**2)
        alpha = math.asin(d/R)
        try:
            C = math.acos((L05**2-L03**2-R**2)/(-2*L03*R))
            B = math.acos((R**2-L05**2-L03**2)/(-2*L03*L05))
        except:
            print('Could not converge to solution!')
            return
        thetas[1] = np.pi/2-(alpha+C) #theta2 measured from vert, not horiz!
        thetas[2] = np.pi-B
        thetas[3] = -(np.pi-(np.pi/2-thetas[1])-B)
        anglelist = []
        #to keep angles within -2*pi and 2*pi
        for angle in thetas:
            while angle >= 2*np.pi:
                angle = angle - 2*np.pi
            while angle <= -2*np.pi:
                angle = angle + 2*np.pi
            anglelist.append(angle)
        return anglelist
