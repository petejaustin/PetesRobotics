# Script written to track torque values as the Panda arm is sent down z-axis. As
# well as this, it also sends force across the y-axis to the Panda to simulate
# it being pushed around rather than just being commanded where to go.
#
# A plot is created of all the torque vals for each iteration, so we can track
# how the 'unforeseen' force affects the torques.
#
# This is a control law designed to control force given to it. All params are
# commented up. This builds directly on ik_force_apply.py

#! /usr/bin/env python

import sys
from sys import argv
import copy
from pathlib import Path
import numpy as np
from numpy.linalg import norm, solve
import rospy
from panda_robot import PandaArm
import pinocchio
from math import pi, cos, sin
import math
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
from gazebo_msgs.srv import ApplyBodyWrench
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

tq1 = []
tq2 = []
tq3 = []
tq4 = []
tq5 = []
tq6 = []
tq7 = []
iVec = []

forcexlist = []
forceylist = []
forcezlist = []

Xcx = []
Xcy = []
Xcz = []
Xvx = []
Xvy = []
Xvz = []

rospy.init_node("panda_sim")
p = PandaArm()
forceVals = None

def getForce(data):
    global forceVals
    forceVals = data.wrench.force

rospy.Subscriber("/gazebo/robot/wrist/ft", WrenchStamped, getForce)

def applyForce(forceX, forceY, forceZ):

    rospy.wait_for_service('/gazebo/apply_body_wrench')
    force = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    panda_wrench = WrenchStamped()
    panda_wrench.wrench.force.x = forceX
    panda_wrench.wrench.force.y = forceY
    panda_wrench.wrench.force.z = forceZ

    force(body_name = "panda_link7", wrench = panda_wrench.wrench, duration = rospy.Duration(-1))

# reset force on the arm
applyForce(0,0,0)

def ik_solver(ori, pos):

    # MAKE SURE TO CHANGE WORKSPACE IN urdf_filename TO WHATEVER YOUR WS IS!!!!!
    home = str(Path.home())
    urdf_filename = home + '/catkin_ws/src/franka_panda_description/robots/panda_arm.urdf' if len(argv)<2 else argv[1]
    model = pinocchio.buildModelFromUrdf(urdf_filename)
    data = model.createData()

    JOINT_ID = 7
    oMdes = pinocchio.SE3(ori, pos)

    initDict = p.joint_angles()
    initVals = initDict.values()
    init = list(initVals)

    q = pinocchio.neutral(model)
    q[0] = 2.7
    q[1] = -1.
    q[2] = 0.0198163
    q[3] = -2.3421145
    q[4] = 0.02988193
    q[5] = 1.5413066
    q[6] = 0.7534217

    eps = 1e-4
    IT_MAX = 100000
    DT = 1e-1
    damp = 1e-12

    pinocchio.forwardKinematics(model,data,q)

    i=0
    while True:
        pinocchio.forwardKinematics(model,data,q)
        dMi = oMdes.actInv(data.oMi[JOINT_ID])
        err = pinocchio.log(dMi).vector
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break
        J = pinocchio.computeJointJacobian(model,data,q,JOINT_ID)

        v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pinocchio.integrate(model,q,v*DT)

        # joint 1
        if (q[0] <= -2.7):
            q[0] = 0.
        elif (q[0] >= 2.7):
            q[0] = 0.

        # joint 2
        if (q[1] <= -1.6):
            q[1] = 0.
        elif (q[1] >= 1.6):
            q[1] = 0.

        # joint 3
        if (q[2] <= -2.7):
            q[2] = 0.
        elif (q[2] >= 2.7):
            q[2] = 0.

        #joint 4
        if (q[3] <= -2.8):
            q[3] = -1.
        elif (q[3] >= -0.1):
            q[3] = -1.

        #joint 5
        if (q[4] <= -2.7):
            q[4] = 0.
        elif (q[4] >= 2.7):
            q[4] = 0.

        #joint 6
        if (q[5] <= 0.):
            q[5] = 1.
        elif (q[5] >= 3.5):
            q[5] = 1.

        #joint 7
        if (q[6] <= -2.7):
            q[6] = 0.
        elif (q[6] >= 2.7):
            q[6] = 0.
        i += 1
	
    return q, success

def mov_neutral():

    angles = p.joint_angles()
    target = np.array([0.31057491, 0.00564737, 0.68653714])

    destJointVals, success = ik_solver(np.array([[1,0,0],
                                                 [0,-1,0],
                                                 [0,0,-1]]), target)

    angles['panda_joint1'] = destJointVals[0]
    angles['panda_joint2'] = destJointVals[1]
    angles['panda_joint3'] = destJointVals[2]
    angles['panda_joint4'] = destJointVals[3]
    angles['panda_joint5'] = destJointVals[4]
    angles['panda_joint6'] = destJointVals[5]
    angles['panda_joint7'] = destJointVals[6]

    listAngle = list(angles.values())

    p.set_joint_positions_velocities(listAngle, [0.1]*7)
    rospy.sleep(2)

def movJoint(Kd):

    mov_neutral()

    angles = p.joint_angles()

    #Curr = current co-ordinates. Instantiated as a generic start point
    Curr = np.array([0.5, 0., 0.69])

    #Fr = desired forces. Constant
    Fr = np.array([0., 0., 0.])

    #iterator to be used in control loop for artificial force; no other purpose.
    #commented out for now as while loop didnt work
    i = 0

    #Kd = Stiffness hyperparam. Play around with this. 2.5m best thus far

    destJointVals, success = ik_solver(np.array([[1,0,0],
                                                 [0,-1,0],
                                                 [0,0,-1]]), Curr)

    angles['panda_joint1'] = destJointVals[0]
    angles['panda_joint2'] = destJointVals[1]
    angles['panda_joint3'] = destJointVals[2]
    angles['panda_joint4'] = destJointVals[3]
    angles['panda_joint5'] = destJointVals[4]
    angles['panda_joint6'] = destJointVals[5]
    angles['panda_joint7'] = destJointVals[6]

    listAngle = list(angles.values())

    p.set_joint_positions_velocities(listAngle, [0.1]*7)
    rospy.sleep(2)

    while(True):

        global tq1, tq2, tq3, tq4, tq5, tq6, tq7, iVec, forceVals
        global forcexlist, forceylist, forcezlist, Xcx, Xcy, Xcz, Xvx, Xvy, Xvz

        #Relatively 'safe' force can be considered as around 30N, the arm can
        #handle up to around 150N before erroneous patterns emerge
        if (i == 100):
            applyForce(0,30,0)

        #Although unnamed, this is delta
        Curr[2] -= 0.001

        #Xv = zCurr + delta
        Xv = Curr

        #Fsense = force detected at current iteration. Used to calc Xc
        Fsense = np.array([0., 0., 0.])
        Fsense[0] = forceVals.x
        Fsense[1] = forceVals.y
        Fsense[2] = forceVals.z

        #Identity matrix for Kd. For now only using the 2nd vec of the matrix
        KdMatrix = np.array([[1/Kd, 0, 0],
                             [0, 1/Kd, 0],
                             [0, 0, 1/Kd]])

        #Xc = vector applied with impedance control. This is fed into ik_solver
        #
        #Note: slice all with 0:, saving needless for loops
        
        # Fdiff = difference between reference force and sensed force. Used to
        # calc dot product of difference and KdMatrix
        Fdiff = Fsense - Fr
        
        Xc = np.array([0., 0., 0.])
        Xc[0:] = np.dot(Fdiff, KdMatrix[0:]) + Xv[0:]
        
        #Original solution: Xc = (Fsense[0:] - Fr[0:]) * KdMatrix[1][0:] + Xv

        destJointVals, success = ik_solver(np.array([[1,0,0],
	                                                 [0,-1,0],
												     [0,0,-1]]), Xc)

        if (Fsense[2]<-60. and i>20):

            j = 0

            rospy.sleep(1)

            while (j<3):

                Curr[2] += 0.001

                #Xv = zCurr + delta
                Xv = Curr

                #Fsense = force detected at current iteration. Used to calc Xc
                Fsense = np.array([0., 0., 0.])
                Fsense[0] = forceVals.x
                Fsense[1] = forceVals.y
                Fsense[2] = forceVals.z

                #Identity matrix for Kd. For now only using the 2nd vec of the matrix
                KdMatrix = np.array([[1/Kd, 0, 0],
                                     [0, 1/Kd, 0],
                                     [0, 0, 1/Kd]])
                
                Fdiff = Fsense - Fr
        
                Xc = np.array([0., 0., 0.])
                Xc[0:] = np.dot(Fdiff, KdMatrix[0:]) + Xv[0:]

                destJointVals, success = ik_solver(np.array([[1,0,0],
	                                                         [0,-1,0],
												             [0,0,-1]]), Xc)

                angles['panda_joint1'] = destJointVals[0]
                angles['panda_joint2'] = destJointVals[1]
                angles['panda_joint3'] = destJointVals[2]
                angles['panda_joint4'] = destJointVals[3]
                angles['panda_joint5'] = destJointVals[4]
                angles['panda_joint6'] = destJointVals[5]
                angles['panda_joint7'] = destJointVals[6]
                listAngle = list(angles.values())

                p.set_joint_positions_velocities(listAngle, [0.1]*7)
                rospy.sleep(0.01)
                j += 1

                tqDict = p.joint_efforts()
                tq1.append(tqDict['panda_joint1'])
                tq2.append(tqDict['panda_joint2'])
                tq3.append(tqDict['panda_joint3'])
                tq4.append(tqDict['panda_joint4'])
                tq5.append(tqDict['panda_joint5'])
                tq6.append(tqDict['panda_joint6'])
                tq7.append(tqDict['panda_joint7'])
                iVec.append(i+j)
                forcexlist.append(forceVals.x)
                forceylist.append(forceVals.y)
                forcezlist.append(forceVals.z)
                Xcx.append(Xc[0])
                Xcy.append(Xc[1])
                Xcz.append(Xc[2])
                Xvx.append(Xv[0])
                Xvy.append(Xv[1])
                Xvz.append(Xv[2])

            return Xc, i+j

        # Large block of appending to global vectors for graph visualisation
        tqDict = p.joint_efforts()
        tq1.append(tqDict['panda_joint1'])
        tq2.append(tqDict['panda_joint2'])
        tq3.append(tqDict['panda_joint3'])
        tq4.append(tqDict['panda_joint4'])
        tq5.append(tqDict['panda_joint5'])
        tq6.append(tqDict['panda_joint6'])
        tq7.append(tqDict['panda_joint7'])
        iVec.append(i+1)
        forcexlist.append(forceVals.x)
        forceylist.append(forceVals.y)
        forcezlist.append(forceVals.z)
        Xcx.append(Xc[0])
        Xcy.append(Xc[1])
        Xcz.append(Xc[2])
        Xvx.append(Xv[0])
        Xvy.append(Xv[1])
        Xvz.append(Xv[2])


        if (success == True):
            angles['panda_joint1'] = destJointVals[0]
            angles['panda_joint2'] = destJointVals[1]
            angles['panda_joint3'] = destJointVals[2]
            angles['panda_joint4'] = destJointVals[3]
            angles['panda_joint5'] = destJointVals[4]
            angles['panda_joint6'] = destJointVals[5]
            angles['panda_joint7'] = destJointVals[6]
            listAngle = list(angles.values())

            p.set_joint_position_speed(speed=0.8)

            p.set_joint_positions(angles)
            rospy.sleep(0.01)
        i+=1

# SYNTAX: movJoint(Kd stiffness)
xc_vals, newTimestep = movJoint(2000000)

def arch_spiral(old_Xc, Kd, timestep):

    global tq1, tq2, tq3, tq4, tq5, tq6, tq7, iVec, forceVals
    global forcexlist, forceylist, forcezlist, Xcx, Xcy, Xcz, Xvx, Xvy, Xvz

    Curr = old_Xc
    angles = p.joint_angles()

    for i in range(500):

        theta = i/30 * pi
        dx = (0.1 + 0.25 * theta) * cos(theta)
        dy = (0.1 + 0.25 * theta) * sin(theta)
        dx /= 2000
        dy /= 2000
        Curr[0] += dx
        Curr[1] += dy

        #Xv = zCurr + delta
        Xv = Curr

        Fr = np.array([0., 0., -2.])

        #Fsense = force detected at current iteration. Used to calc Xc
        Fsense = np.array([0., 0., 0.])
        Fsense[0] = forceVals.x
        Fsense[1] = forceVals.y
        Fsense[2] = forceVals.z

        #Identity matrix for Kd. For now only using the 2nd vec of the matrix
        KdMatrix = np.array([[1/Kd, 0, 0],
                            [0, 1/Kd, 0],
                            [0, 0, 1/Kd]])
                
        Fdiff = Fsense - Fr
        
        Xc = np.array([0., 0., 0.])
        Xc[0:] = np.dot(Fdiff, KdMatrix[0:]) + Xv[0:]

        destJointVals, success = ik_solver(np.array([[1,0,0],
	                                                 [0,-1,0],
												     [0,0,-1]]), Xc)

        tqDict = p.joint_efforts()
        tq1.append(tqDict['panda_joint1'])
        tq2.append(tqDict['panda_joint2'])
        tq3.append(tqDict['panda_joint3'])
        tq4.append(tqDict['panda_joint4'])
        tq5.append(tqDict['panda_joint5'])
        tq6.append(tqDict['panda_joint6'])
        tq7.append(tqDict['panda_joint7'])
        iVec.append(timestep+(i+1))
        forcexlist.append(forceVals.x)
        forceylist.append(forceVals.y)
        forcezlist.append(forceVals.z)
        Xcx.append(Xc[0])
        Xcy.append(Xc[1])
        Xcz.append(Xc[2])
        Xvx.append(Xv[0])
        Xvy.append(Xv[1])
        Xvz.append(Xv[2])

        angles['panda_joint1'] = destJointVals[0]
        angles['panda_joint2'] = destJointVals[1]
        angles['panda_joint3'] = destJointVals[2]
        angles['panda_joint4'] = destJointVals[3]
        angles['panda_joint5'] = destJointVals[4]
        angles['panda_joint6'] = destJointVals[5]
        angles['panda_joint7'] = destJointVals[6]
        listAngle = list(angles.values())

        p.set_joint_positions_velocities(listAngle, [0.1]*7)
        rospy.sleep(0.01)
        i += 1

arch_spiral(xc_vals, 2000000, newTimestep)

fig, ax = plt.subplots()
ax.plot(iVec, tq1, label='joint 1')
ax.plot(iVec, tq2, label='joint 2')
ax.plot(iVec, tq3, label='joint 3')
ax.plot(iVec, tq4, label='joint 4')
ax.plot(iVec, tq5, label='joint 5')
ax.plot(iVec, tq6, label='joint 6')
ax.plot(iVec, tq7, label='joint 7')
ax.legend()

plt.ylim([-85, 70])
fig.show()

input("Press Enter to see forces graph")

fig2, ax2 = plt.subplots()
ax2.plot(iVec, forcexlist, label='Force X')
ax2.plot(iVec, forceylist, label='Force Y')
ax2.plot(iVec, forcezlist, label='Force Z')
ax2.legend()
fig2.show()

input("Press Enter to see Xv and Xc Trajectories")

fig3 = plt.figure()
ax3 = plt.axes(projection='3d')
ax3.plot3D(Xvx, Xvy, Xvz, label='Xv')
ax3.plot3D(Xcx, Xcy,Xcz, ':', label='Xc')
ax3.set_title('Xv and Xc Trajectories')
ax3.set_xlabel('X axis')
ax3.set_ylabel('Y axis')
ax3.set_zlabel('Z axis')
ax3.legend()
plt.show()

input("Press Enter to move arm to neutral and end program")

# Dont forget to reset force with applyForce(0,0,0)!
applyForce(0,0,0)
mov_neutral()