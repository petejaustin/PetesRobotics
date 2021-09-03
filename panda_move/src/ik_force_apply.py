# Script written to track torque values as the Panda arm is sent down z-axis. As
# well as this, it also sends force across the y-axis to the Panda to simulate
# it being pushed around rather than just being commanded where to go.
#
# A plot is created of all the torque vals for each iteration, so we can track
# how the 'unforeseen' force affects the torques.

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

tq1 = []
tq2 = []
tq3 = []
tq4 = []
tq5 = []
tq6 = []
tq7 = []
iVec = []

rospy.init_node("panda_sim")
p = PandaArm()

def applyForce(forceX, forceY, forceZ):

    rospy.wait_for_service('/gazebo/apply_body_wrench')
    force = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    panda_wrench = WrenchStamped()
    panda_wrench.wrench.force.x = forceX
    panda_wrench.wrench.force.y = forceY
    panda_wrench.wrench.force.z = forceZ

    force(body_name = "panda_link7", wrench = panda_wrench.wrench, duration = rospy.Duration(1))

def ik_solver(ori, pos):

    # MAKE SURE TO CHANGE WORKSPACE IN urdf_filename TO WHATEVER YOUR WS IS!!!!!
    home = str(Path.home())
    urdf_filename = home + '/panda_ws/src/franka_panda_description/robots/panda_arm.urdf' if len(argv)<2 else argv[1]
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

def movJoint():

    p.move_to_neutral(timeout=30., speed = 0.4)

    angles = p.joint_angles()

    destCart = np.array([0.5, 0., 0.69])

    destJointVals, success = ik_solver(np.array([[1,0,0],
                                                 [0,-1,0],
                                                 [0,0,-1]]), destCart)

    angles['panda_joint1'] = destJointVals[0]
    angles['panda_joint2'] = destJointVals[1]
    angles['panda_joint3'] = destJointVals[2]
    angles['panda_joint4'] = destJointVals[3]
    angles['panda_joint5'] = destJointVals[4]
    angles['panda_joint6'] = destJointVals[5]
    angles['panda_joint7'] = destJointVals[6]

    p.move_to_joint_positions(angles)
    rospy.sleep(0.01)

    for i in range(300):

        global tq1, tq2, tq3, tq4, tq5, tq6, tq7, iVec

        if (i == 100):
            applyForce(0,100,0)
        elif(i == 200):
            applyForce(0,0,0)

        destCart[2] -= 0.001

        destJointVals, success = ik_solver(np.array([[1,0,0],
	                                                 [0,-1,0],
												     [0,0,-1]]), destCart)

        tqDict = p.joint_efforts()
        tq1.append(tqDict['panda_joint1'])
        tq2.append(tqDict['panda_joint2'])
        tq3.append(tqDict['panda_joint3'])
        tq4.append(tqDict['panda_joint4'])
        tq5.append(tqDict['panda_joint5'])
        tq6.append(tqDict['panda_joint6'])
        tq7.append(tqDict['panda_joint7'])
        iVec.append(i+1)


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

movJoint()

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

input("Enter to end program")