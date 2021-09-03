# Experimented with a spiral smoothing algorithm, that does not work...
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

def ik_solver(ori, pos):

    # MAKE SURE TO CHANGE WORKSPACE IN urdf_filename TO WHATEVER YOUR WS IS!!!!!
    home = str(Path.home())
    urdf_filename = home + '/catkin_ws/src/franka_panda_description/robots/panda_arm.urdf' if len(argv)<2 else argv[1]
    model = pinocchio.buildModelFromUrdf(urdf_filename)
    data = model.createData()

    JOINT_ID = 7
    oMdes = pinocchio.SE3(ori, pos)

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
            q[0] = -2.7
        elif (q[0] >= 2.7):
            q[0] = 2.7

        # joint 2
        if (q[1] <= -1.6):
            q[1] = -1.6
        elif (q[1] >= 1.6):
            q[1] = 1.6

        # joint 3
        if (q[2] <= -2.7):
            q[2] = -2.7
        elif (q[2] >= 2.7):
            q[2] = 2.7

        #joint 4
        if (q[3] <= -2.8):
            q[3] = -2.8
        elif (q[3] >= -0.1):
            q[3] = -0.1

        #joint 5
        if (q[4] <= -2.7):
            q[4] = -2.7
        elif (q[4] >= 2.7):
            q[4] = 2.7

        #joint 6
        if (q[5] <= 0.):
            q[5] = 0.
        elif (q[5] >= 3.5):
            q[5] = 3.5

        #joint 7
        if (q[6] <= -2.7):
            q[6] = -2.7
        elif (q[6] >= 2.7):
            q[6] = 2.7
        i += 1
	
    return q, success

def movJoint():

    jointMatrix = np.zeros(42000)
    jointMatrix.shape = (6000, 7)

	# move out to further on x and lesser down in y before beginning
    destCart = np.array([0.6, 0.0, 0.5])

    # copy of destcart. Must be hard coded and not equivalenced
    originxyz = np.array([0.6, 0.0, 0.5])

    destJointVals, success = ik_solver(np.array([[1,0,0],
                                                 [0,-1,0],
                                                 [0,0,-1]]), destCart)

    # Archimedean spiral begins here
    for i in range(6000):

        theta = i/30 * pi
        dx = (0.1 + 0.25 * theta) * cos(theta)
        dy = (0.1 + 0.25 * theta) * sin(theta)
        dx /= 2000
        dy /= 2000
        destCart[0] += dx
        destCart[1] += dy

        destJointVals, success = ik_solver(np.array([[1,0,0],
	                                                 [0,-1,0],
												     [0,0,-1]]), destCart)

        if (success == True):
            
            # Euclidean distance checker to check for a 0.1 radius
            tempx = (abs(destCart[0]) - abs(originxyz[0])) ** 2
            tempy = (abs(destCart[1]) - abs(originxyz[1])) ** 2
            tempx = math.sqrt(tempx + tempy)
            
            # debug checking euclid distance works
            #print("tempx: ", tempx)

            for j in range(7):
                jointMatrix[i][j] = destJointVals[j]
            
            # If difference from origin values is larger than 0.1, end
            if (tempx >= 0.1):
                print("0.1m radius exceeded. Ending spiral.")
                return jointMatrix[0:i+1]
                break

        else:
            print("Can no longer safely continue spiral. Ending program.")
            break
    
    return jointMatrix[0:6000]

jointMat = movJoint()
print(jointMat)

def smoothMat(x, weight_data=0.5, weight_smooth=0.1, tolerance=0.000001):

    change = tolerance
    result = x
    while change >= tolerance:

        for i in range(1, 801):

            for j in range(len(x[0])):

                x_i = x[i][j]
                y_i, y_prev, y_next = result[i][j], result[i-1][j], result[i+1][j]

                y_i_saved = y_i
                y_i += weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2*y_i))
                result[i][j] = y_i

                change += abs(y_i - y_i_saved)

    return result

jointMat = smoothMat(jointMat)
print(jointMat) 