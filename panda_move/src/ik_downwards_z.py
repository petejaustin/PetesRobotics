# complete modification of ik_downwards_ft.
#
# INTUITION:
# 1) Goes down by 1cm at a time, tracks z axis force.
# 2) If force goes negative, start travelling backwards on x axis for 20 cm.
# 3) At each iteration of the x axis movement, still check the z axis force,
#    and correct if needed by 1cm

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

# Global variable tracking force values
forceVals = None

rospy.init_node("panda_sim")
p = PandaArm()

# Function to grab the force vector from the ft topic
def getForce(data):
    global forceVals
    forceVals = data.wrench.force

# Generate a subscriber and return values using getForce
rospy.Subscriber("/gazebo/robot/wrist/ft", WrenchStamped, getForce)

# Function to grab the force norm
def getForceZ():

    global forceVals
    forceZ = forceVals.z

    print("Force Z:\n", forceZ)
    return forceZ

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
    
    zCheck = getForceZ()

    for i in range(10000):

        destCart[2] -= 0.002

        destJointVals, success = ik_solver(np.array([[1,0,0],
	                                                 [0,-1,0],
												     [0,0,-1]]), destCart)

        zCheck = getForceZ()
        if (zCheck<-20. and i>20):

            destCart[2] += 0.001

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
            listAngle = list(angles.values())

            p.set_joint_positions_velocities(listAngle, [0.1]*7)
            rospy.sleep(0.1)

            return destCart

        if (success == True):
            angles['panda_joint1'] = destJointVals[0]
            angles['panda_joint2'] = destJointVals[1]
            angles['panda_joint3'] = destJointVals[2]
            angles['panda_joint4'] = destJointVals[3]
            angles['panda_joint5'] = destJointVals[4]
            angles['panda_joint6'] = destJointVals[5]
            angles['panda_joint7'] = destJointVals[6]
            listAngle = list(angles.values())

            p.set_joint_positions_velocities(listAngle, [0.1]*7)
            rospy.sleep(0.1)

def xMove(destCart):

    angles = p.joint_angles()

    for i in range(100):

        destCart[0] -= 0.002

        destJointVals, success = ik_solver(np.array([[1,0,0],
                                                     [0,-1,0],
                                                     [0,0,-1]]), destCart)

        zCheck = getForceZ()

        if (zCheck>0.):

            destCart[2] -= 0.001

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
            listAngle = list(angles.values())

            p.set_joint_positions_velocities(listAngle, [0.1]*7)
            rospy.sleep(0.1)

        if (success == True):
            angles['panda_joint1'] = destJointVals[0]
            angles['panda_joint2'] = destJointVals[1]
            angles['panda_joint3'] = destJointVals[2]
            angles['panda_joint4'] = destJointVals[3]
            angles['panda_joint5'] = destJointVals[4]
            angles['panda_joint6'] = destJointVals[5]
            angles['panda_joint7'] = destJointVals[6]
            listAngle = list(angles.values())

            p.set_joint_positions_velocities(listAngle, [0.1]*7)
            rospy.sleep(0.1)

coords = movJoint()
xMove(coords)