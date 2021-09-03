#! /usr/bin/env python3

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

rospy.init_node("panda_sim")
p = PandaArm()

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

    p.move_to_neutral(timeout=30., speed=0.4)
    
    p.set_joint_position_speed(speed=0.8)
    angles = p.joint_angles()
    jointMatrix = np.array([])

    # Set joint torques to half. We grab these with joint_efforts(), as these
    # are effort torques
    tq = p.joint_efforts()
    tq['panda_joint1']/=2
    tq['panda_joint2']/=2
    tq['panda_joint3']/=2
    tq['panda_joint4']/=2
    tq['panda_joint5']/=2
    tq['panda_joint6']/=2
    tq['panda_joint7']/=2
    p.set_joint_torques(tq)

    # Set joint velocities to half speed
    #vel = p.joint_velocities()
    #vel['panda_joint1']/=2
    #vel['panda_joint2']/=2
    #vel['panda_joint3']/=2
    #vel['panda_joint4']/=2
    #vel['panda_joint5']/=2
    #vel['panda_joint6']/=2
    #vel['panda_joint7']/=2
    #listVel = list(vel.values())

	# move out to further on x and lesser down in y before beginning
    destCart = np.array([0.6, 0.0, 0.5])

    # copy of destcart. Must be hard coded and not equivalenced
    originxyz = np.array([0.6, 0.0, 0.5])

    destJointVals, success = ik_solver(np.array([[1,0,0],
                                                 [0,-1,0],
                                                 [0,0,-1]]), destCart)

    #angles['panda_joint1'] = destJointVals[0]
    #angles['panda_joint2'] = destJointVals[1]
    #angles['panda_joint3'] = destJointVals[2]
    #angles['panda_joint4'] = destJointVals[3]
    #angles['panda_joint5'] = destJointVals[4]
    #angles['panda_joint6'] = destJointVals[5]
    #angles['panda_joint7'] = destJointVals[6]
    #listAngle = list(angles.values())

    # set_joint_positions_velocities() is more efficient than 
    # move_to_joint_positions(). Uses lists of floats as args. However it is
    # not working properly just yet so use move_to for initial positioning
    p.move_to_joint_positions(angles, timeout=30., speed=0.4)
    rospy.sleep(0.5)

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
            angles['panda_joint1'] = destJointVals[0]
            angles['panda_joint2'] = destJointVals[1]
            angles['panda_joint3'] = destJointVals[2]
            angles['panda_joint4'] = destJointVals[3]
            angles['panda_joint5'] = destJointVals[4]
            angles['panda_joint6'] = destJointVals[5]
            angles['panda_joint7'] = destJointVals[6]
            listAngle = list(angles.values())

            p.set_joint_positions_velocities(listAngle, [0.0]*7)
            rospy.sleep(0.01)

            # Euclidean distance checker to check for a 0.1 radius
            tempx = (abs(destCart[0]) - abs(originxyz[0])) ** 2
            tempy = (abs(destCart[1]) - abs(originxyz[1])) ** 2
            tempx = math.sqrt(tempx + tempy)
            
            # debug checking euclid distance works
            #print("tempx: ", tempx)
            
            # If difference from origin values is larger than 0.1, end
            if (tempx >= 0.1):
                print("0.1m radius exceeded. Ending spiral.")
                break

        else:
            print("Can no longer safely continue spiral. Ending program.")
            break

movJoint()