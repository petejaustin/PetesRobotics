from __future__ import print_function
 
import pinocchio
from sys import argv
from os.path import dirname, join, abspath
import numpy as np
from numpy.linalg import norm, solve
 
import pinocchio

# Define workspace path here
ws_dir = join(dirname(dirname(str(abspath(__file__)))), "catkin_ws")

# Add the rest of the directory path to the panda_arm.urdf file.
urdf_filename = ws_dir + '/src/franka_panda_description/robots/panda_arm.urdf' if len(argv)<2 else argv[1]
 
model = pinocchio.buildModelFromUrdf(urdf_filename)
model.lowerPositionLimits = [-2.8973, -1.7628, -28973, -3.0718, -2.8973, -0.0175, -2.8973]
model.upperPositionLimits = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
data  = model.createData()

# discovered how to find frames, uncomment below to test
#print(model.frames[0])
 
# Panda arm has 7 joints, end effector is id = 7
JOINT_ID = 7

# This is the DESIRED OUTCOME. The argument format is the rotation matrix,
# followed by the position vector. For extra rotation info, check up on the 
# rotational matrices info online. 
oMdes = pinocchio.SE3(np.array([[1,0,0],
                                [0,-1,0],
                                [0,0,-1]]), np.array([0.3069, 0.0, 0.69]))

print(oMdes)
 
# Starting point for the algorithm. is an np.ndarray.
# Instantiate with neutral then fill q[0] to q[6]. These are panda coords in
# panda gazebo AFTER neutral is ran
q      = pinocchio.neutral(model)
q[0] = -0.01782388
q[1] = -0.76013098
q[2] = 0.0198163
q[3] = -2.3421145
q[4] = 0.02988193
q[5] = 1.5413066
q[6] = 0.7534217

# Note: cartesian value of this joint setup after neutral is [0.3069, 0.0, 0.69]

# save initial vals into copy np.array
init = q

#print(q.shape)

print('\njoint angles in rads: %s' % q.flatten().tolist())
# Desired position precision
eps    = 1e-4

# Max amount of iterations to try to see if position is reachable
IT_MAX = 100000

# Positive timestep defining convergence rate
DT     = 1e-1

# Damping factor for pseudoinversion
damp   = 1e-12

# Always apply forward kinematics first BEFORE displaying
pinocchio.forwardKinematics(model,data,q)

# print joint cartesian
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
        .format( name, *oMi.translation.T.flat )))
 
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

    # set if statement block here to check bounds.

    # joint 1
    if (q[0] <= -2.7):
        q[0] = init[0]
    elif (q[0] >= 2.7):
        q[0] = init[0]

    # joint 2
    if (q[1] <= -1.6):
        q[1] = init[1]
    elif (q[1] >= 1.6):
        q[1] = init[1]

    # joint 3
    if (q[2] <= -2.7):
        q[2] = init[2]
    elif (q[2] >= 2.7):
        q[2] = init[2]

    #joint 4
    if (q[3] <= -2.6):
        q[3] = init[3]
    elif (q[3] >= -0.1):
        q[3] = init[3]

    #joint 5
    if (q[4] <= -2.7):
        q[4] = init[4]
    elif (q[4] >= 2.7):
        q[4] = init[4]

    #joint 6
    if (q[5] <= 0.):
        q[5] = init[5]
    elif (q[5] >= 3.5):
        q[5] = init[5]

    #joint 7
    if (q[6] <= -2.7):
        q[6] = init[6]
    elif (q[6] >= 2.7):
        q[6] = init[6]

    if not i % 10:
        print('%d: error = %s' % (i, err.T))

        # print joint cartesian
        for name, oMi in zip(model.names, data.oMi):
            print(("{:<24} : {: .2f} {: .2f} {: .2f}"
                .format( name, *oMi.translation.T.flat )))
    i += 1
 
if success:
    print("Convergence achieved!")
else:
    print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")

print('\nresult: %s' % q.flatten().tolist())
print('\nfinal error: %s' % err.T)

# print joint cartesian
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
        .format( name, *oMi.translation.T.flat )))