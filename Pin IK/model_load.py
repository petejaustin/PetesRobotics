# This script showcases how to load a URDF using Pinocchio

import pinocchio
from sys import argv
from os.path import dirname, join, abspath

# Define workspace path here. CHANGE THE WORKSPACE AS NECESSARY BEFORE EXECUTING
ws_dir = join(dirname(dirname(str(abspath(__file__)))), "catkin_ws")

# Add the rest of the directory path to the panda_arm.urdf file.
urdf_filename = ws_dir + '/src/franka_panda_description/robots/panda_arm.urdf' if len(argv)<2 else argv[1]

# Load the urdf model
model    = pinocchio.buildModelFromUrdf(urdf_filename)
print('model name: ' + model.name)

# Create data required by the algorithms
data     = model.createData()

# Sample a random configuration
q        = pinocchio.randomConfiguration(model)
print('q: %s' % q.T)

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model,data,q)

# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
          .format( name, *oMi.translation.T.flat )))