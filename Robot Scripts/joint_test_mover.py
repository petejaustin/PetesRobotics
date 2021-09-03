# This Python script showcases how we can spin around the first joint in the
# Panda arm, as well as going through the fundamentals of how to write basic
# scripts for the Panda arm

# Initial set up of ROS node and importing packages
import rospy
from panda_robot import PandaArm
rospy.init_node("panda_sim")
p = PandaArm()

# Make function to carry out the spin.
def spinJoint():

	# Start arm at neutral
	p.move_to_neutral()

	# Gather joint positions of initial pose values into an 'initial' dictionary
	initial = p.joint_angles()

	# Use a temp dictionary to take in mutated values which will be fed into 
	# move_to functions. Copying this dictionary saves effort and clutter
	tmp = initial

	# Change the radians value associated with joint 1 to LIMIT via the temp dict
	tmp['panda_joint1'] = 0.1870151
	tmp['panda_joint2'] = -0.7980226
	tmp['panda_joint3'] = -0.1219961
	tmp['panda_joint4'] = -2.376634
	tmp['panda_joint5'] = -0.0872458
	tmp['panda_joint6'] = 1.582290832
	tmp['panda_joint7'] = 0.10273326

	# Move joint 1 to 2.7 radians using the temp dict as input
	p.move_to_joint_positions(tmp)

	print(p.joint_angles())

	print(p.endpoint_pose())
	
# Execute the spinJoint function
spinJoint()
