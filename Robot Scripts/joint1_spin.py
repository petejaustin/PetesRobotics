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
	tmp['panda_joint1'] = 2.6

	# Move joint 1 to 2.7 radians using the temp dict as input
	p.move_to_joint_positions(tmp)

	# If we try to move to the opposite side straight away, the Terminal will 
	# throw that it has failed to meet the trajectory. To counter this, reset to 
	# initial at first
	p.move_to_neutral()

	# Change rad value to the opposite limit of -2.7, and call to move joints
	tmp['panda_joint1'] = -2.6
	p.move_to_joint_positions(tmp)

	# End by returning to initial
	p.move_to_neutral()
	
# Execute the spinJoint function
spinJoint()
