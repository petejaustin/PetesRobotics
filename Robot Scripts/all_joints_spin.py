# Comment out all joints you dont want running at the same time.

import rospy
from panda_robot import PandaArm
rospy.init_node("panda_sim")
p = PandaArm()

def spinJoint():

	p.move_to_neutral()
	initial = p.joint_angles()
	
	'''
	tmp = initial
	tmp['panda_joint1'] = 2.6
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()
	tmp['panda_joint1'] = -2.6
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()

	tmp = initial
	tmp['panda_joint2'] = 1.5
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()
	tmp['panda_joint2'] = -1.5
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()
	
	tmp = initial
	tmp['panda_joint3'] = 2.6
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()
	tmp['panda_joint3'] = -2.6
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()

	tmp = initial
	tmp['panda_joint4'] = -2.8
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()
	tmp['panda_joint4'] = -0.2
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()

	tmp = initial
	tmp['panda_joint5'] = 2.6
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()
	tmp['panda_joint5'] = -2.6
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()

	tmp = initial
	tmp['panda_joint6'] = 3.6
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()
	tmp['panda_joint6'] = 0.
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()
	'''

	tmp = initial
	tmp['panda_joint7'] = 2.6
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()
	tmp['panda_joint7'] = -2.6
	p.move_to_joint_positions(tmp)
	p.move_to_neutral()
	
# Execute the spinJoint function
spinJoint()
