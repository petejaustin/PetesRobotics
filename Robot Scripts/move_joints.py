# move joints, display joints and end effector

import rospy
from panda_robot import PandaArm
rospy.init_node("panda_sim")
p = PandaArm()

p.move_to_neutral()

tmp = p.joint_angles()

#0.35877
tmp['panda_joint1'] = 0.35877
tmp['panda_joint2'] = 1.21621
tmp['panda_joint3'] = -0.000029126
tmp['panda_joint4'] = -0.467001
tmp['panda_joint5'] = 0.00002754
tmp['panda_joint6'] = 1.693766
tmp['panda_joint7'] = -1.99742

p.move_to_joint_positions(tmp)

print(p.joint_angles())

print(p.endpoint_pose())