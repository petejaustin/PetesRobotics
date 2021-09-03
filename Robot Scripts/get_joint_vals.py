import rospy
from panda_robot import PandaArm
rospy.init_node("panda_sim")
p = PandaArm()

p.move_to_neutral()

print(p.joint_angles())

print(p.endpoint_pose())