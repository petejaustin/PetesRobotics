# Grabs the magnitude a.k.a l2 norm of the xyz force values, only for 1
# instance. Can easily apply to while rospy is not shutdown

import rospy
from panda_robot import PandaArm
from geometry_msgs.msg import WrenchStamped
import numpy as np

forceVals = None

rospy.init_node("panda_sim")

def getForce(data):
    global forceVals
    forceVals = data.wrench.force

rospy.Subscriber("/gazebo/robot/wrist/ft", WrenchStamped, getForce)

p = PandaArm()

forceX = forceVals.x 
forceY = forceVals.y 
forceZ = forceVals.z

forceVec = np.array([forceX, forceY, forceZ])
forceNorm = np.linalg.norm(forceVec)

print(forceX)
print(forceY)
print(forceZ)
print("MAGNITUDE:\n", forceNorm)