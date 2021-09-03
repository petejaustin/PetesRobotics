# Subscriber practice for the '/gazebo/robot/wrist/ft' topic.
# Script grabs the force x,y,z values from the topic listed above.

import rospy
from panda_robot import PandaArm
from geometry_msgs.msg import WrenchStamped

forceVals = None

rospy.init_node("panda_sim")

def getForce(data):
    global forceVals
    forceVals = data.wrench.force

rospy.Subscriber("/gazebo/robot/wrist/ft", WrenchStamped, getForce)

p = PandaArm()

while not rospy.is_shutdown():

    forceX = forceVals.x 
    forceY = forceVals.y 
    forceZ = forceVals.z 

    print(forceX)
    print(forceY)
    print(forceZ)