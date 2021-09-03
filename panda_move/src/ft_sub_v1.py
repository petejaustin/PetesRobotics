# Subscriber practice for the '/gazebo/robot/wrist/ft' topic.
# Script grabs the force x,y,z values from the topic listed above.

import rospy
from panda_robot import PandaArm
from geometry_msgs.msg import WrenchStamped

forceVals = None
headerVals = None

rospy.init_node("panda_sim")

def getForce(data):
    global forceVals
    global headerVals
    forceVals = data.wrench.force
    headerVals = data.header

rospy.Subscriber("/gazebo/robot/wrist/ft", WrenchStamped, getForce)

p = PandaArm()

print(headerVals)

# rospy.sleep(5.) means it will only print once every 5sec
'''
while not rospy.is_shutdown():

    forceX = forceVals.x 
    forceY = forceVals.y 
    forceZ = forceVals.z 

    print(forceX)
    print(forceY)
    print(forceZ)
    rospy.sleep(5.)
'''