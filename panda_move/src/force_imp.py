import rospy
from panda_robot import PandaArm
from geometry_msgs.msg import WrenchStamped
from gazebo_msgs.srv import ApplyBodyWrench

rospy.init_node("panda_sim")
p = PandaArm()

panda_wrench = WrenchStamped()
forceX = panda_wrench.wrench.force.x
forceY = panda_wrench.wrench.force.y
forceZ = panda_wrench.wrench.force.z

def applyForce(forceInputX, forceInputY, forceInputZ):

    rospy.wait_for_service('/gazebo/apply_body_wrench')
    force = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    global panda_wrench
    panda_wrench.wrench.force.x = forceInputX
    panda_wrench.wrench.force.y = forceInputY
    panda_wrench.wrench.force.z = forceInputZ

    # Duration = -1 means this force will hold until force is changed somehow.
    # If a duration is given, it will return to original force after that
    # duration is over.
    force(body_name = "panda_link7", wrench = panda_wrench.wrench, duration = rospy.Duration(-1))
    rospy.sleep(1.)
    
applyForce(100, 200, 100)

if (forceX != panda_wrench.wrench.force.x):
    applyForce(forceX, forceY, forceZ)