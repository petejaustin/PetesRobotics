import rospy
from panda_robot import PandaArm
from geometry_msgs.msg import WrenchStamped
from gazebo_msgs.srv import ApplyBodyWrench

rospy.init_node("panda_sim")
p = PandaArm()
p.move_to_neutral()

def applyForce(forceX, forceY, forceZ):

    rospy.wait_for_service('/gazebo/apply_body_wrench')
    force = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    panda_wrench = WrenchStamped()
    panda_wrench.wrench.force.x = forceX
    panda_wrench.wrench.force.y = forceY
    panda_wrench.wrench.force.z = forceZ

    force(body_name = "panda_link7", wrench = panda_wrench.wrench, duration = rospy.Duration(-1))
    
applyForce(0, 5000, 0)