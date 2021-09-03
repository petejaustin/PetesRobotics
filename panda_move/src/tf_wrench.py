# This script transforms forces read into the link7 FT as how they have really
# been applied through the world. Have this running in a Terminal at the same
# time as the Panda is running, as well as some script to manipulate it.

import rospy
from panda_robot import PandaArm
from geometry_msgs.msg import Wrench, WrenchStamped
import tf2_ros
import tf2_geometry_msgs

forceVals = None
tqVals = None

rospy.init_node("foo")

tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)
rospy.sleep(2)

def t_wrench(input, curr_frame, desired_frame):

    wrench_stamped = tf2_geometry_msgs.WrenchStamped()
    wrench_stamped.header.frame_id = curr_frame
    wrench_stamped.wrench = input
    wrench_stamped.header.stamp = rospy.Time.now()

    output_wrench_stamped = tf_buffer.transform(wrench_stamped, desired_frame, rospy.Duration(1))
    return output_wrench_stamped.wrench

def getFT(data):
    global forceVals
    global tqVals
    forceVals = data.wrench.force
    tqVals = data.wrench.torque

rospy.Subscriber("/gazebo/robot/wrist/ft", WrenchStamped, getFT)

p = PandaArm()

while not rospy.is_shutdown():

    my_wrench = Wrench()
    my_wrench.force.x = forceVals.x
    my_wrench.force.y = forceVals.y
    my_wrench.force.z = forceVals.z
    my_wrench.torque.x = tqVals.x
    my_wrench.torque.y = tqVals.y
    my_wrench.torque.z = tqVals.z
    transformed_wrench = t_wrench(my_wrench, "panda_link7", "world")
    print("WRENCH FT WITH LINK7 AS REFERENCE FRAME:\n", my_wrench)
    print("\nWRENCH FT WITH WORLD AS REFERENCE FRAME:\n", transformed_wrench)