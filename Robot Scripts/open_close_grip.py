# Very basic script showing how to open the Panda gripper jaws to max open
# distance, as well as close them again

import rospy

# Import the GripperInterface for this
from franka_interface import GripperInterface

rospy.init_node("panda_sim")

# Define an object of the GripperInterface
gripCtrl = GripperInterface()

# open() = Open to max distance
# close() = close fully
gripCtrl.open()
gripCtrl.close()