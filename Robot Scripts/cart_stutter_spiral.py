# A version of the Archimedean spiral that does not invoke the moveit server
# within the source code. However it does need the moveit server to still run,
# meaning that implicitly it still definitely uses it. 
#
# This script will be saved as it is a good showcase of how to interact with the
# end effector dictionary, as well as rudimentary usage of sending commands to
# the Cartesian task space.

import sys
import copy
import rospy
import numpy as np

from math import pi, cos, sin

from panda_robot import PandaArm

class Practice(object):

    def __init__(self):

        rospy.init_node("panda_sim", anonymous=True)

        # Declaration of PandaArm
        p = PandaArm()

        # Return the Panda arm to a neutral pose before performing any kind of
        # movement function on it
        p.move_to_neutral()

        self.p = p

    # Without planning the Cartesian path first, movement functions will NOT
    # be executed
    def movement(self, scale=1):

        p = self.p

        # endpoint_pose() returns a dictionary. Must extract vector found at
        # the key 'position'
        tempDict = p.endpoint_pose()

        # coord = 3 element vector (numpy array) with 
        # x, y, z values in that order
        coord = tempDict['position']

        # Archimedean spiral algorithm
        for i in range(200):

            theta = i / 30 * pi
            dx = (0.1 + 0.25 * theta) * cos(theta)
            dy = (0.1 + 0.25 * theta) * sin(theta)

            dx /= 400
            dy /= 400

            # coord[0] = x, coord[1] = y, coord[2] = z
            coord[0] += scale * dx
            coord[1] += scale * dy
            p.move_to_cartesian_pose(coord)

def main():

    try:

        # Make an object of the Practice class we made above
        tut = Practice()
        tut.movement()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()