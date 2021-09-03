#!/usr/bin/env python

# This script showcases how we can use Cartesian space to move a Panda arm.
# To use Cartesian space, we must use the movegroup examples. This is the case
# for both the physical Franka Emika Panda arm, as well as the Panda sim.
# This script shows how we can utilise an Archimedean spiral in task space.

# Import necessary packages
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# Pi, Cos, Sin functions necessary for Archimedean Spirals
from math import pi, cos, sin

# PandaArm() can be used to easily facilitate joint space movement
from panda_robot import PandaArm

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class Practice(object):

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("panda_sim")

        # RobotCommander and PlanningSceneInterface are necessary for Cartesian
        # movement
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        # Declaration of PandaArm
        p = PandaArm()

        # Return the Panda arm to a neutral pose before performing any kind of
        # movement function on it
        p.move_to_neutral()

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.p = p

    # Without planning the Cartesian path first, movement functions will NOT
    # be executedsuper(Practice, self).__init__()
    def plan_cartesian_path(self, scale=1):

        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose

        dx = 0.04
        dy = 0.06

        wpose.position.x += scale * dx
        wpose.position.y += scale * dy
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction
    
    def execute_plan(self, plan):

        p = self.p

        move_group = self.move_group

        move_group.execute(plan, wait=True)

        print(p.joint_angles())

def main():

    try:

        # Make an object of the Practice class we made above
        tut = Practice()

        cartesian_plan, fraction = tut.plan_cartesian_path()
        print(fraction)

        tut.execute_plan(cartesian_plan)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()