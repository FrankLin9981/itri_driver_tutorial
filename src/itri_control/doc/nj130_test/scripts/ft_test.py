#! /usr/bin/env python
import sys
import rospy
import tf
import moveit_commander
import copy
from math import pi
import geometry_msgs

def open_gripper(group, w):
   group.set_named_target("open")
   group.go(wait=w)
   group.stop()

def close_gripper(group, w):
   group.set_named_target("close")
   group.go(wait=w)
   group.stop()

def go_home(group, w):
   group.set_named_target("HOME")
   group.go(wait=w)
   group.stop()

def go_to_joint_state(group, joint_goal, w):
   group.go(joint_goal, wait=w)
   group.stop()

def go_to_pose_goal(group, pose_goal, w):
   group.set_pose_target(pose_goal)
   group.go(wait=w)
   group.stop()
   group.clear_pose_targets()

def plan_cartesian_path(group, waypoints):
   # We want the Cartesian path to be interpolated at a resolution of 1 cm
   # which is why we will specify 0.01 as the eef_step in Cartesian
   # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
   (plan, fraction) = group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
   # print(plan)
   # Note: We are just planning, not asking move_group to actually move the robot yet:
   return plan, fraction

def execute_plan(group, plan, w):
   ## Executing a Plan
   ## ^^^^^^^^^^^^^^^^
   ## Use execute if you would like the robot to follow
   ## the plan that has already been computed:
   group.execute(plan, wait=w)


def main():
   ## First initialize `moveit_commander`_ and a `rospy`_ node:
   moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
   rospy.init_node('move_group_simul_motion', anonymous=True)

   ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
   ## the robot:
   l_robot = moveit_commander.robot.RobotCommander("/left_manipulator/robot_description", ns="left_manipulator")
   r_robot = moveit_commander.robot.RobotCommander("/right_manipulator/robot_description", ns="right_manipulator")

   ## Instantiate a `MoveGroupCommander`_ object.
   left_arm = moveit_commander.MoveGroupCommander("manipulator", "/left_manipulator/robot_description", ns="left_manipulator")
   right_arm = moveit_commander.MoveGroupCommander("manipulator", "/right_manipulator/robot_description", ns="right_manipulator")
   left_hand = moveit_commander.MoveGroupCommander("gripper", "/left_manipulator/robot_description", ns="left_manipulator")
   right_hand = moveit_commander.MoveGroupCommander("gripper", "/right_manipulator/robot_description", ns="right_manipulator")

   rospy.sleep(5)
   
   joint_goal = left_arm.get_current_joint_values()
   joint_goal[4] = -pi/2
   go_to_joint_state(left_arm, joint_goal, True)

   rospy.sleep(5)

   joint_goal = left_arm.get_current_joint_values()
   joint_goal[4] = pi/2
   go_to_joint_state(left_arm, joint_goal, True)

   rospy.sleep(5)

   # Go home
   go_home(left_arm, True)
   
   rospy.sleep(1)

   moveit_commander.roscpp_initializer.roscpp_shutdown()
   
if __name__ == '__main__':
   main()