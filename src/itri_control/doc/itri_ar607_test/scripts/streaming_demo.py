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
   moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
   rospy.init_node('move_group_streaming', anonymous=True)
   robot = moveit_commander.robot.RobotCommander()

   arm_group = moveit_commander.MoveGroupCommander("manipulator")
   hand_group = moveit_commander.MoveGroupCommander("gripper")

   # Robot go home
   go_home(arm_group, True)

   # Pose goal
   quaternion = tf.transformations.quaternion_from_euler(-pi / 2, -pi / 2, 0)
   pose_goal = geometry_msgs.msg.Pose()
   pose_goal.orientation.x = quaternion[0]
   pose_goal.orientation.y = quaternion[1]
   pose_goal.orientation.z = quaternion[2]
   pose_goal.orientation.w = quaternion[3]
   pose_goal.position.x = 0.1
   pose_goal.position.y = 0.40
   pose_goal.position.z = 0.35
   go_to_pose_goal(arm_group, pose_goal, True)

   rospy.sleep(1)

   # Design a square trajectory
   waypoints = []
   scale = 1.0

   wpose = arm_group.get_current_pose().pose
   wpose.position.z += scale * 0.3  # First move up (z)
   waypoints.append(copy.deepcopy(wpose))

   wpose.position.x -= scale * 0.3  # Second move sideways in (x)
   waypoints.append(copy.deepcopy(wpose))

   wpose.position.z -= scale * 0.3  # Third move down (z)
   waypoints.append(copy.deepcopy(wpose))

   wpose.position.x += scale * 0.3
   waypoints.append(copy.deepcopy(wpose))

   # Planning
   plan, _ = plan_cartesian_path(arm_group, waypoints)

   execute_plan(arm_group, plan, True)

   # Robot go home
   go_home(arm_group, True)

   rospy.sleep(1)
   moveit_commander.roscpp_initializer.roscpp_shutdown()

if __name__ == '__main__':
   main()