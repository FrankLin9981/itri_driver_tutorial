#!/usr/bin/env python
import pickle
import numpy as np
import rospy
import copy
import csv
import actionlib
import openravepy as orpy
from time import time, sleep
from IPython import embed
from openravepy import *
import tf.transformations as tr
import time
# Bimanual package
from bimanual.utils import utils
from bimanual.utils.loggers import TextColors
import bimanual.planners.cc_planner as ccp
# Message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import nj130_dataSubscriber

import os.path as path
model_path = path.abspath(path.join(path.dirname(__file__), "../models"))


def ros_trajectory_from_bimanual(left_robot, right_robot, traj):
   """
   Converts a trajectory computed by bimanual package into a ROS JointTrajectory message.
   @type  left_robot: orpy.robot
   @param right_robot: The robot name
   @type  left_robot: orpy.robot
   @param right_robot: The robot name
   @type  traj: C{CCTrajectory}
   @param traj: The input trajectory computed by bimanual package
   @rtype: tuple
   @return: Tuple contains ROS JointTrajectory messages for left and right robot respectively
   """
   left_ros_traj = JointTrajectory()
   right_ros_traj = JointTrajectory()
   left_dof = left_robot.GetActiveDOF()
   right_dof = right_robot.GetActiveDOF()
   # Copy waypoints
   for i in range(len(traj.bimanual_wpts[0])):
      # if i % (len(traj.bimanual_wpts[0])-1) != 0:
      #    continue
      left_waypoint = traj.bimanual_wpts[0][i]
      right_waypoint = traj.bimanual_wpts[1][i]
      timestamp = traj.timestamps[i]
      # Append waypoint
      left_ros_point = JointTrajectoryPoint()
      right_ros_point = JointTrajectoryPoint()
      left_ros_point.positions = left_waypoint
      left_ros_point.velocities = np.zeros(left_dof)
      right_ros_point.positions = right_waypoint
      right_ros_point.velocities = np.zeros(right_dof)
      left_ros_point.time_from_start = rospy.Duration(timestamp)
      right_ros_point.time_from_start = rospy.Duration(timestamp)
      left_ros_traj.points.append(left_ros_point)
      right_ros_traj.points.append(right_ros_point)
   return left_ros_traj, right_ros_traj


def saveTraj(cctraj):
   # CSV stream
   csvfile = open('/home/frank/Desktop/data/ref_js_left.csv', 'w+')
   w_ref_js_left = csv.writer(csvfile)
   w_ref_js_left.writerow(
       ['time', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])

   csvfile = open('/home/frank/Desktop/data/ref_js_right.csv', 'w+')
   w_ref_js_right = csv.writer(csvfile)
   w_ref_js_right.writerow(
       ['time', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])

   csvfile = open('/home/frank/Desktop/data/ref_obj_pose.csv', 'w+')
   w_ref_obj_pose = csv.writer(csvfile)
   w_ref_obj_pose.writerow(['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'w'])

   timestamps = cctraj.timestamps
   lie_traj = cctraj.lie_traj
   translation_traj = cctraj.translation_traj
   left_wpts = cctraj.bimanual_wpts[0]
   right_wpts = cctraj.bimanual_wpts[1]

   T_obj = np.eye(4)
   for (q_left, q_right, t) in zip(left_wpts, right_wpts, timestamps):
      js_left = list([t])
      js_right = list([t])
      obj_pose = list([t])
      T_obj[0:3, 0:3] = lie_traj.EvalRotation(t)
      T_obj[0:3, 3] = translation_traj.Eval(t)
      q_obj = tr.quaternion_from_matrix(T_obj).tolist()
      obj_pose.extend(T_obj[0:3, 3].tolist())
      obj_pose.extend(q_obj)
      js_left.extend(q_left.tolist())
      js_right.extend(q_right.tolist())
      
      w_ref_js_left.writerow(js_left)
      w_ref_js_right.writerow(js_right)
      w_ref_obj_pose.writerow(obj_pose)

if __name__ == "__main__":
   # Initialize a ROS node
   rospy.init_node('bimanual_planning')

   # Generic configuration
   np.set_printoptions(precision=10, suppress=True)

   # Load OpenRAVE environment
   env=orpy.Environment()
   env.SetViewer('qtcoin')
   env.Load(model_path + '/worlds/bimanual_setup.env.xml')
   Lshape=env.ReadKinBodyXMLFile(model_path + '/Lshape_regrasp.kinbody.xml')
   env.Add(Lshape)

   # Add collision checker
   env.SetCollisionChecker(orpy.RaveCreateCollisionChecker(env, 'ode'))

   # Setup robot and manipulator
   module=orpy.RaveCreateModule(env, 'urdf')
   left_nj130=module.SendCommand(
       'load {}/left_nj130_sensor.urdf {}/left_nj130.srdf'.format(model_path, model_path))
   right_nj130=module.SendCommand(
       'load {}/right_nj130_sensor.urdf {}/right_nj130.srdf'.format(model_path, model_path))

   left_robot=env.GetRobot(left_nj130)
   right_robot=env.GetRobot(right_nj130)
   left_manip=left_robot.SetActiveManipulator('gripper')
   right_manip=right_robot.SetActiveManipulator('gripper')
   left_manip.SetChuckingDirection([-1, -1])
   right_manip.SetChuckingDirection([-1, -1])
   utils.disable_gripper([left_robot, right_robot])
   utils.load_IK_model([left_robot, right_robot])

   left_basemanip=orpy.interfaces.BaseManipulation(left_robot)
   left_taskmanip=orpy.interfaces.TaskManipulation(left_robot)
   right_basemanip=orpy.interfaces.BaseManipulation(right_robot)
   right_taskmanip=orpy.interfaces.TaskManipulation(right_robot)

   # Set robot & object initial pose
   left_robot.SetTransform(np.array(
      [[1.,  0.,  0., -2.],
       [0.,  1.,  0.,  0.],
       [0.,  0.,  1.,  0.],
       [0.,  0.,  0.,  1.]]))
   right_robot.SetTransform(np.array(
      [[-1., 0.,  0.,  2.],
       [0., -1.,  0.,  0.],
       [0.,  0.,  1.,  0.],
       [0.,  0.,  0.,  1.]]))
   Lshape.SetTransform(np.array(
      [[1.,  0.,  0.,  0.],
       [0.,  1.,  0.,  0.],
       [0.,  0.,  1.,  1.5],
       [0.,  0.,  0.,  1.]]))
   left_robot.SetActiveDOFValues(
    [0.,  0.,  -1.57,
      0.,  0.,  0.])
   right_robot.SetActiveDOFValues(
    [0.,  0.,  -1.57,
      0.,  0.,  0.])

   # Release gripper & move both arms to grasp pose
   left_taskmanip.ReleaseFingers()
   left_robot.WaitForController(0)
   right_taskmanip.ReleaseFingers()
   right_robot.WaitForController(0)

   left_robot.SetActiveDOFValues(
    [-0.0656948016,  0.4977366403, -0.8535341474,
     -0.0700703429, -1.2158003159, -1.5464057115])
   right_robot.SetActiveDOFValues(
    [-0.,           0.4959780546, -0.8502039449,
      3.1415926536, 1.2165704365, 0.])

   left_taskmanip.CloseFingers()
   left_robot.WaitForController(0)
   right_taskmanip.CloseFingers()
   right_robot.WaitForController(0)

   ################## closed chain planning ###################
   obj_translation_limits=[[0.6, 0.6, 3.], [-0.6, -0.6, 0.136]]
   q_robots_start=[left_robot.GetActiveDOFValues(),
                     right_robot.GetActiveDOFValues()]
   q_robots_grasp=[left_robot.GetDOFValues()[0:2],
                     right_robot.GetDOFValues()[0:2]]
   T_obj_start=Lshape.GetTransform()
   T_obj_goal=np.array(
      [[0.433, -0.866,  0.25,  0.0],
      [0.75,  0.5,  0.433,  0.0],
      [-0.50,  0.0,  0.866,  1.8],
      [0.,  0.,  0.,  1.]])

   q_robots_goal=utils.compute_bimanual_goal_configs(
                     [left_robot, right_robot], Lshape, q_robots_start,
                        q_robots_grasp, T_obj_start, T_obj_goal)
   # embed()
   # exit(0)

   logger=TextColors(TextColors.INFO)
   ccplanner=ccp.CCPlanner(Lshape, [left_robot, right_robot], logger=logger,
                              planner_type='RRTConnect')
   ccquery=ccp.CCQuery(obj_translation_limits, q_robots_start,
                        q_robots_goal, q_robots_grasp, T_obj_start, nn=2,
                        step_size=0.3, velocity_scale=0.5,
                        enable_bw=True)
   ccplanner.set_query(ccquery)
   res=ccplanner.solve(timeout=20)

   ###################### Visualization #######################
   ccplanner.shortcut(ccquery, maxiter=20)
   ccplanner.visualize_cctraj(ccquery.cctraj, speed=1)

   embed()
   # exit(0)

   ################### Simulate in Gazebo #####################
   left_client=actionlib.SimpleActionClient('left_manipulator/manipulator_controller/follow_joint_trajectory',
                                                FollowJointTrajectoryAction)
   right_client=actionlib.SimpleActionClient('right_manipulator/manipulator_admittance_controller/follow_joint_trajectory',
                                                FollowJointTrajectoryAction)
   s1=left_client.wait_for_server(timeout=rospy.Duration(5.0))
   s2=right_client.wait_for_server(timeout=rospy.Duration(5.0))
   if(not s1 or not s2):
      rospy.logerr('Timed out waiting for Joint Trajectory'
                   ' Action Server to connect. Start the action server'
                   ' before running this node.')
      raise rospy.ROSException('JointTrajectoryController timed out')

   left_goal=FollowJointTrajectoryGoal()
   right_goal=FollowJointTrajectoryGoal()
   left_goal.trajectory.joint_names=[
       'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
   right_goal.trajectory.joint_names=[
       'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

   left_traj, right_traj=ros_trajectory_from_bimanual(left_robot, right_robot, ccquery.cctraj)
   left_goal.trajectory.points=copy.deepcopy(left_traj.points)
   right_goal.trajectory.points=copy.deepcopy(right_traj.points)
   left_goal.trajectory.header.stamp=rospy.Time.now() + rospy.Duration(0.1)
   right_goal.trajectory.header.stamp=rospy.Time.now() + rospy.Duration(0.1)
   left_client.send_goal(left_goal)
   right_client.send_goal(right_goal)
   
   saveTraj(ccquery.cctraj)
   nj130_dataSubscriber.getData()

   left_client.wait_for_result()
