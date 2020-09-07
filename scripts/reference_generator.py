#!/usr/bin/env python
import rospy
import math
import random
import numpy as np
import geometry_msgs.msg
import std_msgs.msg
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import sys
import moveit_commander
import moveit_msgs.msg
import copy

base_goal_reached = None


def base_goal_reached_callback(msg):
  global base_goal_reached
  base_goal_reached = msg.data


def get_plan(move_group, scale=1):
  waypoints = []

  wpose = move_group.get_current_pose().pose
  wpose.position.x = 0.6
  wpose.position.y = 0
  wpose.position.z = 0.7
  # 0.859, -0.506, -0.070, 0.038
  wpose.orientation.x = 0.859
  wpose.orientation.y = -0.506
  wpose.orientation.z = -0.070
  wpose.orientation.w = 0.038
  n = 100
  radius = 0.05
  center_y = wpose.position.y
  center_x = wpose.position.x - radius
  thetas = np.linspace(0, n*2*np.pi, n*50)

  for theta in thetas:
    y = radius * math.cos(theta) + center_y
    x = radius * math.sin(theta) + center_x
    wpose.position.y = y
    wpose.position.x = x
    waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
  return plan


def get_random_base_pose():
  pose = geometry_msgs.msg.Pose2D()
  pose.x = 10 * random.random() - 5
  pose.y = 10 * random.random() - 5
  pose.theta = 2*np.pi * random.random() - np.pi
  return pose

def main():
  rospy.init_node('reference_generator')

  base_ref_pub = rospy.Publisher('/base_position_controller/pose_reference',
                                 geometry_msgs.msg.Pose2D, queue_size=1,
                                 tcp_nodelay=True)
  rospy.Subscriber('/base_position_controller/goal_reached',
                   std_msgs.msg.Bool, base_goal_reached_callback)
 #--------------------------------------------------------------
 # Generating the reference  joint goal for Franka arm
  #moveit_commander.roscpp_initialize(sys.argv)
  group_name = "panda_arm"
  move_group = moveit_commander.MoveGroupCommander(group_name)

  #joint_goal = move_group.get_current_joint_values()
  #joint_goal[0] = 0
  #joint_goal[1] = +np.pi/3
  #joint_goal[2] = 0
  #joint_goal[3] = -np.pi/2
  #joint_goal[4] = -np.pi/3
  #joint_goal[5] = np.pi/3
  #joint_goal[6] = 0

  #move_group.go(joint_goal, wait=True)
  #move_group.stop()
  #--------------------------------------------------------------
  # Generating a pose goal for Franka arm
  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.orientation.w = 1.0
  pose_goal.position.x = 0.3
  pose_goal.position.y = 0.2
  pose_goal.position.z = 0.6

  #move_group.set_pose_target(pose_goal)
  #(plan,fraction) = move_group.compute_cartesian_path(wait=True)
  #move_group.stop
  #move_group.clear_pose_targets()
  #move_group.execute(plan, wait=True)

  #--------------------------------------------------------------
  #group_name = "panda_arm"
  #move_group = moveit_commander.MoveGroupCommander(group_name)
  

  base_ref_msg = get_random_base_pose()
  rospy.loginfo("Waiting initial state")
  while not rospy.is_shutdown() and base_goal_reached is None:
    base_ref_pub.publish(base_ref_msg)
    rospy.sleep(1)
  rospy.loginfo("Go")

  rate = rospy.Rate(100)

  plan = get_plan(move_group)
  move_group.execute(plan, wait=False)

  last_base_goal_reached = base_goal_reached
  is_ref_sent = False
  while not rospy.is_shutdown():
    if last_base_goal_reached is True and base_goal_reached is False:
      is_ref_sent = False

    if base_goal_reached is True and not is_ref_sent:
      base_ref_msg = get_random_base_pose()
      base_ref_pub.publish(base_ref_msg)
      is_ref_sent = True

    last_base_goal_reached = base_goal_reached
    rate.sleep()
    

  return 0


if __name__ == '__main__':
  main()