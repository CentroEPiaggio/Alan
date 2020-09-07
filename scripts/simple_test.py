#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import sys
import moveit_commander
import moveit_msgs.msg
import copy

base_pose = None

def wrapToPi(angle):
  while angle > np.pi:
    angle -= 2 * np.pi
  while angle < -np.pi:
    angle += 2 * np.pi
  return angle


def base_odom_callback(msg):
  global base_pose
  quaternion = np.array([
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
  ])
  angles = euler_from_quaternion(quaternion)
  base_pose = np.array([
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
    angles[2]
  ])


def base_control(reference_position):
  base_cmd = Twist()

  max_vel = 1
  max_angular = 0.5
  gain = 1

  base_position = base_pose[[0,1]]
  yaw = wrapToPi(base_pose[2])
  abs_to_local = np.array([
    [math.cos(yaw), math.sin(yaw)],
    [-math.sin(yaw), math.cos(yaw)],
  ])

  err_pos = reference_position - base_position
  des_yaw = wrapToPi(math.atan2(err_pos[1], err_pos[0]))
  err_yaw = wrapToPi(des_yaw - yaw)

  cmd_pos = gain * abs_to_local.dot(err_pos)
  vel_norm = np.linalg.norm(cmd_pos)
  if vel_norm > max_vel:
    cmd_pos = max_vel * cmd_pos / vel_norm
  
  cmd_yaw = gain * err_yaw
  if cmd_yaw > max_angular:
    cmd_yaw = max_angular

  if abs(err_yaw) < 0.05 or np.linalg.norm(err_pos) < 0.05:
    # print("here - err_yaw {}".format(err_yaw))
    base_cmd.linear.x = cmd_pos[0]
    base_cmd.linear.y = cmd_pos[1]
    base_cmd.angular.z = 0
  else:
    base_cmd.linear.x = 0
    base_cmd.linear.y = 0
    base_cmd.angular.z = cmd_yaw
  return base_cmd


def get_plan(move_group, scale=1):
  waypoints = []

  wpose = move_group.get_current_pose().pose
  radius = 0.1
  center_y = wpose.position.y
  center_z = wpose.position.z - radius
  thetas = np.linspace(0, 10*np.pi, 250)

  for theta in thetas:
    y = radius * math.cos(theta) + center_y
    z = radius * math.sin(theta) + center_z
    wpose.position.y = y
    wpose.position.z = z
    waypoints.append(copy.deepcopy(wpose))

  #wpose = move_group.get_current_pose().pose
  #wpose.position.z -= scale * 0.1  # First move up (z)
  #wpose.position.y += scale * 0.2  # and sideways (y)
  #waypoints.append(copy.deepcopy(wpose))

  #wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
  #waypoints.append(copy.deepcopy(wpose))

  #wpose.position.y -= scale * 0.1  # Third move sideways (y)
  #waypoints.append(copy.deepcopy(wpose))

  # We want the Cartesian path to be interpolated at a resolution of 1 cm
  # which is why we will specify 0.01 as the eef_step in Cartesian
  # translation.  We will disable the jump threshold by setting it to 0.0,
  # ignoring the check for infeasible jumps in joint space, which is sufficient
  # for this tutorial.
  (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

  # Note: We are just planning, not asking move_group to actually move the robot yet:
  return plan

def main():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('simple_test', anonymous=True)

  base_cmd_pub = rospy.Publisher('/robotnik_base_control/cmd_vel', Twist,
                                 queue_size=1, tcp_nodelay=True)
  arm_cmd_pub = rospy.Publisher('/joint_position_controller/command',
                                Float64MultiArray, queue_size=1,
                                tcp_nodelay=True)
  rospy.Subscriber('/robotnik_base_control/odom',
                   Odometry, base_odom_callback)

  group_name = "panda_arm"
  move_group = moveit_commander.MoveGroupCommander(group_name)
  print("EE Link {}".format(move_group.get_end_effector_link()))
  
  rospy.loginfo("Waiting initial state")
  while not rospy.is_shutdown() and base_pose is None:
    rospy.sleep(1)

  rate = rospy.Rate(1000.0)

  plan = get_plan(move_group)
  move_group.execute(plan, wait=False)


  base_ref_position = np.array([0, 0])
  while not rospy.is_shutdown():
    base_msg = base_control(base_ref_position)
    
    arm_msg = Float64MultiArray()
    arm_msg.data = [0, 0, 0, -np.pi/2, 0, 3*np.pi/4, 0]

    base_cmd_pub.publish(base_msg)
    # arm_cmd_pub.publish(arm_msg)
    try:
      rate.sleep()
    except:
      break

  return 0


if __name__ == '__main__':
  main()