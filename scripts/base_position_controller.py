#!/usr/bin/env python
import rospy
import math
import copy
import numpy as np
import geometry_msgs.msg
import std_msgs.msg
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

current_pose = None
reference_pose = None

max_vel = 3
max_ang_vel = 1
gain_position = 5
gain_orientation = 1


def wrapToPi(angle):
  while angle > np.pi:
    angle -= 2 * np.pi
  while angle < -np.pi:
    angle += 2 * np.pi
  return angle

def base_odom_callback(msg):
  global current_pose

  quaternion = np.array([
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
  ])
  angles = euler_from_quaternion(quaternion)

  current_pose = geometry_msgs.msg.Pose2D()
  current_pose.x = msg.pose.pose.position.x
  current_pose.y = msg.pose.pose.position.y
  current_pose.theta = wrapToPi(angles[2])


def reference_callback(msg):
  global reference_pose
  reference_pose = copy.deepcopy(msg)
  reference_pose.theta = wrapToPi(reference_pose.theta)


def pose_control(current, reference):
  cmd_msg = geometry_msgs.msg.Twist()

  current_position = np.array([current.x, current.y])
  reference_position = np.array([reference.x, reference.y])
  rot_0L = np.array([
    [math.cos(current.theta), math.sin(current.theta)],
    [-math.sin(current.theta), math.cos(current.theta)],
  ])

  err_position = reference_position - current_position
  if np.linalg.norm(err_position) > 0.05:
    reference_theta = wrapToPi(math.atan2(err_position[1], err_position[0]))
  else:
    reference_theta = reference.theta
  err_theta = wrapToPi(reference_theta - current.theta)

  cmd_position = gain_position * rot_0L.dot(err_position)
  cmd_position_norm = np.linalg.norm(cmd_position)
  if cmd_position_norm > max_vel:
    cmd_position = max_vel * cmd_position / cmd_position_norm
  
  cmd_theta = gain_orientation * err_theta
  if cmd_theta > max_ang_vel:
    cmd_theta = max_ang_vel
  
  if abs(err_theta) < 0.005:
    cmd_msg.linear.x = cmd_position[0]
    cmd_msg.linear.y = cmd_position[1]
    cmd_msg.angular.z = 0
  else:
    cmd_msg.linear.x = 0
    cmd_msg.linear.y = 0
    cmd_msg.angular.z = cmd_theta
  return cmd_msg


def is_goal_reached(current, reference):
  current_position = np.array([current.x, current.y])
  reference_position = np.array([reference.x, reference.y])
  err_position = reference_position - current_position
  err_theta = reference.theta - current.theta
  
  rospy.logdebug("err_position {}".format(np.linalg.norm(err_position)))
  rospy.logdebug("err_theta {}".format(abs(err_theta)))
  if np.linalg.norm(err_position) > 0.005 or abs(err_theta) > 0.005:
    return False
  return True


def main():
  global reference_pose
  rospy.init_node('base_position_controller')

  base_cmd_pub = rospy.Publisher('/robotnik_base_control/cmd_vel',
                                 geometry_msgs.msg.Twist,
                                 queue_size=1, tcp_nodelay=True)
  base_goal_pub = rospy.Publisher('/base_position_controller/goal_reached',
                                  std_msgs.msg.Bool,
                                  queue_size=1, tcp_nodelay=True)
  rospy.Subscriber('/robotnik_base_control/odom',
                   Odometry, base_odom_callback)
  rospy.Subscriber('/base_position_controller/pose_reference',
                   geometry_msgs.msg.Pose2D, reference_callback)
  
  rospy.loginfo("Waiting initial state")
  while not rospy.is_shutdown() and current_pose is None:
    rospy.sleep(1)

  reference_pose = current_pose

  rospy.loginfo("Commading...")

  rate = rospy.Rate(100.0)
  while not rospy.is_shutdown():
    cmd_msg = pose_control(current_pose, reference_pose)
    
    if is_goal_reached(current_pose, reference_pose):
      goal_reached_msg = std_msgs.msg.Bool(data=True)
    else:
      goal_reached_msg = std_msgs.msg.Bool(data=False)
    
    base_cmd_pub.publish(cmd_msg)
    base_goal_pub.publish(goal_reached_msg)

    try:
      rate.sleep()
    except:
      break
  
  return 0

if __name__ == '__main__':
  main()