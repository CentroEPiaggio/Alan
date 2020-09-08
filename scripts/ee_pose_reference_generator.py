#!/usr/bin/env python
from __future__ import print_function
import rospy
import sys
import geometry_msgs.msg
import std_msgs.msg
import numpy as np
import math

import moveit_msgs.msg
import moveit_commander

ARM_LENGTH = 1.0

base_goal_reached = False
arm_goal_reached = False


class StateMachine:
  STATE_WAITING_GO = "WAITING_GO"
  STATE_BASE_APPROACHING = "BASE_APPROACHING"
  STATE_GRASPING = "GRASPING"
  STATE_ARM_RETURNING_TO_READY = "ARM_RETURNING_TO_READY"
  STATE_BASE_GOING_HOME = "BASE_GOING_HOME"
  STATE_WAIT_AFTER_GRASPED = "WAIT_AFTER_GRASPED"
  STATE_ROBOT_READY = "ROBOT_READY"

  EVENT_START = "START"
  EVENT_BASE_GOAL_REACHED = "BASE_GOAL_REACHED"
  EVENT_ARM_GOAL_REACHED = "ARM_GOAL_REACHED"
  EVENT_TIMER_END = "TIMER_END"

  ARM_DES_READY = "READY"
  BASE_DES_HOME = geometry_msgs.msg.Pose2D(x=0, y=0, theta=0)

  def __init__(self, base_des_pos, arm_des_pos):
    self._state = StateMachine.STATE_WAITING_GO
    self._base_reference = StateMachine.BASE_DES_HOME
    self._arm_reference = StateMachine.ARM_DES_READY

    self._base_des_pos = base_des_pos
    self._arm_des_pos = arm_des_pos
  
  def on_event(self, event):
    rospy.loginfo("New event {} - Current state {}".format(event, self._state))
    event = event.upper()
    if self._state == StateMachine.STATE_WAITING_GO and \
       event == StateMachine.EVENT_START:
      self._state = StateMachine.STATE_ROBOT_READY
      self._base_reference = StateMachine.BASE_DES_HOME
      self._arm_reference = StateMachine.ARM_DES_READY
    
    elif self._state == StateMachine.STATE_ROBOT_READY and \
         event == StateMachine.EVENT_BASE_GOAL_REACHED:
      self._state = StateMachine.STATE_BASE_APPROACHING
      self._base_reference = self._base_des_pos
      self._arm_reference = StateMachine.ARM_DES_READY
    
    elif self._state == StateMachine.STATE_BASE_APPROACHING and \
         event == StateMachine.EVENT_BASE_GOAL_REACHED:
      self._state = StateMachine.STATE_GRASPING
      self._base_reference = self._base_des_pos
      self._arm_reference = self._arm_des_pos

    elif self._state == StateMachine.STATE_GRASPING and \
         event == StateMachine.EVENT_ARM_GOAL_REACHED:
      self._state = StateMachine.STATE_WAIT_AFTER_GRASPED
      self._base_reference = self._base_des_pos
      self._arm_reference = self._arm_des_pos
      rospy.Timer(rospy.Duration(3),
                  lambda event: self.on_event(StateMachine.EVENT_TIMER_END),
                  oneshot=True)

    elif self._state == StateMachine.STATE_WAIT_AFTER_GRASPED and \
        event == StateMachine.EVENT_TIMER_END:
      self._state = StateMachine.STATE_ARM_RETURNING_TO_READY
      self._base_reference = self._base_des_pos
      self._arm_reference = StateMachine.ARM_DES_READY

    elif self._state == StateMachine.STATE_ARM_RETURNING_TO_READY and \
         event == StateMachine.EVENT_ARM_GOAL_REACHED:
      self._state = StateMachine.STATE_BASE_GOING_HOME
      self._base_reference = StateMachine.BASE_DES_HOME
      self._arm_reference = StateMachine.ARM_DES_READY

    elif self._state == StateMachine.STATE_BASE_GOING_HOME and \
         event == StateMachine.EVENT_BASE_GOAL_REACHED:
      self._state = StateMachine.STATE_WAITING_GO
      self._base_reference = StateMachine.BASE_DES_HOME
      self._arm_reference = StateMachine.ARM_DES_READY

    rospy.loginfo("New state {}".format(self._state))

  def get_state(self):
    return self._state
  
  def get_base_reference(self):
    return self._base_reference
  
  def get_arm_reference(self):
    return self._arm_reference
  
  def get_start_time(self):
    return self._start_time


def base_goal_reached_callback(msg, state_machine):
  global base_goal_reached
  prev_goal_reached = base_goal_reached
  base_goal_reached = msg.data
  if base_goal_reached and prev_goal_reached is False:
    rospy.loginfo("Base Goal Reached")
    state_machine.on_event(StateMachine.EVENT_BASE_GOAL_REACHED)

def arm_action_result_callback(msg, state_machine):
  global arm_goal_reached
  arm_goal_reached = (msg.status.status == msg.status.SUCCEEDED)
  if arm_goal_reached:
    rospy.loginfo("Arm Goal Reached")
    state_machine.on_event(StateMachine.EVENT_ARM_GOAL_REACHED)


def compute_base_des_pose(des_position):
  base_des_pose = geometry_msgs.msg.Pose2D()

  pos_2d = np.copy(des_position[0:2])
  rospy.loginfo(pos_2d)
  pos_2d_norm = np.linalg.norm(pos_2d)

  direction = pos_2d / pos_2d_norm
  position = direction * (pos_2d_norm - ARM_LENGTH/2)
  
  base_des_pose.x = position[0]
  base_des_pose.y = position[1]
  base_des_pose.theta = math.atan2(direction[1], direction[0]) + np.pi/2

  return base_des_pose

def compute_arm_ee_des_pose(des_position, base_pose):
  ee_des_position = geometry_msgs.msg.Pose()

  cos_th = math.cos(base_pose.theta)
  sin_th = math.sin(base_pose.theta)
  rot_matrix = np.array([
    [cos_th, sin_th],
    [-sin_th, cos_th]
  ])
  delta = np.array([des_position[0] - base_pose.x,
                    des_position[1] - base_pose.y]).transpose()
  delta_base = rot_matrix.dot(delta)

  ee_des_position.position.x = delta_base[0]
  ee_des_position.position.y = delta_base[1]
  ee_des_position.position.z = des_position[2]

  ee_des_position.orientation.x = 1.0
  ee_des_position.orientation.y = 0.0
  ee_des_position.orientation.z = 0.0
  ee_des_position.orientation.w = 0.0

  return ee_des_position

def main():
  global arm_goal_reached
  rospy.init_node('ee_pose_reference_generator')

  if len(sys.argv) < 4:
    raise Exception("Missing arguments")
  
  des_position = [0, 0, 0]
  des_position[0] = float(sys.argv[1])
  des_position[1] = float(sys.argv[2])
  des_position[2] = float(sys.argv[3])
  rospy.loginfo("Desired position: {}".format(des_position))

  base_des_pose = compute_base_des_pose(des_position)
  arm_ee_des_pose = compute_arm_ee_des_pose(des_position, base_des_pose)
  rospy.loginfo("Base desired pose: [{}, {}, {}]".format(
    base_des_pose.x, base_des_pose.y, base_des_pose.theta))
  rospy.loginfo("Arm desired EE position: [{}, {}, {}]".format(
    arm_ee_des_pose.position.x, arm_ee_des_pose.position.y,
    arm_ee_des_pose.position.z))
  state_machine = StateMachine(base_des_pose, arm_ee_des_pose)

  base_ref_pub = rospy.Publisher('/base_position_controller/pose_reference',
                                 geometry_msgs.msg.Pose2D, queue_size=1,
                                 tcp_nodelay=True)
  rospy.Subscriber('/base_position_controller/goal_reached',
                   std_msgs.msg.Bool, base_goal_reached_callback,
                   callback_args=state_machine)
  rospy.Subscriber('/move_group/result',
                   moveit_msgs.msg.MoveGroupActionResult,
                   arm_action_result_callback,
                   callback_args=state_machine)

  robot = moveit_commander.RobotCommander()
  move_group = moveit_commander.MoveGroupCommander("panda_arm")
  print("Planning frame is {}".format(move_group.get_planning_frame()))
  
  rate = rospy.Rate(100)
  state_machine.on_event(StateMachine.EVENT_START)

  last_arm_reference = None
  while not rospy.is_shutdown():    
    # Arm Control
    if last_arm_reference is not state_machine.get_arm_reference():
      arm_reference = state_machine.get_arm_reference()
      rospy.loginfo("Planning arm to {}".format(arm_reference))
      if isinstance(arm_reference, str) and arm_reference == "READY":
        move_group.set_named_target("ready")
      else:
        move_group.set_pose_target(arm_reference)
      
      rospy.loginfo("Commanding the arm")
      plan = move_group.go(wait=False)
      rospy.loginfo("Plan is {}".format(plan))
      arm_goal_reached = False

    # Base control
    base_ref_msg = state_machine.get_base_reference()
    base_ref_pub.publish(base_ref_msg)

    # Preparing for next iteration
    last_arm_reference = state_machine.get_arm_reference()
    rate.sleep()

if __name__ == '__main__':
  main()
