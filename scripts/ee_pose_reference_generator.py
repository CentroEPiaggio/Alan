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

base_goal_reached = None
arm_goal_reached = None

def base_goal_reached_callback(msg):
  global base_goal_reached
  base_goal_reached = msg.data

def arm_action_result_callback(msg):
  global arm_goal_reached
  arm_goal_reached = (msg.status.status == msg.status.SUCCEEDED)
  rospy.loginfo("arm goal reached {}".format(arm_goal_reached))


class StateMachine:
  STATE_WAITING_GO = "WAITING_GO"
  STATE_BASE_APPROACHING = "BASE_APPROACHING"
  STATE_GRASPING = "GRASPING"
  STATE_ARM_RETURNING_TO_READY = "ARM_RETURNING_TO_READY"
  STATE_BASE_GOING_HOME = "BASE_GOING_HOME"

  EVENT_START = "START"
  EVENT_BASE_GOAL_REACHED = "BASE_GOAL_REACHED"
  EVENT_ARM_GOAL_REACHED = "ARM_GOAL_REACHED"
  EVENT_TIMER_END = "TIMER_END"

  def __init__(self, base_des_pos, arm_des_pos):
    self._state = "WAITING_GO"
    self._base_reference = np.array([0, 0, 0])
    self._arm_reference = "READY"

    self._base_des_pos = base_des_pos
    self._arm_des_pos = arm_des_pos
  
  def on_event(self, event):
    rospy.loginfo("New event {} - Current state {}".format(event, self._state))
    event = event.upper()
    if self._state == "WAITING_GO" and event == "START":
      self._state = "BASE_APPROACHING"
      self._base_reference = self._base_des_pos
      self._arm_reference = "READY"
    
    elif self._state == "BASE_APPROACHING" and event == "BASE_GOAL_REACHED":
      self._state = "GRASPING"
      self._base_reference = self._base_des_pos
      self._arm_reference = self._arm_des_pos

    elif self._state == "GRASPING" and event == "ARM_GOAL_REACHED":
      self._state = "WAIT_AFTER_GRASPED"
      self._base_reference = self._base_des_pos
      self._arm_reference = self._arm_des_pos
      rospy.Timer(rospy.Duration(3),
                  lambda event: self.on_event("TIMER_END"),
                  oneshot=True)

    elif self._state == "WAIT_AFTER_GRASPED" and event == "TIMER_END":
      self._state = "ARM_RETURNING_TO_READY"
      self._base_reference = self._base_des_pos
      self._arm_reference = "READY"

    elif self._state == "ARM_RETURNING_TO_READY" and event == "ARM_GOAL_REACHED":
      self._state = "BASE_GOING_HOME"
      self._base_reference = np.array([0, 0, 0])
      self._arm_reference = "READY"

    elif self._state == "BASE_GOING_HOME" and event == "BASE_GOAL_REACHED":
      self._state = "WAITING_GO"
      self._base_reference = np.array([0, 0, 0])
      self._arm_reference = "READY"

    rospy.loginfo("New state {}".format(self._state))

  def get_state(self):
    return self._state
  
  def get_base_reference(self):
    return self._base_reference
  
  def get_arm_reference(self):
    return self._arm_reference
  
  def get_start_time(self):
    return self._start_time


def compute_des_position(des_position):
  arm_length = 1.0

  des_pos_projection = np.copy(des_position)
  des_pos_projection[2] = 0

  direction = des_pos_projection / np.linalg.norm(des_pos_projection)
  des_yaw = math.atan2(direction[1], direction[0])
  des_base_pos = direction * (np.linalg.norm(des_pos_projection) - arm_length/2)

  arm_des_pos = des_position - des_base_pos
  des_base_pos[2] = des_yaw

  return des_base_pos, arm_des_pos

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

  base_des_pos, arm_des_pos = compute_des_position(des_position)
  rospy.loginfo("Base desired pose: {}".format(base_des_pos))
  rospy.loginfo("Arm desired EE position: {}".format(arm_des_pos))

  base_ref_pub = rospy.Publisher('/base_position_controller/pose_reference',
                                 geometry_msgs.msg.Pose2D, queue_size=1,
                                 tcp_nodelay=True)
  rospy.Subscriber('/base_position_controller/goal_reached',
                   std_msgs.msg.Bool, base_goal_reached_callback)
  rospy.Subscriber('/move_group/result',
                   moveit_msgs.msg.MoveGroupActionResult,
                   arm_action_result_callback)

  robot = moveit_commander.RobotCommander()
  move_group = moveit_commander.MoveGroupCommander("panda_arm")
  print("Planning frame is {}".format(move_group.get_planning_frame()))

  state_machine = StateMachine(base_des_pos, arm_des_pos)

  rate = rospy.Rate(100)

  state_machine.on_event(StateMachine.EVENT_START)
  
  base_ref_msg = geometry_msgs.msg.Pose2D()
  last_base_goal_reached = False
  last_arm_goal_reached = False
  last_arm_reference = None
  while not rospy.is_shutdown():
    if last_base_goal_reached is False and base_goal_reached is True:
      state_machine.on_event(StateMachine.EVENT_BASE_GOAL_REACHED)
    if last_arm_goal_reached is False and arm_goal_reached is True:
      state_machine.on_event(StateMachine.EVENT_ARM_GOAL_REACHED)
    
    if last_arm_reference is not state_machine.get_arm_reference():
      arm_reference = state_machine.get_arm_reference()
      rospy.loginfo("Planning arm to {}".format(arm_reference))
      if isinstance(arm_reference, str) and arm_reference == "READY":
        move_group.set_named_target("ready")
      else:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 1.0
        pose_goal.position.x = arm_reference[0]
        pose_goal.position.y = arm_reference[1]
        pose_goal.position.z = arm_reference[2]
        move_group.set_pose_target(pose_goal)
      
      rospy.loginfo("Commanding the arm")
      plan = move_group.go(wait=False)
      rospy.loginfo("Plan is {}".format(plan))
      arm_goal_reached = False

    base_ref = state_machine.get_base_reference()
    base_ref_msg.x = base_ref[0]
    base_ref_msg.y = base_ref[1]
    base_ref_msg.theta = base_ref[2]

    base_ref_pub.publish(base_ref_msg)
    last_base_goal_reached = base_goal_reached
    last_arm_goal_reached = arm_goal_reached
    last_arm_reference = state_machine.get_arm_reference()
    rate.sleep()

if __name__ == '__main__':
  main()
