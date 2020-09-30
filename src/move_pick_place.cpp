#include <ros/ros.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <functional>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

#include "boost/sml.hpp"

constexpr double ARM_LENGTH = 1.0;

double is_base_goal_reached = false;


geometry_msgs::Pose2D
compute_base_des_pose(const Eigen::Vector3d& des_position);

trajectory_msgs::JointTrajectory open_gripper();
trajectory_msgs::JointTrajectory close_gripper(double object_dimension);

void update_collision_objects(
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    const geometry_msgs::Pose& base_pose);
void add_collision_objects(
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

void pick(moveit::planning_interface::MoveGroupInterface& move_group,
          const geometry_msgs::Pose& grasp_pose,
          double object_dimension);

void place(moveit::planning_interface::MoveGroupInterface& group,
           const geometry_msgs::Pose& place_pose);


// Dependencies
struct references {
  geometry_msgs::Pose2D base;
};

struct target {
  geometry_msgs::Pose2D base;
  geometry_msgs::Pose arm_ee_pick;
  geometry_msgs::Pose arm_ee_place;
};

// Events 
struct start {};
struct base_goal_reached {};
struct pick_place_done {};

// Actions
auto move_base_to_home = [](references& ref, target& target){
  ROS_INFO("calling move_base_to_home");
  ref.base = geometry_msgs::Pose2D();
};

constexpr auto move_base_to_target = [](references& ref, target& target) {
  ROS_INFO("calling move_base_to_target");
  ref.base = target.base;
};

constexpr auto do_pick_place = [](references& ref, target& target,
                                  moveit::planning_interface::MoveGroupInterface& group) {
  ROS_INFO("calling do_pick_place");
  pick(group, target.arm_ee_pick, 0.25);
  ros::WallDuration(1.0).sleep();
  place(group, target.arm_ee_place);
};

// State Machine
struct state_machine {
  auto operator()() const {
    using namespace boost::sml;

    return make_transition_table(
      *"waiting_go"_s + event<start>                  / move_base_to_home   = "robot_ready"_s,
      "robot_ready"_s + event<base_goal_reached>      / move_base_to_target = "base_approaching"_s,
      "base_approaching"_s + event<base_goal_reached> / do_pick_place       = "grasping"_s,
      "grasping"_s + event<pick_place_done>           / move_base_to_home   = "base_going_home"_s,
      "base_going_home"_s + event<base_goal_reached>                        = "waiting_go"_s
    );
  }
};

void base_goal_reached_callback(const std_msgs::Bool::ConstPtr& msg,
                                boost::sml::sm<state_machine>& sm);

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (argc < 4) {
    throw std::runtime_error("Missing arguments");
  }

  Eigen::Vector3d des_position;
  des_position[0] = std::stod(argv[1]);
  des_position[1] = std::stod(argv[2]);
  des_position[2] = std::stod(argv[3]);
  /*ROS_INFO_STREAM("Desired position: [%d, %d, %d]",
           des_position[0], des_position[1], des_position[2]);*/
  ROS_INFO_STREAM("Desired position: " << des_position);

  target sm_target;
  sm_target.base = compute_base_des_pose(des_position);
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  sm_target.arm_ee_pick.orientation = tf2::toMsg(orientation);
  sm_target.arm_ee_pick.position.x = 2.0;
  sm_target.arm_ee_pick.position.y = 2.0;
  sm_target.arm_ee_pick.position.z = 1.0;

  // sm_target.arm_ee_place;
  
  references sm_references;

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  ROS_INFO_STREAM("planning frame " << group.getPlanningFrame());
  group.setPlanningTime(45.0);

  add_collision_objects(planning_scene_interface);
  ros::WallDuration(1.0).sleep();
  
  boost::sml::sm<state_machine> sm{sm_references, sm_target, group};

  auto base_ref_pub = nh.advertise<geometry_msgs::Pose2D>(
      "/base_position_controller/pose_reference", 1);
  
  auto base_goal_sub = nh.subscribe<std_msgs::Bool>(
      "/base_position_controller/goal_reached", 1,
      std::bind(base_goal_reached_callback,
                std::placeholders::_1,
                std::ref(sm)));
  
  auto base_odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "/robotnik_base_control/odom", 1,
      [&](const nav_msgs::Odometry::ConstPtr& msg) { 
        update_collision_objects(planning_scene_interface, msg->pose.pose);
      });

  sm.process_event(start{});

  ros::Timer base_ref_timer = nh.createTimer(
      ros::Duration(0.01),
      [&](const ros::TimerEvent&){ 
        base_ref_pub.publish(sm_references.base); 
      }, false);

  ros::waitForShutdown();
  return 0;
}

geometry_msgs::Pose2D
compute_base_des_pose(const Eigen::Vector3d& des_position) {
  using Eigen::Vector2d;
  Vector2d pos_2d = des_position.segment(0, 2);
  double pos_2d_norm = pos_2d.norm();

  Vector2d direction = pos_2d / pos_2d_norm;
  Vector2d position = direction * (pos_2d_norm - 2.0*ARM_LENGTH/3.0);
  
  geometry_msgs::Pose2D base_des_pose;
  base_des_pose.x = position[0];
  base_des_pose.y = position[1];
  base_des_pose.theta = std::atan2(direction[1], direction[0]);

  return base_des_pose;
}

void base_goal_reached_callback(const std_msgs::Bool::ConstPtr& msg,
                                boost::sml::sm<state_machine>& sm) {
  auto is_prev_goal_reached = is_base_goal_reached;
  is_base_goal_reached = msg->data;
  if (is_base_goal_reached && !is_prev_goal_reached) {
    ROS_INFO("Base Goal Reached");
    sm.process_event(base_goal_reached{});
  }
}

trajectory_msgs::JointTrajectory open_gripper() {
  trajectory_msgs::JointTrajectory posture;
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  return posture;
}

trajectory_msgs::JointTrajectory close_gripper(double object_dimension) {
  trajectory_msgs::JointTrajectory posture;
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.97 * object_dimension / 2.0;
  posture.points[0].positions[1] = 0.97 * object_dimension / 2.0;
  posture.points[0].time_from_start = ros::Duration(0.5);
  return posture;
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group,
          const geometry_msgs::Pose& grasp_pose,
          double object_dimension) {
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  grasps[0].grasp_pose.header.frame_id = "summit_xl_odom";
  grasps[0].grasp_pose.pose = grasp_pose;

  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "summit_xl_odom";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "summit_xl_odom";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  grasps[0].pre_grasp_posture = open_gripper();

  grasps[0].grasp_posture = close_gripper(object_dimension);

  move_group.setSupportSurfaceName("table1");
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group,
           const geometry_msgs::Pose& place_pose) {
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);  

  place_location[0].place_pose.header.frame_id = "panda_link0";
  place_location[0].place_pose.pose = place_pose;

  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "summit_xl_base_footprint";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "summit_xl_base_footprint";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  /* Similar to the pick case */
  place_location[0].post_place_posture = open_gripper();

  group.setSupportSurfaceName("summit");
  group.place("object", place_location);
}

void update_collision_objects(
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    const geometry_msgs::Pose& base_pose) {
  static geometry_msgs::Pose table1_pose_fixed;
  static double table1_yaw_fixed  = M_PI/4;
  tf2::Quaternion table1_orientation_fixed;
  table1_orientation_fixed.setRPY(0, 0, table1_yaw_fixed);
  table1_pose_fixed.orientation = tf2::toMsg(table1_orientation_fixed);
  table1_pose_fixed.position.x = 2;
  table1_pose_fixed.position.y = 2;
  table1_pose_fixed.position.z = 0.45;

  geometry_msgs::Point position_err_fixed;
  position_err_fixed.x = table1_pose_fixed.position.x - base_pose.position.x;
  position_err_fixed.y = table1_pose_fixed.position.y - base_pose.position.y;
  position_err_fixed.z = table1_pose_fixed.position.z - base_pose.position.z;

  double yaw_err = table1_yaw_fixed - tf::getYaw(base_pose.orientation);
  double cos_th = std::cos(tf::getYaw(base_pose.orientation));
  double sin_th = std::sin(tf::getYaw(base_pose.orientation));
  double table1_body_x = cos_th * position_err_fixed.x  + sin_th * position_err_fixed.y;
  double table1_body_y = -sin_th * position_err_fixed.x + cos_th * position_err_fixed.y;

  
  /*
  table1_pos_fixed = [2, 2, 0.45]
  table1_quat_fixed = [0, 0, 0, 1]
  */

  /*
  R_fixed_body = quat_to_rot(base_odom.orientation)
  base_fixed = base_odom.position
  */

  /*
  ?? table1_position_body, table1_ori_body

  err_position_fixed = table1_pos_fixed - base_fixed
  table1_position_body = (R_fixed_body)^T * err_position_fixed
  table1_quat_body = fn(table1_quat_fixed, base_quat_fixed)
  table1_quat_fixed = base_quat_fixed * table_quat_body
  table_quat_boyd = inv(base_quat_fixed) * table1_quat_fixed

  R_fixed_body
  table1_pos_body = R_fixed_body^T * table1_pos_fixed
  table_quat_body = 

  */

  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "summit_xl_base_footprint";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.9;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = table1_body_x;
  collision_objects[0].primitive_poses[0].position.y = table1_body_y;
  collision_objects[0].primitive_poses[0].position.z = table1_pose_fixed.position.z;
  tf2::Quaternion table1_orientation_body;
  table1_orientation_body.setEulerZYX(yaw_err, 0, 0);
  collision_objects[0].primitive_poses[0].orientation = tf2::toMsg(table1_orientation_body);

  collision_objects[0].operation = collision_objects[0].MOVE;

  // Define the object that we will be manipulating
  collision_objects[1].header.frame_id = "summit_xl_base_footprint";
  collision_objects[1].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.025;
  collision_objects[1].primitives[0].dimensions[1] = 0.025;
  collision_objects[1].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 2;
  collision_objects[1].primitive_poses[0].position.y = 2;
  collision_objects[1].primitive_poses[0].position.z = 1.0;

  collision_objects[2].operation = collision_objects[2].MOVE;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void add_collision_objects(
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "summit_xl_base_footprint";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.9;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 2;
  collision_objects[0].primitive_poses[0].position.y = 2;
  collision_objects[0].primitive_poses[0].position.z = 0.45;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "summit";
  collision_objects[1].header.frame_id = "summit_xl_base_footprint";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.75;
  collision_objects[1].primitives[0].dimensions[1] = 0.665;
  collision_objects[1].primitives[0].dimensions[2] = 0.50;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.0;
  collision_objects[1].primitive_poses[0].position.y = 0.0;
  collision_objects[1].primitive_poses[0].position.z = 0.25;

  collision_objects[1].operation = collision_objects[1].ADD;

  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "summit_xl_base_footprint";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.025;
  collision_objects[2].primitives[0].dimensions[1] = 0.025;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 2;
  collision_objects[2].primitive_poses[0].position.y = 2;
  collision_objects[2].primitive_poses[0].position.z = 1.0;

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}