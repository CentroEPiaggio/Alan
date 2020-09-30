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
#include <tf2_eigen/tf2_eigen.h>

#include "boost/sml.hpp"

constexpr double ARM_LENGTH = 1.0;

double is_base_goal_reached = false;
bool is_object_picked = false;


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
  ROS_INFO("calling pick");
  pick(group, target.arm_ee_pick, 0.025);
  ROS_INFO("calling place");
  place(group, target.arm_ee_place);
};

constexpr auto arm_to_ready = [](references& ref, target& target,
                                  moveit::planning_interface::MoveGroupInterface& group) {
  group.setNamedTarget("ready");
  group.move();
};

// State Machine
struct state_machine {
  auto operator()() const {
    using namespace boost::sml;

    return make_transition_table(
      *"waiting_go"_s + event<start>                  / move_base_to_home   = "robot_ready"_s,
      "robot_ready"_s + event<base_goal_reached>      / move_base_to_target = "base_approaching"_s,
      "base_approaching"_s + event<base_goal_reached> / do_pick_place       = "base_going_home"_s,
      "base_going_home"_s + on_entry<_>               / ( move_base_to_home, arm_to_ready ),
      "base_going_home"_s + event<base_goal_reached>                        = "waiting_go"_s
    );
  }
};

void base_goal_reached_callback(const std_msgs::Bool::ConstPtr& msg,
                                boost::sml::sm<state_machine>& sm);

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
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
  tf2::Quaternion pick_orientation;
  pick_orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  sm_target.arm_ee_pick.orientation = tf2::toMsg(pick_orientation);
  sm_target.arm_ee_pick.position.x = 0.405;
  sm_target.arm_ee_pick.position.y = 0.0;
  sm_target.arm_ee_pick.position.z = 1.0;

  tf2::Quaternion place_orientation;
  place_orientation.setRPY(M_PI, M_PI / 2, 0);
  sm_target.arm_ee_place.orientation = tf2::toMsg(place_orientation);
  sm_target.arm_ee_place.position.x = -0.33;
  sm_target.arm_ee_place.position.y = 0.20;
  sm_target.arm_ee_place.position.z = 0.55;

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
  Vector2d position = direction * (pos_2d_norm - ARM_LENGTH/2.0);
  
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

  grasps[0].grasp_pose.header.frame_id = "summit_xl_base_footprint";
  grasps[0].grasp_pose.pose = grasp_pose;

  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "summit_xl_base_footprint";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "summit_xl_base_footprint";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  grasps[0].pre_grasp_posture = open_gripper();

  grasps[0].grasp_posture = close_gripper(object_dimension);

  move_group.setSupportSurfaceName("table1");
  is_object_picked = true;
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group,
           const geometry_msgs::Pose& place_pose) {
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);  

  place_location[0].place_pose.header.frame_id = "summit_xl_base_footprint";
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
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
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
  geometry_msgs::Pose table_pose;
  tf2::Quaternion table_orientation;
  table_orientation.setRPY(0, 0, M_PI/4);
  table_pose.orientation = tf2::toMsg(table_orientation);
  table_pose.position.x = 2;
  table_pose.position.y = 2;
  table_pose.position.z = 0.45;

  geometry_msgs::Pose object_pose;
  tf2::Quaternion object_orientation;
  object_orientation.setRPY(0, 0, M_PI/4);
  object_pose.orientation = tf2::toMsg(object_orientation);
  object_pose.position.x = 2;
  object_pose.position.y = 2;
  object_pose.position.z = 1;
  
  Eigen::Affine3d tf_fixed_base;
  Eigen::Affine3d tf_fixed_table;
  Eigen::Affine3d tf_fixed_object;
  tf2::fromMsg(base_pose, tf_fixed_base);
  tf2::fromMsg(table_pose, tf_fixed_table);
  tf2::fromMsg(object_pose, tf_fixed_object);

  Eigen::Affine3d tf_base_table = tf_fixed_base.inverse() * tf_fixed_table;
  Eigen::Affine3d tf_base_object = tf_fixed_base.inverse() * tf_fixed_object;
  geometry_msgs::Pose table_body_pose = tf2::toMsg(tf_base_table);
  geometry_msgs::Pose object_body_pose = tf2::toMsg(tf_base_object);

  if (!is_object_picked) {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "summit_xl_base_footprint";

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0] = table_body_pose;
    collision_objects[0].operation = collision_objects[0].MOVE;

    // Define the object that we will be manipulating
    collision_objects[1].header.frame_id = "summit_xl_base_footprint";
    collision_objects[1].id = "object";

    /* Define the pose of the object. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0] = object_body_pose;

    collision_objects[1].operation = collision_objects[1].MOVE;

    planning_scene_interface.applyCollisionObjects(collision_objects);
  }

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