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

geometry_msgs::Pose
pose_2d_to_3d(const geometry_msgs::Pose2D& pose_2d);


std::vector<moveit_msgs::Grasp>
compute_arm_ee_pick_grasp(const geometry_msgs::Pose& object_pose,
                          const geometry_msgs::Pose2D& base_pose,
                          double grip_closure);

trajectory_msgs::JointTrajectory open_gripper();
trajectory_msgs::JointTrajectory close_gripper(double grip_closure);

void update_collision_objects(
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    const geometry_msgs::Pose& base_pose,
    const geometry_msgs::Pose& object_pose,
    const geometry_msgs::Pose& table_pose);
void add_collision_objects(
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    const geometry_msgs::Pose& object_pose,
    const geometry_msgs::Pose& table_pose);

void pick(moveit::planning_interface::MoveGroupInterface& move_group,
          const std::vector<moveit_msgs::Grasp>& grasp_pose);

void place(moveit::planning_interface::MoveGroupInterface& group,
           const geometry_msgs::Pose& place_pose,
           const geometry_msgs::Pose& object_pose,
           const geometry_msgs::Pose& base_pose);


// Dependencies
struct references {
  geometry_msgs::Pose2D base;
};

struct target {
  geometry_msgs::Pose2D base;
  std::vector<moveit_msgs::Grasp> arm_ee_pick;
  geometry_msgs::Pose arm_ee_place;
  geometry_msgs::Pose object_pose;
  double grip_closure;
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
  pick(group, target.arm_ee_pick);
  ROS_INFO("calling place");
  place(group, target.arm_ee_place, target.object_pose,
        pose_2d_to_3d(target.base));
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

  geometry_msgs::Pose object_pose;
  geometry_msgs::Pose table_pose;
  geometry_msgs::Pose2D base_des_pose;
  double object_theta;
  double table_theta;
  double grip_closure;

  nh.param<double>("/object/x", object_pose.position.x, 2.0);
  nh.param<double>("/object/y", object_pose.position.y, 2.0);
  nh.param<double>("/object/z", object_pose.position.z, 1.0);
  nh.param<double>("/grip_closure", grip_closure, 0.02425);
  nh.param<double>("/object/theta", object_theta, M_PI/4);
  nh.param<double>("/table/x", table_pose.position.x, 2.0);
  nh.param<double>("/table/y", table_pose.position.y, 2.0);
  nh.param<double>("/table/z", table_pose.position.z, 0.45);
  nh.param<double>("/table/theta", table_theta, M_PI/4);
  nh.param<double>("/base_des/x", base_des_pose.x, 1.55);
  nh.param<double>("/base_des/y", base_des_pose.y, 1.55);
  nh.param<double>("/base_des/theta", base_des_pose.theta, M_PI/4);

  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, object_theta);
  object_pose.orientation = tf2::toMsg(orientation);
  orientation.setRPY(0, 0, table_theta);
  table_pose.orientation = tf2::toMsg(orientation);

  ROS_INFO("Base desired pose: [%f, %f, %f]",
           base_des_pose.x, base_des_pose.y, base_des_pose.theta);

  target sm_target;
  sm_target.grip_closure = grip_closure;
  sm_target.object_pose = object_pose;
  sm_target.base = base_des_pose;
  sm_target.arm_ee_pick = compute_arm_ee_pick_grasp(object_pose, base_des_pose,
                                                    grip_closure);
  Eigen::Quaterniond place_orientation;
  place_orientation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ());
  sm_target.arm_ee_place.orientation = tf2::toMsg(place_orientation);
  sm_target.arm_ee_place.position.x = -0.33;
  sm_target.arm_ee_place.position.y = 0.20;
  sm_target.arm_ee_place.position.z = 0.65;

  references sm_references;

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  ROS_INFO_STREAM("planning frame " << group.getPlanningFrame());
  group.setPlanningTime(45.0);

  add_collision_objects(planning_scene_interface, object_pose, table_pose);
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
        update_collision_objects(planning_scene_interface, msg->pose.pose,
                                 object_pose, table_pose);
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

geometry_msgs::Pose pose_2d_to_3d(const geometry_msgs::Pose2D& pose_2d) {
  geometry_msgs::Pose pose;
  pose.position.x = pose_2d.x;
  pose.position.y = pose_2d.y;
  
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, pose_2d.theta);
  pose.orientation = tf2::toMsg(orientation);
  return pose;
}

std::vector<moveit_msgs::Grasp>
compute_arm_ee_pick_grasp(const geometry_msgs::Pose& object_pose,
                          const geometry_msgs::Pose2D& base_pose_2d,
                          double grip_closure) {
  geometry_msgs::Pose base_pose = pose_2d_to_3d(base_pose_2d);

  Eigen::Affine3d tf_fixed_object;
  Eigen::Affine3d tf_fixed_base;
  tf2::fromMsg(object_pose, tf_fixed_object);
  tf2::fromMsg(base_pose, tf_fixed_base);

  Eigen::Affine3d tf_base_object = tf_fixed_base.inverse() * tf_fixed_object;
  Eigen::Matrix3d rot_base_object = tf_base_object.rotation();

  Eigen::Matrix3d rot_0;
  rot_0 = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

  Eigen::Matrix3d rot_y_pi_half;
  rot_y_pi_half = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
  Eigen::Matrix3d rot_z_pi_quat;
  rot_z_pi_quat = Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitZ());

  constexpr int grasp_guesses = 4;
  std::vector<Eigen::Matrix3d> grasp_rotations(grasp_guesses);
  grasp_rotations[0] = rot_base_object * rot_0;
  for (int i = 1; i < grasp_rotations.size(); ++i) {
    grasp_rotations[i] = grasp_rotations[i - 1] * rot_y_pi_half;
  }

  std::vector<geometry_msgs::Pose> grasp_poses(grasp_guesses);
  for (int i = 0; i < grasp_poses.size(); ++i) {
    Eigen::Affine3d tf;
    tf.linear() = grasp_rotations[i] * rot_z_pi_quat;
    Eigen::Vector3d offset = {0, 0, -0.10};
    tf.translation() = tf_base_object.translation() + tf.linear() * offset;
    grasp_poses[i] = tf2::toMsg(tf);
    ROS_INFO("grasp_pose %d: position [%f %f %f]", i,
             grasp_poses[i].position.x, grasp_poses[i].position.y,
             grasp_poses[i].position.z);
    ROS_INFO("grasp_pose %d: orientation [%f %f %f %f]", i,
             grasp_poses[i].orientation.x, grasp_poses[i].orientation.y,
             grasp_poses[i].orientation.z, grasp_poses[i].orientation.w);
  }

  std::vector<moveit_msgs::Grasp> grasps(grasp_guesses);
  for (int i = 0; i < grasps.size(); ++i) {
    auto& grasp = grasps[i];
    grasp.grasp_pose.header.frame_id = "summit_xl_base_footprint";
    grasp.grasp_pose.pose = grasp_poses[i];

    grasp.pre_grasp_approach.direction.header.frame_id = "panda_link8";
    grasp.pre_grasp_approach.direction.vector.z = 1.0;
    grasp.pre_grasp_approach.min_distance = 0.095;
    grasp.pre_grasp_approach.desired_distance = 0.115;

    grasp.post_grasp_retreat.direction.header.frame_id = "summit_xl_base_footprint";
    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.min_distance = 0.1;
    grasp.post_grasp_retreat.desired_distance = 0.25;

    grasp.pre_grasp_posture = open_gripper();

    grasp.grasp_posture = close_gripper(grip_closure);
  }

  return grasps;
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
  posture.points[0].positions[0] = 0.05;
  posture.points[0].positions[1] = 0.05;
  posture.points[0].time_from_start = ros::Duration(0.5);
  return posture;
}

trajectory_msgs::JointTrajectory close_gripper(double grip_closure) {
  trajectory_msgs::JointTrajectory posture;
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = grip_closure / 2.0;
  posture.points[0].positions[1] = grip_closure / 2.0;
  posture.points[0].time_from_start = ros::Duration(0.5);
  return posture;
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group,
          const std::vector<moveit_msgs::Grasp>& grasps) {
  move_group.setSupportSurfaceName("table1");
  is_object_picked = true;
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group,
           const geometry_msgs::Pose& place_pose,
           const geometry_msgs::Pose& object_pose,
           const geometry_msgs::Pose& base_pose) {
  Eigen::Affine3d tf_base_ee_start;
  tf2::fromMsg(group.getCurrentPose().pose, tf_base_ee_start);

  Eigen::Affine3d tf_fixed_object;
  Eigen::Affine3d tf_fixed_base;
  tf2::fromMsg(object_pose, tf_fixed_object);
  tf2::fromMsg(base_pose, tf_fixed_base);

  Eigen::Affine3d tf_base_object_start = tf_fixed_base.inverse() * tf_fixed_object;

  Eigen::Affine3d tf_base_ee_end;
  tf2::fromMsg(place_pose, tf_base_ee_end);

  Eigen::Affine3d tf_base_object_end = tf_base_ee_end *
                                       tf_base_ee_start.inverse() *
                                       tf_base_object_start;
  ROS_INFO_STREAM("tf_fixed_object:\n " << tf_fixed_object.matrix());
  ROS_INFO_STREAM("tf_fixed_base:\n " << tf_fixed_base.matrix());
  ROS_INFO_STREAM("tf_base_object_start:\n " << tf_base_object_start.matrix());
  ROS_INFO_STREAM("tf_base_ee_end:\n " << tf_base_ee_end.matrix());
  ROS_INFO_STREAM("tf_base_object_end:\n " << tf_base_object_end.matrix());

  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);
  
  place_location[0].place_pose.header.frame_id = "summit_xl_base_footprint";
  place_location[0].place_pose.pose = tf2::toMsg(tf_base_object_end);
  place_location[0].place_pose.pose.position = place_pose.position;
  
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
    const geometry_msgs::Pose& base_pose,
    const geometry_msgs::Pose& object_pose,
    const geometry_msgs::Pose& table_pose) {  
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
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    const geometry_msgs::Pose& object_pose,
    const geometry_msgs::Pose& table_pose) {
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
  collision_objects[0].primitive_poses[0] = table_pose;

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
  collision_objects[2].primitive_poses[0] = object_pose;

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}