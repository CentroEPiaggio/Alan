#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include<ros/ros.h>

#include <vector>

namespace alan {

class JointVelocityController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
  bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &n) {
    std::vector<std::string> joint_names;
    if (!n.getParam("joint_names", joint_names)) {
      ROS_ERROR("Could not read joint names from param server");
      return false;
    }

    // retrieve gains
    if (!n.getParam("gains", gains_vec_)) {
      ROS_ERROR("Could not read joint gains from param server");
      return false;
    }

    for (auto &joint_name : joint_names) {
      joint_handles_.push_back(hw->getHandle(joint_name));
    }

    for (auto &joint_handle : joint_handles_) {
      command_.push_back(joint_handle.getPosition());
    }

    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>(std::string("/arm_command"), 1,
                                                            &JointVelocityController::setCommandCallback, this);

    return true;
  }

  void update(const ros::Time &time, const ros::Duration &period) {
    static double t_prev = ros::Time::now().toSec();
    double t_cur = ros::Time::now().toSec();
    static std::vector<double> prev_position_commanded(7);
    static bool ok = true;
    for (size_t i = 0; i < joint_handles_.size(); i++) {
      if (ok)
        prev_position_commanded[i] = joint_handles_.at(i).getPosition();
      double position_command = prev_position_commanded[i] + command_.at(i)*(t_cur - t_prev);
      double error = position_command - joint_handles_.at(i).getPosition();
      double commanded_effort = error * gains_vec_.at(i);
      ROS_INFO("Joint %d - error %f - effort %f - position %f", i, error, commanded_effort, position_command);
      joint_handles_.at(i).setCommand(commanded_effort);
      prev_position_commanded[i] = position_command;
    } 
    t_prev = t_cur;
    ok = false;
  }

  void setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg) { command_ = msg->data; }

  void starting(const ros::Time &time) {}

  void stopping(const ros::Time &time) {}

private:
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<double> gains_vec_;
  std::vector<double> command_;
  ros::Subscriber sub_command_;
};

PLUGINLIB_EXPORT_CLASS(alan::JointVelocityController, controller_interface::ControllerBase);

} // namespace alan