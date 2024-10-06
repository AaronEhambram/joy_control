#include <memory>
#include "joy_control/joy_control_node.hpp"

using std::placeholders::_1;
using namespace youbot;

JoyController::JoyController() : Node("joy_control_node")
{
  // setup the subscriber
  subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoyController::topic_callback, this, _1));

  // setup publisher
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  arm_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("arm_joint_state", 10);

  // initial velocities set to 0.0
  longitudinal = 0.0;
  transversal = 0.0;
  angular = 0.0;
  cmd_vel_msg_.angular.x = 0.0;
  cmd_vel_msg_.angular.y = 0.0;
  cmd_vel_msg_.angular.z = 0.0;
  cmd_vel_msg_.linear.x = 0.0;
  cmd_vel_msg_.linear.y = 0.0;
  cmd_vel_msg_.linear.z = 0.0;

  // prepare arm joint state msg
  arm_joint_state_msg_.name.resize(7);
  arm_joint_state_msg_.position.resize(7);
  // arm
  arm_joint_state_msg_.name[0] = "arm_joint_1";
  arm_joint_state_msg_.name[1] = "arm_joint_2";
  arm_joint_state_msg_.name[2] = "arm_joint_3";
  arm_joint_state_msg_.name[3] = "arm_joint_4";
  arm_joint_state_msg_.name[4] = "arm_joint_5";
  // gripper
  arm_joint_state_msg_.name[5] = "gripper_finger_joint_l";
  arm_joint_state_msg_.name[6] = "gripper_finger_joint_r";
}

void JoyController::topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  longitudinal = 0.0;
  transversal = 0.0;
  angular = 0.0;
  if (msg->buttons[7] == 1)
  {
    // clip the values (due to deadzone)
    longitudinal = (abs(msg->axes[3]) < 0.07) ? 0.0 : msg->axes[3];
    transversal = (abs(msg->axes[2]) < 0.07) ? 0.0 : msg->axes[2];
    angular = (abs(msg->axes[0]) < 0.07) ? 0.0 : msg->axes[0];
  }

  // set he base velocity
  quantity<si::velocity> longitudinalVelocity = (longitudinal) * 0.3 * meter_per_second;
  quantity<si::velocity> transversalVelocity = (transversal) * 0.3 * meter_per_second;
  quantity<si::angular_velocity> angularVelocity = (angular) * 0.8 * radian_per_second;
  // myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
  cmd_vel_msg_.angular.z = angularVelocity.value();
  cmd_vel_msg_.linear.x = longitudinalVelocity.value();
  cmd_vel_msg_.linear.y = transversalVelocity.value();
  cmd_vel_publisher_->publish(cmd_vel_msg_);

  if (msg->buttons[1] == 1)
  {
    arm_joint_state_msg_.position[0] = 0.11;
    arm_joint_state_msg_.position[1] = 0.11;
    arm_joint_state_msg_.position[2] = -0.11;
    arm_joint_state_msg_.position[3] = 0.11;
    arm_joint_state_msg_.position[4] = 0.12;
    arm_joint_state_msg_.position[5] = 0.0;
    arm_joint_state_msg_.position[6] = 0.0;
    arm_joint_state_publisher_->publish(arm_joint_state_msg_);
    arm_joint_0_offset_ = 0.0;
    arm_joint_3_offset_ = 0.0;
    arm_joint_4_offset_ = 0.0;
  }
  else if (msg->buttons[2] == 1)
  {
    arm_joint_state_msg_.position[0] = arm_joint_0_fwd_default_;
    arm_joint_state_msg_.position[1] = 1.331;
    arm_joint_state_msg_.position[2] = -1.852;
    arm_joint_state_msg_.position[3] = arm_joint_3_fwd_default_;
    arm_joint_state_msg_.position[4] = arm_joint_4_fwd_default_;
    arm_joint_state_msg_.position[5] = 0.0115;
    arm_joint_state_msg_.position[6] = 0.0115;
    arm_joint_state_publisher_->publish(arm_joint_state_msg_);
    arm_joint_0_offset_ = 0.0;
    arm_joint_3_offset_ = 0.0;
    arm_joint_4_offset_ = 0.0;
  }
  else if (msg->buttons[4] == 1)
  {
    // L1
    if (arm_joint_0_fwd_default_ + arm_joint_0_offset_ > 0.11)
    {
      arm_joint_0_offset_ -= 0.01;
      arm_joint_state_msg_.position[0] = arm_joint_0_fwd_default_ + arm_joint_0_offset_;
      arm_joint_state_msg_.position[1] = 1.331;
      arm_joint_state_msg_.position[2] = -1.852;
      arm_joint_state_msg_.position[3] = arm_joint_3_fwd_default_ + arm_joint_3_offset_;
      arm_joint_state_msg_.position[4] = arm_joint_4_fwd_default_ + arm_joint_4_offset_;
      arm_joint_state_msg_.position[5] = 0.0115;
      arm_joint_state_msg_.position[6] = 0.0115;
      arm_joint_state_publisher_->publish(arm_joint_state_msg_);
    }
  }
  else if (msg->buttons[5] == 1)
  {
    // R1
    if (arm_joint_0_fwd_default_ + arm_joint_0_offset_ < 5.8)
    {
      arm_joint_0_offset_ += 0.01;
      arm_joint_state_msg_.position[0] = arm_joint_0_fwd_default_ + arm_joint_0_offset_;
      arm_joint_state_msg_.position[1] = 1.331;
      arm_joint_state_msg_.position[2] = -1.852;
      arm_joint_state_msg_.position[3] = arm_joint_3_fwd_default_ + arm_joint_3_offset_;
      arm_joint_state_msg_.position[4] = arm_joint_4_fwd_default_ + arm_joint_4_offset_;
      arm_joint_state_msg_.position[5] = 0.0115;
      arm_joint_state_msg_.position[6] = 0.0115;
      arm_joint_state_publisher_->publish(arm_joint_state_msg_);
    }
  }

  if (msg->axes[5] == 1)
  {
    // cross up 0.0221239 and 3.4292
    if (arm_joint_3_fwd_default_ + arm_joint_3_offset_ > 0.03)
    {
      arm_joint_3_offset_ -= 0.01;
      arm_joint_state_msg_.position[0] = arm_joint_0_fwd_default_ + arm_joint_0_offset_;
      arm_joint_state_msg_.position[1] = 1.331;
      arm_joint_state_msg_.position[2] = -1.852;
      arm_joint_state_msg_.position[3] = arm_joint_3_fwd_default_ + arm_joint_3_offset_;
      arm_joint_state_msg_.position[4] = arm_joint_4_fwd_default_ + arm_joint_4_offset_;
      arm_joint_state_msg_.position[5] = 0.0115;
      arm_joint_state_msg_.position[6] = 0.0115;
      arm_joint_state_publisher_->publish(arm_joint_state_msg_);
    }
  }
  else if (msg->axes[5] == -1)
  {
    // cross down
    if (arm_joint_3_fwd_default_ + arm_joint_3_offset_ < 3.42)
    {
      arm_joint_3_offset_ += 0.01;
      arm_joint_state_msg_.position[0] = arm_joint_0_fwd_default_ + arm_joint_0_offset_;
      arm_joint_state_msg_.position[1] = 1.331;
      arm_joint_state_msg_.position[2] = -1.852;
      arm_joint_state_msg_.position[3] = arm_joint_3_fwd_default_ + arm_joint_3_offset_;
      arm_joint_state_msg_.position[4] = arm_joint_4_fwd_default_ + arm_joint_4_offset_;
      arm_joint_state_msg_.position[5] = 0.0115;
      arm_joint_state_msg_.position[6] = 0.0115;
      arm_joint_state_publisher_->publish(arm_joint_state_msg_);
    }
  }

  if(msg->axes[4] == 1)
  {
    // cross left 0.110619 and 5.64159
    if (arm_joint_4_fwd_default_ + arm_joint_4_offset_ < 5.64)
    {
      arm_joint_4_offset_ += 0.01;
      arm_joint_state_msg_.position[0] = arm_joint_0_fwd_default_ + arm_joint_0_offset_;
      arm_joint_state_msg_.position[1] = 1.331;
      arm_joint_state_msg_.position[2] = -1.852;
      arm_joint_state_msg_.position[3] = arm_joint_3_fwd_default_ + arm_joint_3_offset_;
      arm_joint_state_msg_.position[4] = arm_joint_4_fwd_default_ + arm_joint_4_offset_;
      arm_joint_state_msg_.position[5] = 0.0115;
      arm_joint_state_msg_.position[6] = 0.0115;
      arm_joint_state_publisher_->publish(arm_joint_state_msg_);
    }
  }
  else if(msg->axes[4] == -1)
  {
    // cross right
    if (arm_joint_4_fwd_default_ + arm_joint_4_offset_ > 0.12)
    {
      arm_joint_4_offset_ -= 0.01;
      arm_joint_state_msg_.position[0] = arm_joint_0_fwd_default_ + arm_joint_0_offset_;
      arm_joint_state_msg_.position[1] = 1.331;
      arm_joint_state_msg_.position[2] = -1.852;
      arm_joint_state_msg_.position[3] = arm_joint_3_fwd_default_ + arm_joint_3_offset_;
      arm_joint_state_msg_.position[4] = arm_joint_4_fwd_default_ + arm_joint_4_offset_;
      arm_joint_state_msg_.position[5] = 0.0115;
      arm_joint_state_msg_.position[6] = 0.0115;
      arm_joint_state_publisher_->publish(arm_joint_state_msg_);
    }
  }

  SLEEP_MILLISEC(15);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyController>());
  rclcpp::shutdown();
  return 0;
}