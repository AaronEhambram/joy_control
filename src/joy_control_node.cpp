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
  arm_joint_state_msg_.name[0] = "arm_joint_1"; arm_joint_state_msg_.name[1] = "arm_joint_2"; arm_joint_state_msg_.name[2] = "arm_joint_3";
  arm_joint_state_msg_.name[3] = "arm_joint_4"; arm_joint_state_msg_.name[4] = "arm_joint_5"; 
  // gripper
  arm_joint_state_msg_.name[5] = "gripper_finger_joint_l"; arm_joint_state_msg_.name[6] = "gripper_finger_joint_r"; 
}

void JoyController::topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  longitudinal = 0.0;
  transversal = 0.0;
  angular = 0.0;
  if(msg->buttons[7] == 1)
  {
    // clip the values (due to deadzone)
    longitudinal = (abs(msg->axes[3]) < 0.07) ? 0.0 : msg->axes[3];
    transversal = (abs(msg->axes[2]) < 0.07) ? 0.0 : msg->axes[2];
    angular = (abs(msg->axes[0]) < 0.07) ? 0.0 : msg->axes[0];
  }

  // set he base velocity
  quantity<si::velocity> longitudinalVelocity = (longitudinal)*0.3 * meter_per_second;
  quantity<si::velocity> transversalVelocity = (transversal)*0.3 * meter_per_second;
  quantity<si::angular_velocity> angularVelocity = (angular)*0.8 * radian_per_second;
  //myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
  cmd_vel_msg_.angular.z = angularVelocity.value(); 
  cmd_vel_msg_.linear.x = longitudinalVelocity.value();
  cmd_vel_msg_.linear.y = transversalVelocity.value(); 
  cmd_vel_publisher_->publish(cmd_vel_msg_);

  if(msg->buttons[1] == 1)
  {
    arm_joint_state_msg_.position[0] = 0.11;
    arm_joint_state_msg_.position[1] = 0.11;
    arm_joint_state_msg_.position[2] = -0.11;
    arm_joint_state_msg_.position[3] = 0.11;
    arm_joint_state_msg_.position[4] = 0.12;
    arm_joint_state_msg_.position[5] = 0.0;
    arm_joint_state_msg_.position[6] = 0.0;
    arm_joint_state_publisher_->publish(arm_joint_state_msg_);
  }
  else if(msg->buttons[2] == 1)
  {
    arm_joint_state_msg_.position[0] = 2.965;
    arm_joint_state_msg_.position[1] = 1.331;
    arm_joint_state_msg_.position[2] = -1.852;
    arm_joint_state_msg_.position[3] = 2.797;
    arm_joint_state_msg_.position[4] = 2.871;
    arm_joint_state_msg_.position[5] = 0.0115;
    arm_joint_state_msg_.position[6] = 0.0115;
    arm_joint_state_publisher_->publish(arm_joint_state_msg_);
  }
  SLEEP_MILLISEC(15);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyController>());
  rclcpp::shutdown();
  return 0;
}