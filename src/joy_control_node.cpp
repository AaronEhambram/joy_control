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
  SLEEP_MILLISEC(10);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyController>());
  rclcpp::shutdown();
  return 0;
}