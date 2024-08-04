#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "youbot/YouBotBase.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using namespace youbot;

class JoyController : public rclcpp::Node
{
  public:
    JoyController();
  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // joy subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    // cmd_vel publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    geometry_msgs::msg::Twist cmd_vel_msg_;

    // arm state publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_joint_state_publisher_;
    sensor_msgs::msg::JointState arm_joint_state_msg_;

    // velocity values
    double longitudinal = 0.0;
    double transversal = 0.0;
    double angular = 0.0;

    // Arm joint default
    const double arm_joint_0_fwd_default_ = 2.965;
    double arm_joint_0_offset_ = 0.0; 
};