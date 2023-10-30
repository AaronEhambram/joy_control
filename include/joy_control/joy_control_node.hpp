#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "youbot/YouBotBase.hpp"
#include "geometry_msgs/msg/twist.hpp"

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

    // velocity values
    double longitudinal = 0.0;
    double transversal = 0.0;
    double angular = 0.0;
};