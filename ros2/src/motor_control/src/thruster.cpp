#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;
using std::placeholders::_1;

class ThrusterNode : public rclcpp::Node
{
  public:
    ThrusterNode()
    : Node("thruster_node")
    {
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("joy/cmd_vel", 10, std::bind(&ThrusterNode::cmd_vel_callback, this, _1));
    }

  private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      double linear_x = msg->linear.x;
      double linear_y = msg->linear.y;
      double linear_z = msg->linear.z;
      double angular_x = msg->angular.x;
      double angular_y = msg->angular.y;
      double angular_z = msg->angular.z;

      RCLCPP_INFO(this->get_logger(), "I heard: '%f', '%f', '%f', '%f', '%f', '%f'", linear_x, linear_y, linear_z, angular_x, angular_y, angular_z);

      // TODO: send this data to the arduino
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterNode>());
  rclcpp::shutdown();
  return 0;
}
