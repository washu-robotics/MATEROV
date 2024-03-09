#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cstring"


using namespace std;
using std::placeholders::_1;

class EKFNode : public rclcpp::Node
{
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr imu_sub_;

  public:
    EKFNode()
    : Node("ekf_node")
    {
      
      imu_sub_ = this->create_subscription<std_msgs::msg::String>("imu", 10, std::bind(&EKFNode::imu_callback, this, _1));

      RCLCPP_INFO(this->get_logger(), "EKFNode node has been started.");
    }

  private:

    void imu_callback(const std_msgs::msg::String & msg)
    {
      
      RCLCPP_INFO(this->get_logger(), "I heard '%s'", msg.data.c_str());

    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}

