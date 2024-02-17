#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cstring"
#include <serial/serial.h> 

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
      if(!my_serial.isOpen()) 
      {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
      }
      double linear_x = msg->linear.x;
      double linear_y = msg->linear.y;
      double linear_z = msg->linear.z;
      double angular_x = msg->angular.x;
      double angular_y = msg->angular.y;
      double angular_z = msg->angular.z;

      // RCLCPP_INFO(this->get_logger(), "I heard: '%f', '%f', '%f', '%f', '%f', '%f'", linear_x, linear_y, linear_z, angular_x, angular_y, angular_z);

      // TODO: send this data to the arduino
      // encode as json and send to the arduino through serial
      /* FORMAT:
        {
          'controls': {
            'linear' : {
              'x': linear_x,
              'y': linear_y,
              'z': linear_z
            },
            'angular' : {
              'x': angular_x,
              'y': angular_y,
              'z': angular_z
            }
          }
        }
      */
      string json = "{";
      json += "'controls': {";
      json += "'linear': {";
      json += "'x': " + to_string(linear_x) + ",";
      json += "'y': " + to_string(linear_y) + ",";
      json += "'z': " + to_string(linear_z);
      json += "},";
      json += "'angular': {";
      json += "'x': " + to_string(angular_x) + ",";
      json += "'y': " + to_string(angular_y) + ",";
      json += "'z': " + to_string(angular_z);
      json += "}";
      json += "}";
      json += "}";
      RCLCPP_INFO(this->get_logger(), "Sending to arduino: '%s'", json.c_str());

      // send to arduino using serial
      my_serial.write(json);

    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  serial::Serial my_serial("/dev/ttyUSB0", 57600, serial::Timeout::simpleTimeout(1000));
  // Adjust port and baud rate

  rclcpp::spin(std::make_shared<ThrusterNode>());
  rclcpp::shutdown();
  return 0;
}
