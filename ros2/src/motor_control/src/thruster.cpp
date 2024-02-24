#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cstring"
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

using namespace std;
using std::placeholders::_1;
using LibSerial::SerialPort;
using LibSerial::BaudRate;

class ThrusterNode : public rclcpp::Node
{
  SerialPort serial_port;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  public:
    ThrusterNode()
    : Node("thruster_node")
    {
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("joy/cmd_vel", 10, std::bind(&ThrusterNode::cmd_vel_callback, this, _1));
      serial_port.Open("/dev/ttyUSB0");
      serial_port.SetBaudRate(BaudRate::BAUD_9600); // may need to change this
    }

  private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      double linear_x = msg->linear.x;
      double linear_y = msg->linear.y;
      double linear_z = msg->linear.z;
      double angular_x = msg->angular.x;
      double angular_y = msg->angular.y;
      double angular_z = msg->angular.z;

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

      // send to arduino using serial
      if (serial_port.IsOpen()) {
        serial_port.Write(json);
        RCLCPP_INFO(this->get_logger(), "Sent to Arduino: '%s'", json.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
      }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Adjust port and baud rate
  rclcpp::spin(std::make_shared<ThrusterNode>());
  rclcpp::shutdown();
  return 0;
}
