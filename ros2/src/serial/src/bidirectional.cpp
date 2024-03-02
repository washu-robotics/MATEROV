#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cstring"
#include "string"

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

using namespace std;
using std::placeholders::_1;
using LibSerial::SerialPort;
using LibSerial::BaudRate;

class BidirectionalSerialNode : public rclcpp::Node
{
  SerialPort serial_port;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Time last_serial_read_time_;

  public:
    BidirectionalSerialNode()
    : Node("serial_node")
    {
      auto port_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      port_param_desc.description = "The port to open the serial port on";
      this->declare_parameter("port", "/dev/ttyACM0", port_param_desc);

      string port = this->get_parameter("port").as_string();
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("joy/cmd_vel", 10, std::bind(&BidirectionalSerialNode::cmd_vel_callback, this, _1));

      // Open the serial port
      try {
        serial_port.Open(port);
        RCUTILS_LOG_INFO("Serial port has been opened on %s", port.c_str());
      } catch (const LibSerial::OpenFailed &open_failed) {
        RCLCPP_FATAL(this->get_logger(), "The serial port did not open correctly.");
        serial_port.Close();
      }
      serial_port.SetBaudRate(BaudRate::BAUD_19200); // using high baud rate for faster round trip time
      last_serial_read_time_ = this->now();

      RCLCPP_INFO(this->get_logger(), "BidirectionalSerial node has been started.");
    }

    ~BidirectionalSerialNode()
    {
      serial_port.Close();
    }

  private:
    string serial_comm(string msg)
    {
      if (!serial_port.IsOpen()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return "";
      }

      serial_port.Write(msg);
      RCLCPP_INFO(this->get_logger(), "Sent to Arduino: '%s'", msg.c_str());

      string response;
      serial_port.ReadLine(response);

      rclcpp::Time current_time = this->now();
      RCLCPP_INFO(this->get_logger(), "Delay: %f", (current_time - last_serial_read_time_).seconds());
      last_serial_read_time_ = current_time;

      return response;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      double linear_x = msg->linear.x;
      double linear_y = msg->linear.y;
      double linear_z = msg->linear.z;
      double angular_x = msg->angular.x;
      double angular_y = msg->angular.y;
      double angular_z = msg->angular.z;

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

      string response = serial_comm(json);
      RCUTILS_LOG_INFO("Response: %s", response.c_str());

      // TODO: Parse response and publish to a topic
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BidirectionalSerialNode>());
  rclcpp::shutdown();
  return 0;
}

