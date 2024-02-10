#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial
import struct

def twist_callback(data):
    # Create data packet by packing linear and angular velocities
    twist_data = struct.pack('<ff', data.linear.x, data.angular.z)
    # Send the packet through the serial port
    serial_port.write(twist_data)

if __name__ == '__main__':
    rospy.init_node('drive_node', anonymous=True)

    # Get serial port and baud rate from ROS parameters
    serial_port_name = rospy.get_param('~serial_port', '/dev/ttyACM0')
    baud_rate = rospy.get_param('~baud_rate', 9600)

    try:
        # Open the serial port
        serial_port = serial.Serial(port=serial_port_name, baudrate=baud_rate)
        rospy.loginfo(f'Serial port {serial_port_name} opened successfully at {baud_rate} baud.')

        # Subscribe to the cmd_vel topic
        rospy.Subscriber('/cmd_vel', Twist, twist_callback, queue_size = 15)

        rospy.spin()  # Keep the node running

    except serial.SerialException as e:
        rospy.logerr(f'Error opening serial port: {e}')
