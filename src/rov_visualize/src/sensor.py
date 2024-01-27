#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import serial
import json

def read_serial_data(serial_connection):
    try:
        if serial_connection.in_waiting > 0:
            return serial_connection.readline().decode('utf-8').rstrip()
    except:
        rospy.logerr("Failed to read from serial port")
    return None

def main():
    rospy.init_node('json_to_twist', anonymous=True)
    twist_publisher = rospy.Publisher('/sensors/imu/ypr', Twist, queue_size=10)
    rate = rospy.Rate(50)  # 10 Hz

    serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
    baud_rate = rospy.get_param('~baud_rate', 9600)
    ser = serial.Serial(serial_port, baud_rate, timeout=1)

    while not rospy.is_shutdown():
        serial_data = read_serial_data(ser)
        rospy.loginfo(serial_data)
        if serial_data:
            try:
                data = json.loads(serial_data)
                
                yaw = data['yaw']
                pitch = data['pitch']
                roll = data['roll']

                # Create Twist message
                twist_msg = Twist()
                twist_msg.angular.x = roll
                twist_msg.angular.y = pitch
                twist_msg.angular.z = yaw

                twist_publisher.publish(twist_msg)

            except json.JSONDecodeError:
                rospy.logerr("Invalid JSON received.")
            except KeyError:
                rospy.logerr("JSON does not contain yaw, pitch, and roll.")

        rate.sleep()

    ser.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
