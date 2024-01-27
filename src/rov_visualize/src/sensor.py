#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
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
    rospy.init_node('imu', anonymous=True)
    twist_publisher = rospy.Publisher('/sensors/imu', Imu, queue_size=10)
    rate = rospy.Rate(50)  # 10 Hz

    serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
    baud_rate = rospy.get_param('~baud_rate', 9600)
    ser = serial.Serial(serial_port, baud_rate, timeout=1)

    while not rospy.is_shutdown():
        serial_data = read_serial_data(ser)
        '''
        data example: {"orientation":{"w":0.99,"x":-0.00,"y":-0.00,"z":0.11},
                       "angular_velocity":{"x":0.00,"y":0.00,"z":-0.00},
                       "linear_acceleration":{"x":0.04,"y":0.13,"z":9.94}}
        '''
        rospy.loginfo(serial_data)
        if serial_data:
            try:
                data = json.loads(serial_data)
                
                # create a imu message
                imu = Imu()
                imu.header.stamp = rospy.Time.now()
                imu.header.frame_id = 'imu_link'
                imu.orientation.w = data['orientation']['w']
                imu.orientation.x = data['orientation']['x']
                imu.orientation.y = data['orientation']['y']
                imu.orientation.z = data['orientation']['z']
                imu.angular_velocity.x = data['angular_velocity']['x']
                imu.angular_velocity.y = data['angular_velocity']['y']
                imu.angular_velocity.z = data['angular_velocity']['z']
                imu.linear_acceleration.x = data['linear_acceleration']['x']
                imu.linear_acceleration.y = data['linear_acceleration']['y']
                imu.linear_acceleration.z = data['linear_acceleration']['z']
                twist_publisher.publish(imu)


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
