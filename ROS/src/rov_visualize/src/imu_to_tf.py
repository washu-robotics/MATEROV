#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import Imu

def imu_callback(msg):
    br = tf.TransformBroadcaster()
    orientation = msg.orientation
    br.sendTransform((0, 0, 0),  # No translation for base_link
                     [orientation.x, orientation.y, orientation.z, orientation.w],
                     rospy.Time.now(),
                     "base_link",
                     "world")  # Replace 'world' with your fixed frame

if __name__ == '__main__':
    rospy.init_node('imu_to_tf')
    rospy.Subscriber("/sensors/imu", Imu, imu_callback)  # Replace 'imu_data' with your IMU topic
    rospy.spin()
