#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

def joy_callback(data):
    """
    Callback function to process joystick commands and publish Twist messages.
    :type data: Joy
    """

     # Debug: Print raw joystick data
    #rospy.loginfo("Axes: {}".format(data.axes))
    #rospy.loginfo("Buttons: {}".format(data.buttons))


    # Create a Twist message to publish
    cmd = TwistStamped()

    cmd.header.stamp = rospy.Time.now()

    # Interpret joystick axes and buttons to control linear and angular velocities
    cmd.twist.linear.y = data.axes[0] # Left stick left/right -> linear y
    cmd.twist.linear.x = data.axes[1]  # Left stick up/down -> linear x
    cmd.twist.angular.z = data.axes[3]  # Right stick left/right -> angular z
    cmd.twist.angular.y = data.axes[4]  # Right stick up/down -> angular y

    if data.buttons[4]:
        cmd.twist.linear.z = -1.0
    if data.buttons[5]:
        cmd.twist.linear.z = 1.0

    # Debug: Print interpreted velocities
    #rospy.loginfo("Linear Velocity: {}".format(twist.linear.x))
    #rospy.loginfo("Angular Velocity: {}".format(twist.angular.z))


    # Publish the Twist message to cmd_vel topic
    cmd_vel_pub.publish(cmd)

def joy_teleop_node():
    rospy.init_node('joy_teleop_node', anonymous=True)
    
    # Subscribe to the joy topic to receive joystick commands
    rospy.Subscriber("joy", Joy, joy_callback)

    # Publish Twist messages to cmd_vel topic
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/controllers/velocity_controller/cmd_vel', TwistStamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        joy_teleop_node()
    except rospy.ROSInterruptException:
        pass