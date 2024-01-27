#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def joy_callback(data):
    """
    Callback function to process joystick commands and publish Twist messages.
    :type data: Joy
    """

     # Debug: Print raw joystick data
    #rospy.loginfo("Axes: {}".format(data.axes))
    #rospy.loginfo("Buttons: {}".format(data.buttons))


    # Create a Twist message to publish
    twist = Twist()

    # Interpret joystick axes and buttons to control linear and angular velocities
    twist.linear.x = data.axes[1]  # Left stick up/down
    twist.angular.z = data.axes[3]  # Right stick left/right

    # Debug: Print interpreted velocities
    #rospy.loginfo("Linear Velocity: {}".format(twist.linear.x))
    #rospy.loginfo("Angular Velocity: {}".format(twist.angular.z))

    # If button 0 (A button on Xbox controller) is pressed, stop the robot
    #if data.buttons[0]:
        #twist.linear.x = 0.0
        #twist.angular.z = 0.0
        #rospy.loginfo("Stop Command Issued")

    # Publish the Twist message to cmd_vel topic
    cmd_vel_pub.publish(twist)

def joy_teleop_node():
    rospy.init_node('joy_teleop_node', anonymous=True)
    
    # Subscribe to the joy topic to receive joystick commands
    rospy.Subscriber("joy", Joy, joy_callback)

    # Publish Twist messages to cmd_vel topic
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/controllers/diff_drive_controller/cmd_vel', Twist, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    try:
        joy_teleop_node()
    except rospy.ROSInterruptException:
        pass