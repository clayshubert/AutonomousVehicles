#!/usr/bin/python2
#

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class DragRaceNavigationNode:
    def __init__(self):
        # subscribe to PID Node
        rospy.Subscriber("steering_angle", Float32, self.steering_angle_callback)
        
        # subscribe to joystick input to turn navigation on/off
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        # publish to ackermann_cmd_mux
        self.cmd_pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)
        
        # drive constants
        self.navigation_enabled = False
        self.speed = 10 # probably the limit
        
    def steering_angle_callback(self, turn_angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.steering_angle = turn_angle.data
        if self.navigation_enabled:
            msg.drive.speed = self.speed
        else:
            msg.drive.speed = 0

        # publish turn angle to ackermann_cmd_mux
        self.cmd_pub.publish(msg)

    def joy_callback(self, msg):
        buttons = msg.buttons
        if buttons[2] == 1:
            self.navigation_enabled = True
        elif buttons[3] == 1:
            self.navigation_enabled = False


if __name__ == '__main__':
    rospy.init_node('drag_race_navigation')
    node = DragRaceNavigationNode()
    rospy.spin()

