#!/usr/bin/python2
#

import rospy
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class LineFollowerTeleopNode:
    def __init__(self):
        # subscribe to PID Node
        rospy.Subscriber('line_follower_drive_cmd', AckermannDriveStamped, self.drive_cmd_callback)
        # subscribe to node that says whether or not we should even be driving forward
        rospy.Subscriber('line_in_view', Bool, self.line_in_view_callback)

        # subscribe to joystick input to turn navigation on/off
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        # publish to ackermann_cmd_mux
        self.cmd_pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)
        
        # drive constants
        self.navigation_enabled = False
        
    def drive_cmd_callback(self, drive_cmd):
        if not self.navigation_enabled:
            drive_cmd.drive.speed = 0
        
        # publish turn angle to ackermann_cmd_mux
        self.cmd_pub.publish(drive_cmd)

    def joy_callback(self, msg):
        buttons = msg.buttons
        if buttons[2] == 1:
            self.navigation_enabled = True
        elif buttons[3] == 1:
            self.navigation_enabled = False

    def line_in_view_callback(self, msg):
        # if the line isn't in view, we should turn navigation off
        if not bool(msg):
            self.navigation_enabled = False

if __name__ == '__main__':
    rospy.init_node('line_follower_teleop')
    node = LineFollowerTeleopNode()
    rospy.spin()

