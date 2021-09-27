#!/usr/bin/python
#

import rospy
from math import pi
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyControllerNode:
    def __init__(self):
        # subscribe to incomming Ackermann drive commands
        rospy.Subscriber("ackermann_cmd_input", AckermannDriveStamped,
                         self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.scanner_callback)

        # publisher for the safe Ackermann drive command
        self.cmd_pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=10)
        self.DRIVE = True
        self.DANGER_DISTANCE = 0.4 # meters
        self.ANGLE = 30 * pi / 180 # degrees to radians

    def ackermann_cmd_input_callback(self, msg):
        if self.DRIVE:
            self.cmd_pub.publish(msg)
        else:
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0
            stop_msg.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(stop_msg)

    def scanner_callback(self, msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = msg.ranges
        self.DRIVE = True
        for index in range(0, len(ranges)):
            angle = angle_min + angle_inc * index

            LOW = -self.ANGLE / 2
            HIGH = self.ANGLE / 2
            
            if angle > LOW and angle < HIGH:
                # we are looking in the area in front of us
                if ranges[index] < self.DANGER_DISTANCE:
                    self.DRIVE = True
                    #from easyros import move
                    #move(speed=-4, steering_angle=0, duration=1.75)
                    #self.ackermann_cmd_input_callback(AckermannDriveStamped())
                    break
                else:
                    self.DRIVE = True
                    

if __name__ == "__main__":
    rospy.init_node("safety_controller")
    node = SafetyControllerNode()
    rospy.spin()

