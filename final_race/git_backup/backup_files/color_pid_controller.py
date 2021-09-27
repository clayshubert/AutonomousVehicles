#!/usr/bin/python2
#

import rospy
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from racecar_line_follower.msg import LineOffset

class PidControllerNode():
    def __init__(self):
        rospy.Subscriber('line_offset', LineOffset, self.offset_callback)
        self.pub = rospy.Publisher('line_follower_drive_cmd', AckermannDriveStamped, queue_size=5)
        
        self.speed = 1
        self.steering_max = 0.32

        self.Kp = 0.002
        self.Ki = 0
        self.Kd = -0.01

        self.integrator = 0
        self.derivator = 0
        self.prev = 0

    def offset_callback(self, msg):
        e = msg.immediate
        self.integrator += e
        self.derivator = self.prev - e

        steering_angle = self.Kp * e + self.Ki * self.integrator + self.Kd * self.derivator

        saturation_value = self.steering_max
        if steering_angle > saturation_value:
            steering_angle = saturation_value
	elif steering_angle < -saturation_value:
            steering_angle = -saturation_value

        self.prev = e

        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle = steering_angle
        self.pub.publish(drive_cmd)

if __name__ == '__main__':
    rospy.init_node('pid_controller')
    node = PidControllerNode()
    rospy.spin()

