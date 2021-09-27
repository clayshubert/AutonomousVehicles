#!/usr/bin/python2
#

import rospy
from std_msgs.msg import Float32

#print "pid controller"
class PidControllerNode():
    def __init__(self):
        rospy.Subscriber('smoothed_rotational_error', Float32, self.rotational_error_callback)
        self.pub = rospy.Publisher('steering_angle', Float32, queue_size=5)

        self.Kp = 0.2
        self.Ki = 0
        self.Kd = -0.03

        self.integrator = 0
        self.derivator = 0
        self.prev = 0

    def rotational_error_callback(self, msg):
        e = msg.data
        self.integrator += e
        self.derivator = self.prev - e

        steering_angle = self.Kp * e + self.Ki * self.integrator * e + self.Kd * self.derivator * e
        #print "e:", e
	print "steering_angle:", steering_angle

        #saturation_value = 0.3
	saturation_value = 0.6
        if steering_angle > saturation_value:
            steering_angle = saturation_value
	elif steering_angle < -saturation_value:
            steering_angle = -saturation_value
        
        self.prev = e
        self.pub.publish(steering_angle)
        #print steering_angle

if __name__ == '__main__':
    rospy.init_node('pid_wall_follower')
    node = PidControllerNode()
    rospy.spin()

