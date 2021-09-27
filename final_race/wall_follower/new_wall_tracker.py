#!/usr/bin/python2
# from car 66 code

import rospy
import math
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class WallTrackerNode:
    def __init__(self):
        
        # subscribing to laser scan data
        rospy.Subscriber("/scan", LaserScan, self.scanner_callback)
      
        
        # subscribe to gamepad data to get button input
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        # publisher for distance
        self.distance_pub = rospy.Publisher("wall_distance", Float32, queue_size=10)
        # publisher for rotational error
        self.angle_pub = rospy.Publisher("rotational_error", Float32, queue_size=10)
        # publisher for smoother_rotational_error (rolling average values)
        self.smoothed_pub = rospy.Publisher("smoothed_rotational_error", Float32, queue_size=10)

        # flag for start of race configuration
        self.is_configured = False

        # wall to follow (configured on first call of scanner_callback)
        self.follow_right_wall = True

        # list to hold past error values (for rolling average)
        self.rolling_average_max = 4
        self.prev_angles = []
    
    def scanner_callback(self, msg):
        if False:#not self.is_configured:

            angle_min = msg.angle_min 
            angle_inc = msg.angle_increment 
            ranges = msg.ranges 
            shot_range = 150
            dist1 = ranges[shot_range] 
            dist2 = ranges[-shot_range]
            if dist1 > dist2:   
                self.follow_right_wall = True
            else:
                self.follow_right_wall = False
            self.is_configured = True

        # slice the array in half so that we only look at the wall we care about
        if self.follow_right_wall:
            ranges = msg.ranges[:len(msg.ranges) / 2]
            ranges = ranges[int(0.19 / msg.angle_increment):]
            ranges = ranges[::-1]
        else: # following the left wall
            ranges = msg.ranges[len(msg.ranges) / 2 : ]

        f_index = int((math.pi / 2) / msg.angle_increment)
	
	print(ranges)

        # retrieve the two sides of the right triangle
        fixed_distance = ranges[f_index]
        # gets the shortest distance side and its index (so we can calculate sign later)
        shortest_distance, s_index = min((val, idx) for (idx, val) in enumerate(ranges))

        # calculate the angle (rotational error)
        angle = math.acos(shortest_distance / fixed_distance)

        # adjust sign if necessary
        if self.follow_right_wall and s_index > f_index:
            angle *= -1
        elif not self.follow_right_wall and s_index < f_index:
            angle *= -1
        
        # calculate rolling average smoothed angle
        self.prev_angles.append(angle)
        if len(self.prev_angles) > self.rolling_average_max:
            self.prev_angles.pop(0)
        smoothed_angle = sum(self.prev_angles) / len(self.prev_angles)

        # publish
        self.angle_pub.publish(angle)
        self.smoothed_pub.publish(smoothed_angle)
        self.distance_pub.publish(shortest_distance)
    
    def joy_callback(self, msg):
        buttons = msg.buttons
        if buttons[0] == 1:
            self.follow_right_wall = False
        elif buttons[1] == 1:
            self.follow_right_wall = True


if __name__ == "__main__":
    rospy.init_node("new_wall_tracker")
    node = WallTrackerNode()
    rospy.spin()

