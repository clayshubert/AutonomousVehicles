#!/usr/bin/env python
#

import rospy
import numpy as np
from std_msgs.msg import Header
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import ErrorMsg
from ackermann_msgs.msg import AckermannDriveStamped 
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import sys
import cv2


class Control:
    def __init__(self):
        
        rospy.Subscriber("/scan", LaserScan, self.scanner_callback)
	rospy.Subscriber("ar_pose_marker",AlvarMarkers,self.arCallback)
        self.decision = 0	    
	self.arcodes = []
	self.has_seen = False
        self.ctrl = rospy.Publisher("control", Int32 , queue_size=10)
    
    def scanner_callback(self, msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = msg.ranges
        
        # define the distance returned by the laser data 90 degrees left and right
        Dist_L = ranges[int((len(ranges)/1.5)-(3.1415/2))] #used to be 2
        dist_R = ranges[int((len(ranges)/1.5)+(3.1415/2))]
        
        # if the laser scan returns values far enough away, activate line follower
        if Dist_L >4 and dist_R > 4:
            decision = 1
            
        # if no left wall but right wall then activate right wall follower
        if Dist_L >4 and dist_R < 4:
            decision = 2
            
        # if no right wall but left wall then activate left wall follower
        if Dist_L <4 and dist_R > 4:
            decision = 3

        if self.arcodes:
            for i in self.arcodes:
	        if i in [16,17,22]:
	            self.decision = 0
	        if i == 20:
	            self.decision = 1
                if i == 18:
                    self.decision = 2
                if i == 19:
                    self.decision = 3
                if i == 23:
                    self.decision = 4
	        if i == 21:
                    self.decision = 1
                    self.has_seen = True
            if self.has_seen == True and self.arcodes == []:
                self.decision = 0
	self.arcodes = []
        #print "decision", self.decision
        self.ctrl.publish(self.decision)

    def arCallback(self, msg):
        markers = msg.markers
        for marker in markers:
            if (marker.id in [16,17,18,19,20,21,22,23]):
                z = marker.pose.pose.position.z 
                if (z < 0.34) and (marker.id == 21):
		   # print("First!")
                    #print "Marker ID: ", marker.id
                    self.arcodes.append(marker.id)
                elif (z < 0.3) and (marker.id == 20):
		   # print("Second!")
                   rospy.sleep(1) 
                   self.arcodes.append(marker.id)
                    #rospy.sleep(.75)
                elif (z < 1.6) and (not marker.id in [20,21]): 
		   # print("Third!")
                    self.arcodes.append(marker.id)
	#print "decision:", self.decision



if __name__ == "__main__":
    rospy.init_node("master_control")
    
    node = Control()
    
    rospy.spin()  
