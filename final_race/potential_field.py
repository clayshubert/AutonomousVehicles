#!/usr/bin/env python
#

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped 
from std_msgs.msg import Int32


class PotentialField:
    def __init__(self):
        
        rospy.Subscriber("/scan", LaserScan, self.scanner_callback)
	rospy.Subscriber("/control", Int32, self.ctrlcallback)
        self.preve = 0
        self.message = -1
        self.deriv = 0
        self.cmd_pub=rospy.Publisher("/ackermann_cmd_mux/input/navigation",AckermannDriveStamped,queue_size=10 )
#	self.cmd_pub=rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=10)


    def ctrlcallback(self, msg):
        self.message = msg.data
        
    def scanner_callback(self, msg):
        if self.message in [0,2,3,4]:
            angle_min = msg.angle_min
            angle_inc = msg.angle_increment
            ranges = msg.ranges
            x_sum = 0
            y_sum = 0
            self.ks = 0.004
            for i in range(len(ranges)):
                angle = angle_min + angle_inc * i
                if (angle > -1.5 and angle < -0.05) or (angle > 0.05 and angle < 1.5):
		    distance = ranges[i]
		    if distance > 10:
		        distance = 10
                    x_sum = x_sum + (1/(distance*np.sin(angle)))
      
            	if angle > -1.35 and angle < 1.35:
		    distance = ranges[i]
		    if distance > 10:
		        distance = 10
            	    y_sum = y_sum + (1/(distance*np.cos(angle)+0.1))
            
            kp = -0.0015 #kp probably needs to be very small?
            if self.message == 2:
                #kd for the hairpin
                kd = -0.004
                kp = -0.0025
                self.ks = 0.003
            elif self.message == 4:
                #final stretch
                self.ks = 0.005
                kd = -0.009
            else:
                #regular conditions
                kd = -0.0110

            #tuning for the bridge
            if self.message == 3:
                kd = -0.0040
                kp = -0.00005
                self.ks = 0.0020
            
            self.deriv = x_sum - self.preve
            steering_angle = (kp * (x_sum)) + (kd*self.deriv)
          

            self.preve = x_sum
            self.message = -1
	    if steering_angle > 0.3:
		steering_angle = 0.3
	    if steering_angle < -0.3:
		steering_angle = -0.3
	

	
             
            speed = self.ks*y_sum
            #speed = 1
	    #MIN = (len(ranges)/2)-10
	    #MAX = (len(ranges)/2)+10

            #if speed > 3:
            #    speed = 3
	    #if speed < 0:
	    #    speed = .5

	    #if ranges[(len(ranges)/2)] < 0.3:
            #or ranges[(len(ranges)/2)+10] or ranges[(len(ranges)/2)-10] < 0.3:
	    #speed = -1.5
	    #steering_angle = -0.3
		

	


            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = speed
            drive_msg.drive.steering_angle = steering_angle
            self.cmd_pub.publish(drive_msg)
        



if __name__ == "__main__":
    rospy.init_node("potential_field")
    
    node = PotentialField()
    
    rospy.spin()
