#!/usr/bin/python
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from sensor_msgs.msg import Image
import rospy
import sys
import cv2
#import ErrorMsg
import numpy as np

class Controller:

    def __init__(self):
        self.velocity = .834
        self.navigationTopic =  "/ackermann_cmd_mux/input/navigation"
        self.maxSteeringAngle = .32 #rospy.get_param('/line_follower/max_steering_angle')
        #publishing to ackermann and subscribing to alvar whatever
        self.commandPub = rospy.Publisher(self.navigationTopic, AckermannDriveStamped, queue_size = 10)
	rospy.Subscriber("ar_pose_marker",AlvarMarkers,self.errorCallback)
	rospy.Subscriber("/testttttt",AlvarMarkers,self.errorCallback)
				
    def errorCallback(self, msg):
        steeringAngle, velocity = self.getSteeringAngle(msg)
        self.publishCommand(velocity, -steeringAngle)
		
    def getSteeringAngle(self, msg):
        
        markers = msg.markers
	print(markers)
        for marker in markers:
            if (marker.id in [1,2,3,4,5,6,7,8,9,10]):
                quaternion = marker.pose.pose.orientation
                quaternion = (quaternion.x,quaternion.y,quaternion.z,quaternion.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                point = marker.pose.pose.position
                x,y,z = point.x, point.y, point.z #z
                M_X = x #what I added
                Designated_Speed = ((2.82381*z)-.37) #2.52381
                if Designated_Speed < 0:
                    Designated_Speed = -.5
                if Designated_Speed > self.velocity:
                    Designated_Speed = self.velocity
         
                steeringAngle = 2.5*M_X #self.maxSteeringAngle
                if steeringAngle > self.maxSteeringAngle:
                    steeringAngle = self.maxSteeringAngle
                if steeringAngle < -0.32:
                    steeringAngle = -0.32
                return  steeringAngle, Designated_Speed
        
        steeringAngle = 0
        Designated_Speed = 0
            
        return  steeringAngle, Designated_Speed
		
	
    def publishCommand(self, velocity, angle):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "line_follower"

        drivingCommand = AckermannDriveStamped()

        drivingCommand.drive.speed = velocity
        drivingCommand.drive.steering_angle = angle
        print(drivingCommand)
        self.commandPub.publish(drivingCommand)


		

if __name__ == "__main__":
    rospy.init_node('conga_line')
    node = Controller()
    rospy.spin()
