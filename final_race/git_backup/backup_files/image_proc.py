#!/usr/bin/python2
#

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from racecar_line_follower.msg import ImageArray

class ImageProcNode:
    def __init__(self):
        self.image_pub = rospy.Publisher('processed_images', ImageArray, queue_size=10)
        self.bridge = CvBridge()

        # ORANGE
        self.lower_orange_threshold = np.array([11, 80, 200])
        self.upper_orange_threshold = np.array([35, 255, 255])
        # GREEN
        self.lower_green_threshold = np.array([50, 50, 24])
        self.upper_green_threshold = np.array([65, 180, 80])
        # YELLOW
        self.lower_yellow_threshold = np.array([0, 0, 0])
        self.upper_yellow_threshold = np.array([0, 0, 0])
        # BLACK
        self.lower_black_threshold = np.array([0, 0, 0])
        self.upper_black_threshold = np.array([0, 0, 0])

        rospy.Subscriber('/zed/rgb/image_rect_color', Image, self.image_callback)

    def image_callback(self, msg):
        # convert ROS image message to opencv image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

        # convert to hsv colorspace to make thresholding for colors easier
        hsl = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        
        # crop
        hsl = hsl[-int(hsl.shape[0]/2.2):, :]
        
        # threshold to see only orange line
        mask_orange = cv2.inRange(hsl, self.lower_orange_threshold, self.upper_orange_threshold)
        # threshold to see only green line
        mask_green = cv2.inRange(hsl, self.lower_green_threshold, self.upper_green_threshold)
        # threshold to see only yellow line
        mask_yellow = cv2.inRange(hsl, self.lower_yellow_threshold, self.upper_yellow_threshold)
        # threshold to see only black line
        mask_black = cv2.inRange(hsl, self.lower_black_threshold, self.upper_black_threshold)

        # erode/dilate (removes noise smaller than kernel size)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel)

        images = ImageArray()
        images.header.stamp = rospy.Time.now()
        # republish processed image
        try:
            images.data.append(self.bridge.cv2_to_imgmsg(mask_orange, 'passthrough'))
            images.data.append(self.bridge.cv2_to_imgmsg(mask_green, 'passthrough'))
            images.data.append(self.bridge.cv2_to_imgmsg(mask_yellow, 'passthrough'))
            images.data.append(self.bridge.cv2_to_imgmsg(mask_black, 'passthrough'))
        except CvBridgeError as e:
            print(e)

        self.image_pub.publish(images)



if __name__ == '__main__':
    rospy.init_node('image_proc')
    node = ImageProcNode()
    rospy.spin()

