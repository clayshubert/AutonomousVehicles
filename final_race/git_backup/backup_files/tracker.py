#!/usr/bin/python2
#

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from racecar_line_follower.msg import LineOffset, ImageArray

class LineTrackerNode():
    def __init__(self):
        self.offset_pub = rospy.Publisher('line_offset', LineOffset, queue_size=10)
        self.line_in_view_pub = rospy.Publisher('line_in_view', Bool, queue_size=10)
        self.debug_image_pub = rospy.Publisher('debug_image', Image, queue_size=10)
        
        self.bridge = CvBridge()
        self.rolling_average_max= 4
        self.prev_angles = []
        
        self.camera_offset = 0

        rospy.Subscriber('processed_images', ImageArray, self.image_callback)

    def image_callback(self, msg):
        orange = msg.data[0]
        green = msg.data[1]
        yellow = msg.data[2]
        black = msg.data[3]

        # convert ROS image message to opencv image
        # (processed image is already thresholded and cropped)
        try:
            thresholded = self.bridge.imgmsg_to_cv2(orange, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

        res = thresholded.shape
        debug_image = cv2.cvtColor(thresholded, cv2.COLOR_GRAY2BGR)

        # find contours in the image
        _, contours, _ = cv2.findContours(thresholded, 1, 2)
        
        if len(contours) == 0:
            self.line_in_view_pub.publish(False)
            return

        # sort the contours by area (because the line should be the largest)
        def greater(a, b):
            area1 = cv2.contourArea(a)
            area2 = cv2.contourArea(b)
            if area1 > area2:
                return -1
            return 1
        contours.sort(greater)
        
        # calculate percentage of image occupied by largest contour
        contour_area = cv2.contourArea(contours[0])
        image_area = thresholded.shape[0] * thresholded.shape[1]
        contour_size_in_image = contour_area / image_area
        
        # return if the contour isn't large enough
        if contour_size_in_image < 0.01:
            self.line_in_view_pub.publish(False)
            return
        
        front_contour = self.crop_contour_height(contours[0], res[0] * 0.4, res[0])
        if len(front_contour) > 1:
            M = cv2.moments(front_contour)
            front_contour_center = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        else:
            front_contour_center = -1, -1
        
        remaining_contour = self.crop_contour_height(contours[0], 0, res[0] * 0.66)
        if len(remaining_contour) > 1:
            M = cv2.moments(remaining_contour)
            remaining_contour_center = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        else:
            remaining_contour_center = -1, -1

        immediate_offset = -(front_contour_center[0] - (res[1] / 2)) + self.camera_offset
        future_offset = -(remaining_contour_center[0] - (res[1] / 2)) + self.camera_offset

        cv2.drawContours(debug_image, contours, 0, (0, 255, 0), 2)
        cv2.circle(debug_image, front_contour_center, 6, (0, 0, 255), -1)
        cv2.circle(debug_image, remaining_contour_center, 6, (0, 0, 255), -1)

        # publish results
        line_offset = LineOffset()
        line_offset.header.stamp = rospy.Time.now()
        line_offset.immediate = immediate_offset
        line_offset.future = future_offset
        self.offset_pub.publish(line_offset)
        self.line_in_view_pub.publish(True)

        try:
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, 'bgr8'))
        except CvBridgeError as e:
            print(e)
    
    def crop_contour_height(self, cnt, min_h, max_h):
        to_delete = []
        for i, p in enumerate(cnt):
            p = p[0]
            if p[1] > max_h or p[1] < min_h:
                to_delete.append(i)
        cnt = np.delete(cnt, to_delete, 0)
        return cnt

if __name__ == '__main__':
    rospy.init_node('line_tracker')
    node = LineTrackerNode()
    rospy.spin()


