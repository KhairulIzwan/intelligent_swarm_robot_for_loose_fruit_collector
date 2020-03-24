#!/usr/bin/env python
#fruit tracking for loose fruit by detect the fruit and give out the position(x,y)
#servo move according to the position.

from __future__ import print_function
import roslib
# roslib.load_manifest('my_package')

import sys
import rospy
import cv2
import numpy as np
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

class tracking_movement_node:
    def __init__(self):
        # Initializing your ROS Node
        # rospy.init_node('my_node_name', anonymous=True)
        # or
        # rospy.init_node('my_node_name')
        rospy.init_node('tracking_movement_node')

        # Give the OpenCV display window a name
        self.cv_window_name_0 = "OpenCV Image"
        self.cv_window_name_1 = "OpenCV Image Mask"

        # define the lower and upper boundaries of the "green" ball in the HSV color space, then 	 initialize the list of tracked points
        self.greenLower = (0, 26, 135)
        self.greenUpper = (179, 255, 255)

        # initialize the list of tracked points, the frame counter, and the coordinate deltas
        self.pts = deque(maxlen=32)
        self.counter = 0
        (self.dX, self.dY) = (0, 0)
        self.direction = ""

        # Create the window and make it re-sizeable (second parameter = 0)
        # cv2.namedWindow(self.cv_window_name, cv2.WINDOW_NORMAL)

        # rospy.Publisher initialization
        # pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
        # The only required arguments to create a rospy.Publisher are the topic name, the Message 		class, and the queue_size
        self.roi_pub = rospy.Publisher("roi", RegionOfInterest, queue_size=10)

        # Give the camera driver a moment to come up
        rospy.sleep(1)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the raw camera image topic
        # subscribe to a topic using rospy.Subscriber class
        # sub=rospy.Subscriber('TOPIC_NAME', TOPIC_MESSAGE_TYPE, name_callback)
        self.sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.callback)
        self.image_sub = rospy.Subscriber("/camera0/cam0/image_raw", Image, self.callback)

    def callback(self, data):
        # Convert the raw image to OpenCV format using the convert_image() helper function
        self.convert_image(data)

        # Apply image processing using do_preprocess() helper function
        self.do_preprocess()

        # Apply ball tracking using do_ball_tracking() helper function
        # x, y, radius, center = self.do_tracking(cv_mask)
        self.do_detect()

        self.do_trackPts()

        # Refresh the displayed image
        cv2.imshow(self.cv_window_name_0,np.hstack([self.cv_image]))
        cv2.imshow(self.cv_window_name_1, self.cv_mask)
        cv2.waitKey(1)

    def convert_image(self, ros_image):
        # coverting the ROS image data to uint8 OpenCV format
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)

    def do_preprocess(self):
        try:
            # resize the frame, blur it, and convert it to the HSV color space
            # frame = imutils.resize(cv_image, width=600)
            blurred = cv2.GaussianBlur(self.cv_image, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # construct a mask for the color "green", then perform a series of dilations and erosions 		    to remove any small blobs left in the mask
            self.cv_mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
            self.cv_mask = cv2.erode(self.cv_mask, None, iterations=2)
            self.cv_mask = cv2.dilate(self.cv_mask, None, iterations=2)
        except CvBridgeError as e:
            print (e)

    def do_detect(self):
        # find contours in the mask and initialize the current (x, y) center of the ball
        cnts = cv2.findContours(self.cv_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        try:
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use it to compute the minimum enclosing 			circle and centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # Straight Bounding Rectangle
                xBox, yBox, w, h = cv2.boundingRect(c)

                # RegionOfInterest
                roi = RegionOfInterest()
                roi.x_offset = int(M["m10"] / M["m00"])
                roi.y_offset = int(M["m01"] / M["m00"])
                roi.width = w
                roi.height = h

                self.roi_pub.publish(roi)

                # only proceed if the radius meets a minimum size
                if radius > 5:
                    # draw the circle and centroid on the frame, then update the list of tracked points
                    cv2.circle(self.cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(self.cv_image, center, 5, (0, 0, 255), -1)

                    cv2.rectangle(self.cv_image, (xBox, yBox), (xBox + w, yBox + h), (0, 255, 0), 2)

                    # update the points queue
                    self.pts.appendleft(center)
        except CvBridgeError as e:
            print (e)

    def do_trackPts(self):
        try:
            # loop over the set of tracked points
            for i in range(1, len(self.pts)):
                # if either of the tracked points are None, ignore them
                if self.pts[i - 1] is None or self.pts[i] is None:
                    continue

                # check to see if enough points have been accumulated in the buffer
                if self.counter >= 10 and i == 1 and self.pts[-10] is not None:
                    # compute the difference between the x and y coordinates and re-initialize the 			    direction text variables
                    self.dX = self.pts[-10][0] - self.pts[i][0]
                    self.dY = self.pts[-10][1] - self.pts[i][1]
                    (self.dirX, self.dirY) = ("", "")

                    # ensure there is significant movement in the x-direction
                    if np.abs(self.dX) > 20:
                        self.dirX = "East" if np.sign(self.dX) == 1 else "West"

                    # ensure there is significant movement in the y-direction
                    if np.abs(self.dY) > 20:
                        self.dirY = "North" if np.sign(self.dY) == 1 else "South"

                    # handle when both directions are non-empty
                    if self.dirX != "" and self.dirY != "":
                        self.direction = "{}-{}".format(self.dirY, self.dirX)

                    # otherwise, only one direction is non-empty
                    else:
                        self.direction = self.dirX if self.dirX != "" else self.dirY

                # otherwise, compute the thickness of the line and draw the connecting lines
                thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
                cv2.line(self.cv_image, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

	    # show the movement deltas and the direction of movement on the frame
            cv2.putText(self.cv_image, self.direction, (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 3)
            cv2.putText(self.cv_image, "dx: {}, dy: {}".format(self.dX, self.dY), (10, 		        self.cv_image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
            # cv2.putText(self.cv_image, "counter: {}".format(self.counter), (10, self.cv_image.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

            self.counter += 1

        except CvBridgeError as e:
            print (e)


def main(args):
    vn = tracking_movement_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
