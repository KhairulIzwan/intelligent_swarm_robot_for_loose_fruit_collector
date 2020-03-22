#!/usr/bin/env python

#Title: Python Subscriber for Tank Navigation
#Author: Khairul Izwan Bin Kamsani - [23-01-2020]
#Description: Tank Navigation Subcriber Nodes (Python)

#remove or add the library/libraries for ROS
import rospy
import sys
import cv2
import imutils
import argparse

#remove or add the message type
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import RegionOfInterest

class TankFaceDetector:

	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node("face_detector_node", anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Create the Subsciber (image_raw)
		self.sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.callback_image)
		
		# Create the Subsciber (camera_info)
		self.sub = rospy.Subscriber("/cv_camera/camera_info", CameraInfo, self.callback_camerainfo)

		"""Add changes here!"""
		## Create the Publisher (roi)		
		#self.pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=10)

		# Path to input Haar cascade for face detection
		self.faceCascade = cv2.CascadeClassifier("/home/khairulizwan/catkin_ws/src/intelligent_swarm_robot_for_loose_fruit_collector/intelligent_robot_vision/vision_face_detection/library/haarcascade_frontalface_default.xml")
		
	def callback_camerainfo(self, data):
		# Get the image width and height
		self.W = data.width
		self.H = data.height
	
	def callback_image(self, data):
		# Convert ros --> opencv
		self.convert_ros_to_opencv_img(data)
		
		# Detect face
		self.track()

		"""Add changes here!"""
		# loop over the face bounding boxes and draw them
		for self.rect in self.rects:

			"""Add changes here!"""
			# Center of image
			self.center_W = self.W // 2
			self.center_H = self.H // 2
			self.center_x_roi = (self.fW // 2) + self.fX
			self.center_y_roi = (self.fH // 2) + self.fY
			self.distance_x_center = self.center_W - self.center_x_roi

			rospy.loginfo('[INFO] Distance from center: %d' % self.distance_x_center)

			if self.distance_x_center > 0:
				# Blue color in BGR 
				color_roi = (255, 0, 0)
			elif self.distance_x_center < 0:
				# Red color in BGR 
				color_roi = (0, 0, 255)
			else:
				# White color in BGR 
				color_roi = (255, 255, 255)

			cv2.rectangle(self.frameClone, (self.rect[0], self.rect[1]), (self.rect[2], self.rect[3]), color_roi, 2)

			#self.roi=RegionOfInterest()
			#self.roi.x_offset=self.rect[0]
			#self.roi.y_offset=self.rect[1]
			#self.roi.width=self.rect[2]
			#self.roi.height=self.rect[3]

			#self.pub.publish(self.roi)

			cv2.circle(self.frameClone, (self.center_x_roi, self.center_y_roi), 1, (0, 0, 255), 2) 

		# Start coordinate, here (0, 0) 
		# represents the top left corner of image 
		start_point = (self.center_W, 0) 
		  
		# End coordinate, here (250, 250) 
		# represents the bottom right corner of image 
		end_point = (self.center_W, self.H) 
		  
		# Green color in BGR 
		color = (0, 255, 0) 
		  
		# Line thickness of 9 px 
		thickness = 2
		  
		# Using cv2.line() method 
		# Draw a diagonal green line with thickness of 9 px 
		cv2.line(self.frameClone, start_point, end_point, color, thickness) 

		cv2.imshow("Face Detector", self.frameClone)
		cv2.waitKey(1)
	
	def convert_ros_to_opencv_img(self, ros_image):
		self.cv_image = self.bridge.imgmsg_to_cv2(ros_image)
		
		# Clone the original image for displaying purpose later
		self.frameClone = self.cv_image.copy()

	def track(self):
		# Create an empty arrays for save rects value later
		self.rects = []
		
		"""Add changes here!"""
		# Detect all faces in the input frame
		self.faceRects = self.faceCascade.detectMultiScale(self.cv_image,
			scaleFactor = 1.1, minNeighbors = 5, minSize = (30, 30),
			flags = cv2.CASCADE_SCALE_IMAGE)

		"""Add changes here!"""
		# Loop over the face bounding boxes
		for (self.fX, self.fY, self.fW, self.fH) in self.faceRects:
			# Extract the face ROI and update the list of bounding boxes
			self.faceROI = self.cv_image[self.fY:self.fY + self.fH, self.fX:self.fX + self.fW]
			self.rects.append((self.fX, self.fY, self.fX + self.fW, self.fY + self.fH))

	def shutdown(self):
		try:
			rospy.loginfo("[INFO] Tank Face Detector [OFFLINE]")
		finally:
			cv2.destroyAllWindows()

def main(args):
	tfd = TankFaceDetector()
	try:
		rospy.spin()
	except ROSInterruptException:
		rospy.loginfo("[INFO] Tank Face Detector [OFFLINE]")

	cv2.destroyAllWindows()

if __name__ == "__main__":
	rospy.loginfo("[INFO] Tank Face Detector [ONLINE]")
	main(sys.argv)
