#!/usr/bin/env python3

# Title: Segmentation of Oil Palm Loose Fruit using Colorspace Technique
# Description: Using different type of colorspace -- RGB and HSV -- to segmenting the loose fruit -- masking
# Author: Universiti Putra Malaysia (FYP)

# import useful library
import numpy as np
import argparse
import cv2

import imutils
from skimage import measure
from imutils import contours

import os

count = 0

# parsing the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
ap.add_argument("-r", "--raw", required=True, help="Path to save the image")
# ap.add_argument("-r", "--radius", type=int, required=True, help="Radius of Gaussian blur; must be odd")
args = vars(ap.parse_args())

# STEP 1: ================== Load an Image ==================
# read an image
image = cv2.imread(args["image"])

# STEP 1a: ================== Copy Original ==================
image_orig = image.copy()

# STEP 2: ================== ColorSpace ==================
# convert image to hsv
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# STEP 3: ================== Masking ==================
# Range for lower red
lower_red = np.array([0,120,70])
upper_red = np.array([10,255,255])

# mask1 = cv2.inRange(blurred, lower_red, upper_red)
mask1 = cv2.inRange(hsv, lower_red, upper_red)

# Range for upper range
lower_red = np.array([170,120,70])
upper_red = np.array([180,255,255])

# mask2 = cv2.inRange(blurred,lower_red, upper_red)
mask2 = cv2.inRange(hsv,lower_red, upper_red)

# Generating the final mask to detect red color
mask = mask1 + mask2
cv2.imshow("final mask", mask)

mask = cv2.erode(mask, None, iterations=1)
mask = cv2.dilate(mask, None, iterations=1)
cv2.imshow("after filtering",mask)

#Segmenting the cloth out of the frame using bitwise and with the inverted mask
res1 = cv2.bitwise_and(image, image, mask=mask)
cv2.imshow("res",res1)

# STEP 4: ================== Counting and Labelling ==================
# perform labelling
labels = measure.label(mask, neighbors=8, background=0)
mask = np.zeros(mask.shape, dtype="uint8")

for label in np.unique(labels):
	# if this is the background label, ignore it
	if label == 0:
		continue
	# otherwise, construct the label mask and count the
	# number of pixels
	labelMask = np.zeros(mask.shape, dtype="uint8")
	labelMask[labels == label] = 255
	numPixels = cv2.countNonZero(labelMask)

	# if the number of pixels in the component is sufficiently

	# large, then add it to our mask of "large blobs"
	if numPixels > 185:
		mask = cv2.add(mask, labelMask)
    # find the contours in the mask, then sort them from left to
# right
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)
cnts = contours.sort_contours(cnts)[0]

print("I count {} fruits (maybe not!) in this image".format(len(cnts)))

# STEP 5: ================== Display ==================
# loop over the contours
for (i, c) in enumerate(cnts):
    # draw the bright spot on the image
    (x, y, w, h) = cv2.boundingRect(c)
    ((cX, cY), radius) = cv2.minEnclosingCircle(c)

    print(x, y, w, h, image.shape[1], image.shape[0])

    fruit = image_orig[y:y + h, x:x + w]
    fruit = imutils.resize(fruit, width=50, inter=3)

    cv2.imshow("Fruit", fruit)

    dirPath = os.path.sep.join([args["raw"]])
    if not os.path.exists(dirPath):
        os.makedirs(dirPath)

    p = os.path.sep.join([dirPath, "{}.png".format(str(count).zfill(6))])
    cv2.imwrite(p, fruit)

    count+=1

    cv2.circle(image, (int(cX), int(cY)), int(radius), (255, 255, 255), 2)
    cv2.putText(image, "#{}".format(i + 1), (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2)

    cv2.imshow("HSV Masking", np.hstack([image, res1]))
    cv2.waitKey(0)
