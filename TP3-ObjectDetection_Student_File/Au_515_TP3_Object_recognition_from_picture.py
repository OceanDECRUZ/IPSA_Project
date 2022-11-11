# -*- coding: utf-8 -*-
"""
Created on Fri Nov  4 15:44:15 2022

@author: Rayane
"""
import numpy as np
import argparse
import cv2

# Construct the argument parse

parser = argparse.ArgumentParser(description='Script to run MobileNet-SSD object detection network')

parser.add_argument("--i", help="path to image file. If empty, camera's stream will be used")

parser.add_argument("--prototxt", default="MobileNetSSD_deploy.prototxt", help='Path to text network file:' 'MobileNetSSD_deploy.prototxt for Caffe model or ')

parser.add_argument("--weights",default="MobileNetSSD_deploy.caffemodel",help='Path to weights: ''MobileNetSSD_deploy.caffemodel for Caffe model or ')

parser.add_argument("--thr", default=0.2, type=float, help="confidence threshold to filter out weak detections")

args = parser.parse_args()

#QUESTION 1

# We need to construct the parser argument to describe the type of arguments that we will give
# Parser stores the path to the video, the protocol to follow and the threshold to filter out the object with a weak
# precision parameter

# prototxt: is the protocol for exchanging data
# weitght :

# Labels of Network

classNames = { 0: 'background',
1: 'aeroplane', 2: 'bicycle', 3: 'bird',
4: 'boat', 5: 'bottle', 6: 'bus', 7: 'car',
8: 'cat', 9: 'chair', 10: 'cow',
11: 'diningtable', 12: 'dog', 13: 'horse',
14: 'motorbike', 15: 'person',
16: 'pottedplant', 17: 'sheep', 18: 'sofa',
19: 'train', 20: 'tvmonitor' }

COLORS = np.random.uniform(0, 255, size=(len(classNames), 3))

cap = cv2.imread('img.jpeg')
net = cv2.dnn.readNetFromCaffe(args.prototxt, args.weights)

# QUESTION 2

# In line 15 we capture the video moto_bike.mp4 
# In line 16 imports the caffe model args.prototxt and the weights 


frame = cap
frame_resized = cv2.resize(frame,(300,300))
blob = cv2.dnn.blobFromImage(frame_resized, 0.007843, (300, 300), (127.5, 127.5, 127.5), False)
net.setInput(blob)
detections = net.forward()
cols = frame_resized.shape[1]
rows = frame_resized.shape[0]

# QUESTION 3

# blob : stands for a “Binary Large Object,” a data type that stores binary data. Binary Large Objects (BLOBs) 
# can be complex files like images or videos, unlike other data strings that only store letters and numbers.

# QUESTION 4

#In line 22 we create the binary large object from the resized frame

for i in range(detections.shape[2]):
#Confidence of prediction
    confidence = detections[0, 0, i, 2]
    # Filter prediction
    if confidence > args.thr:
        # Class label
        class_id = int(detections[0, 0, i, 1])
        # Object location
        xLeftBottom = int(detections[0, 0,i, 3] * cols)
        yLeftBottom = int(detections[0, 0,i, 4] * rows)
        xRightTop = int(detections[0, 0,
        i, 5] * cols)
        yRightTop = int(detections[0, 0,i, 6] * rows)
        # Factor for scale to original size of
        # frame
        heightFactor = frame.shape[0]/300.0
        widthFactor = frame.shape[1]/300.0
        # Scale object detection to frame
        xLeftBottom = int(widthFactor *xLeftBottom)
        yLeftBottom = int(heightFactor *yLeftBottom)
        xRightTop = int(widthFactor *xRightTop)
        yRightTop = int(heightFactor *yRightTop)
        # Draw location of object
        cv2.rectangle(frame, (xLeftBottom,yLeftBottom), (xRightTop, yRightTop),(0, 255, 0))
        # Draw label and confidence of prediction
        # in frame resized
        if class_id in classNames:
            label = classNames[class_id] + ": " + str(confidence)
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            yLeftBottom = max(yLeftBottom,labelSize[1])
            cv2.rectangle(frame, (xLeftBottom,
            yLeftBottom - labelSize[1]),(xLeftBottom + labelSize[0], yLeftBottom+ baseLine), (255, 255, 255), cv2.FILLED)
            cv2.putText(frame, label, (xLeftBottom,yLeftBottom), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0))
            
            cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
cv2.imshow('frame',frame)
cv2.waitKey(0)
       