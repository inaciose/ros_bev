#!/usr/bin/env python3

#
#
# based on: https://stackoverflow.com/questions/57439977/transforming-perspective-view-to-a-top-view
# issue: how to select the 4 source points coordinates and the the corresponent destination coordinates?
# v1.0.0 by inaciose
#

# Imports
import cv2
import rospy
from sensor_msgs.msg._Image import Image
from cv_bridge.core import CvBridge

"""
def find_countours(img):
    #filters image bilaterally and displays it
    bilatImg = cv2.bilateralFilter(img, 5, 175, 175)

    #finds edges of bilaterally filtered image and displays it
    edgeImg = cv2.Canny(bilatImg, 75, 200)

    #gets contours (outlines) for shapes and sorts from largest area to smallest area
    contours, hierarchy = cv2.findContours(edgeImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    # drawing red contours on the image
    for con in contours:
        cv2.drawContours(img, con, -1, (0, 0, 255), 3)

    # and double-checking the outcome
    cv2.imshow("Contours check",img)
    cv2.waitKey()
    cv2.destroyWindow("Contours check")
"""


import numpy as np

# Callback function to receive image
def message_RGB_ReceivedCallback(message):
    
    # get image from image
    img_rbg = bridge.imgmsg_to_cv2(message, "bgr8")

    # apply transform
    img_ipm = cv2.warpPerspective(img_rbg, ipm_matrix, img_rbg.shape[:2][::-1])

    #find_countours(img_ipm)

    # debug
    cv2.imshow('img', img_rbg)
    cv2.imshow('ipm', img_ipm)
    cv2.waitKey(1)

    # Change the cv2 image to imgmsg and publish
    msg_frame = bridge.cv2_to_imgmsg(img_ipm, "bgr8") 
    imagePub.publish(msg_frame)

def main():
    # Global variables
    global bridge
    global imagePub
    global pts
    global ipm_matrix

    # Init Node
    rospy.init_node('image_crop', anonymous=False)

    image_raw_topic = rospy.get_param('~image_raw_topic', '/ackermann_vehicle/camera/rgb/image_raw')
    image_bev_topic = rospy.get_param('~image_bev_topic', '/bev')
   
    rate_hz = rospy.get_param('~rate', 30)

    # Subscribe and pubblish topics
    rospy.Subscriber(image_raw_topic, Image, message_RGB_ReceivedCallback)
    imagePub = rospy.Publisher(image_bev_topic, Image, queue_size=2)

    # Create an object of the CvBridge class
    bridge = CvBridge()

    # ipm points & matrix
    pts = np.array([[196, 217], [441, 217], [517, 423], [120, 423]], dtype=np.float32)
    ipm_pts = np.array([[212,202], [427,202], [427,417], [212,417]], dtype=np.float32)

    ipm_matrix = cv2.getPerspectiveTransform(pts, ipm_pts)

    # set loop rate 
    rate = rospy.Rate(rate_hz)

    while  True:    
        rate.sleep()

    rospy.loginfo('Done. exit now!')    

if __name__ == '__main__':
    main()
