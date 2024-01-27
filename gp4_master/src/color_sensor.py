#!/usr/bin/env python3

import rospy
import std_msgs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge=CvBridge()
image_received = 0
cv_image = 0
color_show=""

def camera_callback(msg):
    ## Recieve the information and allows the publisher part of the code to run
    image_received=1
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    
    ##Resize of the image to make a smoother image processing
    image = cv2.resize(cv_image,(300, 300))
    
    ##Change from RGB to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    ##Ranges for color detection in HSV values
    min_green = np.array([70,50,150])
    max_green = np.array([90,255,255])

    min_red = np.array([0,100,142])
    max_red = np.array([10,255,255])

    min_orange = np.array([20,120,150])
    max_orange = np.array([45,255,255])

    ##Contrast images for each detected color
    mask_g = cv2.inRange(hsv, min_green, max_green)
    mask_r = cv2.inRange(hsv, min_red, max_red)
    mask_o = cv2.inRange(hsv, min_orange, max_orange)


    res_g = cv2.bitwise_and(image, image, mask=mask_g)
    res_r = cv2.bitwise_and(image, image, mask=mask_r)
    res_o = cv2.bitwise_and(image, image, mask=mask_o)

    ##Allows the code to try to find contours of color to save a variable with the color detected
    try:
        (gr,cont_g,hier_g)=cv2.findContours(res_g,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(cont_g)>0:
            color_show="green"

    except:
        rospy.sleep(1)
    
    try:
        (rd,cont_r,hier_r)=cv2.findContours(res_r,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(cont_r)>0:
            color_show="red"
    except:
        rospy.sleep(1)
    
    try:
        (org,cont_o,hier_o)=cv2.findContours(res_o,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(cont_o)>0:
           color_show="orange"
    except:
        rospy.sleep(1)
    
    #cv2.imshow('green',res_g)
    #cv2.imshow('orange',res_o)
    #cv2.imshow('red',res_r) 
    #cv2.imshow('original',image) 
    #cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('image_process')
    image_sub = rospy.Subscriber("/camera1/color/image_raw", Image, camera_callback)
    color_send = rospy.Publisher("/current_color", String, queue_size=1)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if image_received:
            print=(color_show)
            color_send.publish(color_show)
            cv2.waitKey(1)
            r.sleep()
        cv2.destroyAllWindows()
