#!/usr/bin/env python

import cv2
import urllib
import numpy as np
import math

def get_from_webcam():
    print "try fetch from webcam..."
    stream=urllib.urlopen('http://192.168.0.20/image/jpeg.cgi')
    bytes=''
    bytes+=stream.read(64500)
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')

    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes= bytes[b+2:]
        i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
        i_crop = i[55:350, 300:610]
        return i_crop
    else:
        print "did not receive image, try increasing the buffer size in line 13:"

def extract_single_color_range(image,hsv,lower,upper):
    if len(lower) == 2 and len(upper) == 2:
        mask0 = cv2.inRange(hsv, lower[0], upper[0])
        mask1 = cv2.inRange(hsv, lower[1], upper[1])
        mask = mask0+mask1
    else:
        mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(image,image, mask= mask)
    return res

def threshold_image(image):
    ret,th1 = cv2.threshold(image,50,255,cv2.THRESH_BINARY)
    resdi = cv2.dilate(th1,np.ones((3,3),np.uint8))
    closing = cv2.morphologyEx(resdi, cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    return closing

def contours(image):
    imgray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(imgray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours

def get_centers(contours):
    centers = []
    for cnt in contours:
        epsilon = 0.1*cv2.arcLength(cnt,True)
        approx = cv2.approxPolyDP(cnt,epsilon,True)
        area = cv2.contourArea(approx)
        
        if area > 500:
            moments = cv2.moments(cnt)
            centers.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
    
    return centers

def find_brick_centers():    
    lower_blue = np.array([92,76,103])
    upper_blue = np.array([141,255,255])
    
    lower_green = np.array([36,76,0])
    upper_green = np.array([74,255,255])
    
    lower_yellow = np.array([21,76,103])
    upper_yellow = np.array([38,255,255])
    
    lower_red = np.array([np.array([0,76,103]),np.array([161,76,103])])
    upper_red = np.array([np.array([14,255,255]),np.array([179,255,255])])

    image = get_from_webcam()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    single_color_img_blue = extract_single_color_range(image,hsv,lower_blue,upper_blue)
    single_color_img_green = extract_single_color_range(image,hsv,lower_green,upper_green)
    single_color_img_yellow = extract_single_color_range(image,hsv,lower_yellow,upper_yellow)
    single_color_img_red = extract_single_color_range(image,hsv,lower_red,upper_red)
        
    single_channel_blue = threshold_image(single_color_img_blue)
    single_channel_green = threshold_image(single_color_img_green)
    single_channel_yellow = threshold_image(single_color_img_yellow)
    single_channel_red = threshold_image(single_color_img_red)
    
    cont_blue = contours(single_channel_blue)
    cont_green = contours(single_channel_green)
    cont_yellow = contours(single_channel_yellow)
    cont_red = contours(single_channel_red)
    
    centers_blue = get_centers(cont_blue)
    centers_green = get_centers(cont_green)
    centers_yellow = get_centers(cont_yellow)
    centers_red = get_centers(cont_red)
    
    centers_cm = []
    for c in centers_blue:
        x_koor = float((295.0-c[1])/9.0)
        y_koor = float(-(c[0])/9.0)
        centers_cm.append(x_koor)
        centers_cm.append(y_koor)
        
    for c in centers_green:
        x_koor = float((295.0-c[1])/9.0)
        y_koor = float(-(c[0])/9.0)
        centers_cm.append(x_koor)
        centers_cm.append(y_koor)
        
    for c in centers_yellow:
        x_koor = float((295.0-c[1])/9.0)
        y_koor = float(-(c[0])/9.0)
        centers_cm.append(x_koor)
        centers_cm.append(y_koor)
    
    for c in centers_red:
        x_koor = float((295.0-c[1])/9.0)
        y_koor = float(-(c[0])/9.0)
        centers_cm.append(x_koor)
        centers_cm.append(y_koor)
                          
    return centers_cm