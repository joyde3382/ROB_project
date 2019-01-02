#!/usr/bin/env python
import cv2
import urllib
import numpy as np
import math
# pipes
import cPickle
import os
import json

import rospy
from std_msgs.msg import String

class computerVision:

    def get_from_webcam(self):
        """
        Fetches an image from the webcam
        """
        print "try fetch from webcam..."
        stream=urllib.urlopen('http://192.168.0.20/image/jpeg.cgi')
        bytes=''
        bytes+=stream.read(64500)
        a = bytes.find('\xff\xd8')
        b = bytes.find('\xff\xd9')

        if a != -1 and b != -1:
            jpg = bytes[a:b+2]
            bytes= bytes[b+2:]
            i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),1)
            
            return i
        else:
            print "did not receive image, try increasing the buffer size in line 13:"

    def get_from_file(self, filename):
        """
        Loads image from file
        """
        print "loading from file..."
        return cv2.imread(filename)

    def get_bricks(self, contours):
        """
        For each contour in contours
            approximate the contours such that small variations are removed
            calulate the area of the contour
            if the area is within the desired range we append the box points to the
            bricks.
        """
        bricks = []
        for cnt in contours:
            epsilon = 0.04*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,epsilon,True) 
            
            if len(approx) >= 4:
                rect = cv2.minAreaRect(approx)
                area = cv2.contourArea(approx)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                if area > 600 and area < 4000:

                    brick = Brick()
                    area = np.int0(area)
                    center = np.int0(rect[0])
                    angle = np.int0(rect[2])

                    brick.set_area(area)
                    brick.set_center(center)
                    brick.set_angle(angle)
                    brick.set_box(box)

                    bricks.append(brick)

            # elif len(approx) > 4:
            #     (x,y),radius = cv2.minEnclosingCircle(cnt)
            #     center = (int(x),int(y))
            #     radius = int(radius)
            #     area = radius*radius*math.pi

            #     if area > 600 and area < 2000:

            #         brick = Brick()
            #         area = np.int0(area)
                
            #         brick.set_area(area)
            #         brick.set_center(center)
            #         brick.set_radius(radius)

            #         bricks.append(brick)

        
                
        return bricks


    def extract_single_color_range(self, image,hsv,lower,upper):
        """
        Calculates a mask for which all pixels within the specified range is set to 1
        the ands this mask with the provided image such that color information is
        still present, but only for the specified range
        """
        mask = cv2.inRange(hsv, lower, upper)
        res = cv2.bitwise_and(image,image, mask= mask)

        res = cv2.medianBlur(res, 5)
        return res

    def threshold_image(self, image,debug=False):
        """
        Thresholds the image within the desired range and then dilates with a 3x3 matrix
        such that small holes are filled. Afterwards the 'blobs' are closed using a
        combination of dilate and erode
        """
        ret,th1 = cv2.threshold(image,150,255,cv2.THRESH_BINARY)
        if debug: cv2.imwrite('debug_pics/threshold_binary.jpg',th1) #cv2.imshow('th1',th1)
        resdi = cv2.dilate(th1,np.ones((7,7),np.uint8))
        if debug: cv2.imwrite('debug_pics/dilated.jpg',resdi) #cv2.imshow('dilated',resdi)
        closing = cv2.morphologyEx(resdi, cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
        if debug: cv2.imwrite('debug_pics/closing.jpg',closing) # cv2.imshow('closing',closing)

        return closing

    def contours(self, image,debug=False):
        """
        Extract the contours of the image by first converting it to grayscale and then
        call findContours
        """
        imgray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        if debug: cv2.imwrite('debug_pics/gray_scale_contour.jpg',imgray) # cv2.imshow('gray_scale_contour',imgray)
        im2, contours, hierarchy = cv2.findContours(imgray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        return contours,hierarchy


    def do_full(self, image,hsv,upper,lower,debug=False):
        """
        Main methods for processing an image and detect rectangles in the given
        hsv color range

        set debug to True in order to show the intermediate images
        """
        single_color_img = self.extract_single_color_range(image,hsv,lower,upper)
        if debug:
            # cv2.imshow('single_color_img',single_color_img)
            cv2.imwrite('debug_pics/single_color_img.jpg',single_color_img)
        single_channel = self.threshold_image(single_color_img,debug)
        if debug:
            # cv2.imshow('single_channel',single_channel)
            cv2.imwrite('debug_pics/single_channel.jpg',single_channel)
        cont,hierarchy = self.contours(single_channel,debug)

        if debug:
            for i,cnt in enumerate(cont):
                cv2.drawContours(single_channel,cont,i,(0,0,255),2)
        if debug: cv2.imwrite('debug_pics/contours.jpg',single_channel) #cv2.imshow('contours',single_channel)

        return self.get_bricks(cont)

    def show_bricks(self, image,bricks,color, colorName):

        for b in bricks:

            box = b.box
            cX = b.get_centerFromImage()[0] 
            cY = b.get_centerFromImage()[1]

            if len(box) == 0:
                cv2.circle(image,b.get_centerFromImage(),b.get_radius(),color,2)
            else:
                cv2.drawContours(image,[box],0,color,2)

            brickName = str(colorName) + str(bricks.index(b))

            cv2.putText(image, brickName, (cX, cY - 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (255, 255, 255), 2)
        

class Brick(object):
 
    centerFromImage = 0
    centerFromRobot = 0
    angle = 0
    radius = 0
    area = 0
    box = []

    def set_center(self, center=0.0):
        """Set the center coordinate for the box"""

        self.centerFromImage = center

        tempCenter = [0] * 2

        tempCenter[0] = center[0] - 305 # Xcoord offset
        tempCenter[1] = 313 - center[1] # Ycoord offset

        self.centerFromRobot = tempCenter

    def get_centerFromRobot(self):
        return self.centerFromRobot

    def get_centerFromImage(self):
        return self.centerFromImage

    def set_radius(self, radius = 0.0):
        self.radius = radius

    def get_radius(self):
        return self.radius

    def set_angle(self, angle=0.0):
        """Set the center coordinate for the box"""
        self.angle = angle

    def set_area(self, area=0.0):
        """Set the center coordinate for the box"""
        self.area = area

    def set_box(self, box=0.0):
        """Set the center coordinate for the box"""
        self.box = box

    def get_box(self):
        return self.box



vision = computerVision()

# image = vision.get_from_file('test5.jpg')
image = vision.get_from_webcam()

image = image[70:410, 25:635] # crop_img = img[y:y+h, x:x+w]

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# hsv hue sat value
# this one works quite fine
###################

lower_blue = np.array([110,125,125])
upper_blue = np.array([190,255,255])

lower_yellow = np.array([15,155,125])
upper_yellow = np.array([50,255,255])

# lower_blue = np.array([110,80,80])
# upper_blue = np.array([190,255,255])

# lower_yellow = np.array([15,100,50])
# upper_yellow = np.array([50,255,255])

# lower_red = np.array([0,75,50])
# upper_red = np.array([23,255,255])

####################

blue_bricks = vision.do_full(image,hsv,upper_blue,lower_blue, True)
yellow_bricks = vision.do_full(image,hsv,upper_yellow,lower_yellow)

vision.show_bricks(image,blue_bricks,(255,0,0), 'Blue')
vision.show_bricks(image,yellow_bricks,(0,255,255), 'Yellow')

bcenter = []
bangle = []
barea = []

for b in blue_bricks:

    center = b.get_centerFromRobot()
    angle = b.angle
    area = b.area
    
    bcenter.append(center)
    bangle.append(angle)
    barea.append(area)
    print 'Blue object(x,y) ' + ':' + str(center) + ' //// angle: ' + str(angle) + ' //// area: ' + str(area) 
    
ycenter = []
yangle = []
yarea = []

for b in yellow_bricks:

    center = b.get_centerFromRobot()
    angle = b.angle
    area = b.area
    
    ycenter.append(center)
    yangle.append(angle)
    yarea.append(area)

    print 'Yellow object ' + ':' +  str(center) + ' //// angle: ' + str(angle) + ' //// area: ' + str(area) 

x = {
"Blue": {
  "center": bcenter,
  "angle": bangle,
  "area": barea
  },
  "Yellow": {
  "center": ycenter,
  "angle": yangle,
  "area": yarea
  },
}

#http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29 is used 

y = json.dumps(x)

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    for x in range(10):
		hello_str = y
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

cv2.imshow('result',image)
cv2.imwrite('result.jpg',image)
# while True:
#     c = cv2.waitKey(5)
#     if c != -1:
#         cv2.destroyAllWindows()
#         exit(0)

