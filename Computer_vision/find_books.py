# import the necessary packages
import numpy as np
import cv2
import urllib

def get_from_webcam():
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

# load the image, convert it to grayscale, and blur it
image = cv2.imread("test5.jpg")
# image = get_from_webcam()
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# gray = cv2.GaussianBlur(gray, (1, 1), 0)
gray = cv2.bilateralFilter(gray, 11, 17, 17)
cv2.imshow("Gray", gray)
cv2.waitKey(0)

# detect edges in the image
edged = cv2.Canny(gray, 5, 255)
cv2.imshow("Edged", edged)
cv2.waitKey(0)

# construct and apply a closing kernel to 'close' gaps between 'white'
# pixels
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)
cv2.imshow("Closed", closed)
cv2.waitKey(0)

# find contours (i.e. the 'outlines') in the image and initialize the
# total number of books found
(_, cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
total = 0


for c in cnts:
    # approximate the contour
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    area = cv2.contourArea(approx)

    # if the approximated contour has four points, then assume that the
    # contour is a book -- a book is a rectangle and thus has four vertices
    if len(approx) == 4:

        if area > 300 and area < 5000:
            cv2.drawContours(image, [approx], -2, (0, 255, 0), 4)
            total += 1

# display the output
print "I found {0} books in that image".format(total)
cv2.imshow("Output", image)
cv2.waitKey(0)

#  if len(approx) == 4:
#             rect = cv2.minAreaRect(approx)
#             area = cv2.contourArea(approx)
#             box = cv2.boxPoints(rect)
#             box = np.int0(box)

#             if area > 600 and area < 5000:

#                 brick = Brick()
#                 area = np.int0(area)
#                 center = np.int0(rect[0])
#                 angle = np.int0(rect[2])

#                 brick.set_area(area)
#                 brick.set_center(center)
#                 brick.set_angle(angle)
#                 brick.set_box(box)

#                 bricks.append(brick)