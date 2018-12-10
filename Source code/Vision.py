#!/usr/bin/env python

import rospy
from findBricks import *
from std_msgs.msg import String

def VisionPublisher():
    
    pub = rospy.Publisher('Coordinates', String, queue_size=10)
    rospy.init_node('VisionPublisher', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        try:
            bricks = find_brick_centers()
            
            if len(bricks) != 0:        
                coords_str = "%f,%f"%(bricks[0], bricks[1])
            else:
                coords_str = "%f,%f"%(0, 0)          
            pub.publish(coords_str)
            
            print("Test af coords_str")
            print(coords_str)
        except:
            print "Error"
        rate.sleep()

if __name__ == '__main__':
    try:
        VisionPublisher()
    except rospy.ROSInterruptException:
        pass