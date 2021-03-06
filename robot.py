#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
import math
import sys
from std_msgs.msg import Float64
from move_robot import *
from std_msgs.msg import String
import cPickle
import os
import re
import json

#communicate with another process through named pipe
#one for receive command, the other for send command
#rfPath = "./Computer_vision/p1"

#rp = open(rfPath, 'r')
#response = rp.read()
#print "P2 hear %s" % response
#rp.close()

#y = json.dumps(response)
#y = json.loads(response)

# the result is a Python dictionary:

#yellow = y["Yellow"]
#Blue = y["Blue"]

#bcords = Blue["center"]
#ycords = yellow["center"]



yellow = []
Blue = []

bcords = []
ycords = []

def callback(data):
   #rospy.loginfo(data.data)
    #y = json.dumps(response)
   y = json.loads(data.data)
    # the result is a Python dictionary:
  #  print(y)
  #  print(y["Yellow"])
   yellow = y["Yellow"]
   Blue = y["Blue"]
   global bcords
   global ycords
   bcords = Blue["center"]
   ycords = yellow["center"]

    #http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29 is used 
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
#     rate = rospy.Rate(1) # 10hz
#     for x in range(10):
# 	rospy.Subscriber("chatter", String, callback)
# 	rate.sleep()




def MainRobot():

        rate = rospy.Rate(1) # 10hz
        for x in range(10):
                rospy.Subscriber("chatter", String, callback)
                rate.sleep()

        moreBlocks = True
        while moreBlocks == True:
                pickHeight = 7
                deliverHeight = 7
                xOffset = 8.564
                yOffset = 8.702
                # for each coordinate
                # if (bcords not)
                stackBlue = 3
                for x in bcords:
                        pick_up([float(x[1]/yOffset),-float(x[0]/xOffset),pickHeight,0])
                        deliver([8,20,stackBlue,0])
                        stackBlue =stackBlue + 4

                stackYellow = 3
                for x in ycords:
                        pick_up([float(x[1]/yOffset),-float(x[0]/xOffset),pickHeight,0])
                        deliver([8,-20,stackYellow,0])
                        stackYellow = stackYellow+4
                
                moreBlocks = False
    
if __name__ == "__main__":
        # listener()
        # print(bcords)
        rospy.init_node("robot")
        MainRobot()
        


