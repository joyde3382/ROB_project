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

import cPickle
import os
import re
import json

#communicate with another process through named pipe
#one for receive command, the other for send command
rfPath = "./Computer_vision/p1"

rp = open(rfPath, 'r')
response = rp.read()
#print "P2 hear %s" % response
rp.close()

#y = json.dumps(response)
y = json.loads(response)

# the result is a Python dictionary:

yellow = y["Yellow"]
Blue = y["Blue"]

bcords = Blue["center"]
ycords = yellow["center"]

def MainRobot():
    moreBlocks = True
    while moreBlocks == True:
        pickHeight = 7
        deliverHeight = 4
        xOffset = 8.564
        yOffset = 8.702
        # for each coordinate
        # if (bcords not)
        stackBlue = 3
        for x in bcords:
                pick_up([float(x[1]/yOffset),-float(x[0]/xOffset),pickHeight,0])
                deliver([12,18,deliverHeight,0])
                deliver([20,0,stackBlue,0])
                stackBlue =stackBlue + 4

        stackYellow = 3
        for x in ycords:
                pick_up([float(x[1]/yOffset),-float(x[0]/xOffset),pickHeight,0])
                deliver([12,-18,deliverHeight,0])
                deliver([20,0,stackYellow,0])
                stackYellow = stackYellow+4
        
        moreBlocks = false
    
if __name__ == "__main__":
        rospy.init_node("robot")
        MainRobot()
        


