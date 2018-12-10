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
import time
from std_msgs.msg import String

from findBricks import *

visionCoor = []

def getCoordinates(coord):    
    global visionCoor
    
    coord_str = str(coord)
    coor = coord_str.replace("data: ","")
    visionCoor = [float(s) for s in coor.split(',')]
    
    if visionCoor[0] == 0 and visionCoor[1] == 0:
        visionCoor = []

# Subcribing on Coordinates node
def setUpVisonCoordinates():
    rospy.init_node("InvRobot")
    rospy.Subscriber("Coordinates", String, getCoordinates)

# Return the vision coordinates
def returnvisionCoor():
    return visionCoor
