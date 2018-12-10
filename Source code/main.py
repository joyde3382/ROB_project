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

from InvRobot import *
from GetCoordinates import *

visionCoor = []

def main():
        
    Jobactive = True
    Job = False
    
    while Jobactive == True:
        xy = returnvisionCoor()
        print "XY: "
        print xy
        if len(xy) > 0:
            mirrorCube(xy)
            Job = True
        else:
            Jobactive = False
            if Job == True:
                RobotHandshake()
                print "Jobs done"
            else:
                print "There wasnt any job "                
if __name__ == "__main__":
    setUpVisonCoordinates()
    setupGrabberPressureSensor()
    top = invkin([0,0, 54.1])
    RobotDo(top,0,0)
    print 'Main started: '
    print '______________________'
    main()



