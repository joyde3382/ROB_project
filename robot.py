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
wfPath = "./p2"
try:
    os.mkfifo(wfPath)
    os.mkfifo(rfPath)
except OSError:
    pass
rp = open(rfPath, 'r')
response = rp.read()
#print "P2 hear %s" % response
rp.close()


#y = json.dumps(response)
y = json.loads(response)

# the result is a Python dictionary:
print(y)
print(y["Yellow"])

yellow = y["Yellow"]
Blue = y["Blue"]
Red = y["Red"]

print(Red["center"])
print(Red["area"])
print(Red["angle"])




def MainRobot():
    moreBlocks = True
    while moreBlocks == True:
        # for each coordinate
        pick_up([0,0,0,0])
        deliver([1,1,1,1])
        
        ## end
        moreBlocks = false
    

if __name__ == "__main__":
        rospy.init_node("robot")
        coords = [-132,229]
        x_koor = float(coords[0]/8.6076)
        y_koor = float(coords[1]/8.5714)
        pick_up([y_koor,-x_koor,0,0])
        deliver([20,0,4,0])
