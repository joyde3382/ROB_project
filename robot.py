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