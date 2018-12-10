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
        robotic_arm_standard_position();
        open_gripper();
        move_robotic_arm([0,1,0,0])
        close_gripper();
        robotic_arm_standard_position();
        open_gripper();
    

if __name__ == "__main__":
    print "Initiating Robo-Cop"
    rospy.init_node("robot")
    # MainRobot();
    ## First coordinate while positive -- lift the grabber further and fruther upwards
    ## Second coordinate any value other than 0 will do a -/+ 90deg
    # counter = 0;


    coords = [216, 56]

    robotic_arm_standard_position()
    open_gripper();
    move_robotic_arm([0,0,0,0])
    close_gripper()
    robotic_arm_standard_position()
    # while counter < 25 :
    #     print counter;  
    #     move_robotic_arm([0,0,0,0])
    #     counter = counter -3;


