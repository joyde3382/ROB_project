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

def lim(n):
	# Finds the n value between minn og maxn
    return max(min(1.919, n), -1.919)

def invkin(xyz):
	d1 = 16.8; # cm (height of 2nd joint)
	a1 = 0; # (distance along "y-axis" to 2nd joint)
	a2 = 19.2; # (distance between 2nd and 3rd joints)
	d4 = 23.5; # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

	# Insert code here!!!
	xc = xyz[0]; yc = xyz[1]; zc = xyz[2]

	# # -1.919 is equals to 110 deg
	q1 = lim(math.atan2(yc, xc))
	
	r2 = (xc - a1*math.cos(q1))**2 + (yc - a1*math.sin(q1))**2
	s = (zc - d1)
	D = ( r2 + math.pow(s,2) - math.pow(a2,2) - math.pow(d4,2))/(2*a2*d4)

	q3 = lim(math.atan2(-math.sqrt(1-math.pow(D,2)), D))

	q2 = lim(math.atan2(s, math.sqrt(r2)) - math.atan2(d4*math.sin(q3), a2 + d4*math.cos(q3))-(math.pi/4))

	# TEST

	# x1 = xyz[0];
	# y1 = xyz[1];
	# z1 = xyz[2];

	# q1 = math.atan2(y1, x1)	

	# # Calculate q2 and q3
	# r2 = math.pow((x1 - a1 * math.cos(q1)),2) + math.pow((y1 - a1 * math.sin(q1)),2)
	# s = (z1 - d1)
	# D = (r2 + math.pow(s,2) - math.pow(a2,2) - math.pow(d4,2)) / (2 * a2 * d4)

	# q3 = math.atan2(-math.sqrt(1 - math.pow(D,2)), D)
	# q2 = math.atan2(s, math.sqrt(r2)) - math.atan2(d4 * math.sin(q3), a2 + d4 * math.cos(q3))-(math.pi/2)


	q4 = xyz[3]

	print "q1:" + str(math.degrees(q1)) + ", q2: " + str(math.degrees(q2)) + ", q3: " + str(math.degrees(q3)) + ", q4: " + str(math.degrees(q4));

	return q1,q2,q3,q4

class MoveRobot:

	N_JOINTS = 4

	def __init__(self,server_name, xpos_ypos_zpos_angle):
		self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)
		gripper = rospy.Publisher('/gripper/command', Float64, queue_size=10)

		self.joint_positions = []
		self.names =["joint1", "joint2", "joint3", "joint4"]

		xyz_positions = [
			[
				xpos_ypos_zpos_angle[0], 
				xpos_ypos_zpos_angle[1], 
				xpos_ypos_zpos_angle[2], 
				xpos_ypos_zpos_angle[3]
				]
			]		
		dur = rospy.Duration(5)

		# construct a list of joint positions by calling invkin for each xyz point
		for p in xyz_positions:
			jtp = JointTrajectoryPoint(positions=invkin(p),velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
			dur += rospy.Duration(2)
			self.joint_positions.append(jtp)

		# Here the the movement path is defined
		self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)

		# Here the goal to follow is set
		self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )

	def send_command(self):
		self.client.wait_for_server()
		self.client.send_goal(self.goal)
		self.client.wait_for_result()

def robotic_arm_standard_position():
	node = MoveRobot("/arm_controller/follow_joint_trajectory", [25, 0 , 15, 0])
	# Here we initiate the movement of the arm
	node.send_command()

def move_robotic_arm(xpos_ypos_zpos_angle):
    # Here we instanciate the new class
	node = MoveRobot("/arm_controller/follow_joint_trajectory", xpos_ypos_zpos_angle)
	# Here we initiate the movement of the arm
	node.send_command()

def open_gripper():
	pub = rospy.Publisher('gripper/command', Float64, queue_size=10)
	pub.publish(0)

def close_gripper():	
	pub = rospy.Publisher('gripper/command', Float64, queue_size=10)
	pub.publish(6)
	