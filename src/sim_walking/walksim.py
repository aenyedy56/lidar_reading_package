#!/usr/bin/env python
import numpy as np
import math
from numpy import *
from math import sqrt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# import Person and World objects
from person_geometric import Person
from world import *
from dmp_walking import DMP
import rospy
from lidar_reading_package.msg import Stairs
  #  7     pub = rospy.Publisher('chatter', String, queue_size=10)
  #  8     rospy.init_node('talker', anonymous=True)
  #  9     rate = rospy.Rate(10) # 10hz
  # 10     while not rospy.is_shutdown():
  # 11         hello_str = "hello world %s" % rospy.get_time()
  # 12         rospy.loginfo(hello_str)
  # 13         pub.publish(hello_str)
  # 14         rate.sleep()

  # rospy.init_node('listener', anonymous=True)
  # 16 
  # 17     rospy.Subscriber("chatter", String, callback)
  # 18 
  # 19     # spin() simply keeps python from exiting until this node is stopped
  # 20     rospy.spin()


class WalkSim():

	def __init__(self):
		# using 9in, 7in, 12in, and 24in in mm for base staircase (based on Al-b's staircase)
		
		# using Al-b's biometric data for leg length and foot size
		self.subject = Person(1000, 279.4)

		self.dmp_runner = DMP("leg_info.csv")
		self.dmp_runner.train();

		self.plan = True;

		print("start position")
		self.subject.print_pos()
		jangles = self.subject.inverse_kinematics()
		print(jangles);
		self.subject.forward_kinematics(jangles)
		print("start position-after-ik-k")
		self.subject.print_pos()
		jangles = self.subject.inverse_kinematics()
		print(jangles)

		rospy.init_node('stair_path_planner')
		rospy.Subscriber("stairs", Stairs, self.walk)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()


	def walk(self, data):
		if(self.plan):
			self.plan = False
		else:
			return;

		print(data);
		stair_obj = data.stairs;
		# These should come from the message not be hard coded 
		# l - distance to s
		# h - stair height
		# r - stair run
		# w = stair width
		l = stair_obj[0].distance_to_stair
		h = stair_obj[0].height
		r = stair_obj[0].depth
		w = 609.6
		step_length = self.subject._leg_lenth * 0.413
		steps = 1
		# want to target getting within half a step of the stair so the next step takes us up the stair 
		dist_to_stair = l - (step_length/2)
		# if we are within a single step we are good
		if(dist_to_stair < 0):
			steps = 0
		else:
			step_length_remainder = dist_to_stair % step_length
			if (step_length_remainder == 0):
				# if their step magically lines up with the stairs don't do anything 
				#get the number of steps to get to the stairs based on their base stride 
				steps = floor(dist_to_stair / step_length)
			else:
				# add an additional step
				print(steps)
				steps = floor(dist_to_stair / step_length) + 1;
				print(steps)
				# and shorten the stride length by the remainder over the steps 
				step_length = step_length - ((step_length-step_length_remainder)/steps)
			


		walking_start_end_poses = []
		stair_climb_start_end_poses = []
		for i in range(0, int(steps)):
			start_joint_angles = self.subject.inverse_kinematics();
			foot = self.subject.getTrailingFoot();
			foot_pos = foot[1];

			self.subject.set_foot(foot[0], (foot_pos[0]), foot_pos[1], foot_pos[2]+step_length);
			goal_joint_angles = self.subject.inverse_kinematics(); 

			joint_pairs = []
			for j in range(0, 6):
				joint_pairs.append((start_joint_angles[j], goal_joint_angles[j]))

			walking_start_end_poses.append(joint_pairs);
			print("position update - forward")
			print(str(self.subject._right_hip_pos[0]) + ' ' + str(self.subject._right_hip_pos[2]))
			print(str(self.subject._right_knee_pos[0]) + ' ' + str(self.subject._right_knee_pos[2]))
			print(str(self.subject._right_ankle_pos[0]) + ' ' + str(self.subject._right_ankle_pos[2]))
			print(str(self.subject._right_toe_pos[0]) + ' ' + str(self.subject._right_toe_pos[2]))

		stair_count = len(stair_obj);
		for i in range(0, int(stair_count)):
			start_joint_angles = self.subject.inverse_kinematics();
			foot = self.subject.getTrailingFoot();
			foot_pos = foot[1];

			self.subject.set_foot(foot[0], foot_pos[0]+ h , foot_pos[1], foot_pos[2] + (r * 0.75));	
			goal_joint_angles = self.subject.inverse_kinematics(); 

			joint_pairs = []
			for j in range(0, 6):
				joint_pairs.append((start_joint_angles[j], goal_joint_angles[j]))

			stair_climb_start_end_poses.append(joint_pairs)
			print("position update - up")
			print(str(self.subject._right_hip_pos[0]) + ' ' + str(self.subject._right_hip_pos[2]))
			print(str(self.subject._right_knee_pos[0]) + ' ' + str(self.subject._right_knee_pos[2]))
			print(str(self.subject._right_ankle_pos[0]) + ' ' + str(self.subject._right_ankle_pos[2]))
			print(str(self.subject._right_toe_pos[0]) + ' ' + str(self.subject._right_toe_pos[2]))

		angles = []	
		for i in walking_start_end_poses:
			print("walk ")
			angles.append(self.dmp_runner.run(i))

		for i in stair_climb_start_end_poses:
			print("step")
			angles.append(self.dmp_runner.run(i))

		staircase = World(l,h,r,w)
		subject2 = Person(1000, 279.4)
		plt.plot(subject2._right_hip_pos[0], subject2._right_hip_pos[2], 'ro')
		plt.plot(subject2._right_knee_pos[0], subject2._right_knee_pos[2], 'go')
		plt.plot(subject2._right_ankle_pos[0], subject2._right_ankle_pos[2], 'bo')
		plt.plot(subject2._right_toe_pos[0], subject2._right_toe_pos[2], 'ro')
		
		plt.pause(0.001)
		plt.show()
		i = 0;
		for i in angles:	
			print(len(angles))
			for j in i:
				print(len(i))
				subject2.forward_kinematics(j);
				plt.figure(1)
				print(str(subject2._right_hip_pos[0]) + ' ' + str(subject2._right_hip_pos[2]))
				print(str(subject2._right_knee_pos[0]) + ' ' + str(subject2._right_knee_pos[2]))
				print(str(subject2._right_ankle_pos[0]) + ' ' + str(subject2._right_ankle_pos[2]))
				print(str(subject2._right_toe_pos[0]) + ' ' + str(subject2._right_toe_pos[2]))

				plt.plot(subject2._right_hip_pos[0], subject2._right_hip_pos[2], 'ro')
				plt.plot(subject2._right_knee_pos[0], subject2._right_knee_pos[2], 'go')
				plt.plot(subject2._right_ankle_pos[0], subject2._right_ankle_pos[2], 'bo')
				plt.plot(subject2._right_toe_pos[0], subject2._right_toe_pos[2], 'ro')
				
				plt.pause(0.001)
				plt.show()
			break;	
			

if __name__ == '__main__':
	try:
		ne = WalkSim()
	except rospy.ROSInterruptException: pass	