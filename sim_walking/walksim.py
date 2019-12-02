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
from dmp
def main():
	# l - distance to s
	# h - stair height
	# r - stair run
	# w = stair width
	l = 228.6
	h = 177.8
	r = 304.8
	w = 609.6
	# using 9in, 7in, 12in, and 24in in mm for base staircase (based on Al-b's staircase)
	staircase = World(l,h,r,w)
	# using Al-b's biometric data for leg length and foot size
	subject = Person(1000, 279.4)

	dmp_runner = DMP()
	dmp_runner.train();

	step_length = subject.height * .413

	int steps = step_length / l;

	walking_start_end_poses = []
	stair_climb_start_end_poses = []

	for i in range(steps):
		start_joint_angles = subject.inverse_kinematics();
		tuple foot = subject.getTrailingFoot();

		subject.set_foot(foot(0), foot(1)+step_length, foot(2), foot(3));
		goal_joint_angles = subject.inverse_kinematics(); 

		joint_pairs = []
		for j in range(6):
			joint_pairs[j] = tuple(start_joint_angles[j],goal_joint_angles[j])

		walking_start_end_poses[i] = joint_pairs;

	stair_count = 2;
	for i in range(stair_count):
		start_joint_angles = subject.inverse_kinematics();
		tuple foot = subject.getTrailingFoot();

		subject.set_foot(foot(0), foot(1) + (r * 0.75), foot(2), foot(3) + h);
		goal_joint_angles = subject.inverse_kinematics(); 

		joint_pairs = []
		for j in range(6):
			joint_pairs[j] = tuple(start_joint_angles[j],goal_joint_angles[j])

		stair_climb_start_end_poses[i] = joint_pairs;

	angles = []	
	for i in walking_start_end_poses:
		angles[i] = dmp_runner.execute(i);

	for i in stair_climb_start_end_poses:
		angles[i + len(walking_start_end_poses)] = dmp_runner.execute(i)


if __name__ == '__main__':
	main()