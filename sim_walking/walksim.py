#!/usr/bin/env python
import numpy as np
import math
from numpy import *
from math import sqrt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# import Person and World objects
from person_simplified import Person_simplified
from world import *
from LegsDMP import LegsDMP

# l - distance to s
# h - stair height
# r - stair run
# w = stair width
l = 2280.6
h = 177.8
r = 304.8
w = 609.6
# using 9in, 7in, 12in, and 24in in mm for base staircase (based on Al-b's staircase)

subject = Person_simplified(1000, 279.4);
dmp_runner = LegsDMP()
dmp_runner.train_legs("walk_06_01_rightleg_halfstep.csv", 1000, 1);
# using Al-b's biometric data for leg length and foot size

step_length = subject._leg_lenth * 0.413
steps = l / step_length ;

walking_start_end_poses = []
stair_climb_start_end_poses = []

print("start position")
subject.print_pos()
jangles = subject.inverse_kinematics();
print(jangles);
subject.forward_kinematics(jangles);
print("start position-after-ik-k")
subject.print_pos()
jangles = subject.inverse_kinematics();
print(jangles)


plt.figure(1)

plt.plot(subject._left_hip_pos[0], subject._left_hip_pos[2], 'ro')
plt.plot(subject._left_knee_pos[0], subject._left_knee_pos[2], 'go')
plt.plot(subject._left_ankle_pos[0], subject._left_ankle_pos[2], 'bo')
plt.plot(subject._left_toe_pos[0], subject._left_toe_pos[2], 'mo')
print("moving")
foot = subject.getTrailingFoot();
foot_pos = foot[1];

#subject.set_foot(foot[0], (707.1), foot_pos[1], 707.1+279);
subject.move_foot(100,0)
#goal_joint_angles = subject.inverse_kinematics(); 
subject.print_pos()

#plt.show()
#plt.clf()

plt.plot(subject._left_hip_pos[0], subject._left_hip_pos[2], 'ro')
plt.plot(subject._left_knee_pos[0], subject._left_knee_pos[2], 'go')
plt.plot(subject._left_ankle_pos[0], subject._left_ankle_pos[2], 'bo')
plt.plot(subject._left_toe_pos[0], subject._left_toe_pos[2], 'mo')
	
subject.move_foot(100,0)
#goal_joint_angles = subject.inverse_kinematics(); 
subject.print_pos()

#plt.show()
#plt.clf()

plt.plot(subject._left_hip_pos[0], subject._left_hip_pos[2], 'ro')
plt.plot(subject._left_knee_pos[0], subject._left_knee_pos[2], 'go')
plt.plot(subject._left_ankle_pos[0], subject._left_ankle_pos[2], 'bo')
plt.plot(subject._left_toe_pos[0], subject._left_toe_pos[2], 'mo')
#plt.show()
#plt.clf()

subject = Person_simplified(1000, 279.4)

previous_angles = subject.inverse_kinematics()
walking_start_end_poses = []
for i in range(0, int(steps)):
	start_joint_angles = previous_angles

	goal_joint_angles = subject.move_foot(step_length, 0);

	joint_pairs = []
	for j in range(0, 3):
		joint_pairs.append((start_joint_angles[j], goal_joint_angles[j]))
	previous_angles = goal_joint_angles;

	walking_start_end_poses.append(joint_pairs);

	# plt.figure(1)
	# plt.plot(subject._left_hip_pos[0], subject._left_hip_pos[2], 'ro')
	# plt.plot(subject._left_knee_pos[0], subject._left_knee_pos[2], 'go')
	# plt.plot(subject._left_ankle_pos[0], subject._left_ankle_pos[2], 'bo')
	# plt.plot(subject._left_toe_pos[0], subject._left_toe_pos[2], 'mo')
	# plt.pause(0.01)

stair_count = 1;
for i in range(0, int(stair_count)):

	start_joint_angles = previous_angles
	goal_joint_angles = subject.move_foot(r*0.75, h);

	joint_pairs = []
	for j in range(0, 3):
		joint_pairs.append((start_joint_angles[j], goal_joint_angles[j]))
	previous_angles = goal_joint_angles;	

	stair_climb_start_end_poses.append(joint_pairs)

# 	plt.figure(1)
# 	plt.plot(subject._left_hip_pos[0], subject._left_hip_pos[2], 'ro')
# 	plt.plot(subject._left_knee_pos[0], subject._left_knee_pos[2], 'go')
# 	plt.plot(subject._left_ankle_pos[0], subject._left_ankle_pos[2], 'bo')
# 	plt.plot(subject._left_toe_pos[0], subject._left_toe_pos[2], 'mo')

# plt.show()
print(walking_start_end_poses)
print(stair_climb_start_end_poses)

walk_angles = dmp_runner.move(walking_start_end_poses,1)

stair_angles = dmp_runner.move(stair_climb_start_end_poses,1)

subject2 = Person_simplified(1000, 279.4)
#staircase = World(l, h, r, w, subject2._leg_lenth)
i = 0;
data_frames = []
print(len(walk_angles))
for i in walk_angles:
	print("step " + str(len(i)))
	jangles = []
	for j in i:
		#disp(j)
		subject2.forward_kinematics2(j, (step_length/len(i)), 0);
		x = [subject2._left_hip_pos[0], subject2._left_knee_pos[0], subject2._left_ankle_pos[0], subject2._left_toe_pos[0]]
		y = [subject2._left_hip_pos[2], subject2._left_knee_pos[2], subject2._left_ankle_pos[2], subject2._left_toe_pos[2]]
		data_frames.append([x, y])
		# plt.figure(1) 
		# plt.plot(subject2._left_hip_pos[0], subject2._left_hip_pos[2], 'ro')
		# plt.plot(subject2._left_knee_pos[0], subject2._left_knee_pos[2], 'go')
		# plt.plot(subject2._left_ankle_pos[0], subject2._left_ankle_pos[2], 'bo')
		# plt.plot(subject2._left_toe_pos[0], subject2._left_toe_pos[2], 'mo')
		jangles = j
	# The starting position is stainding striaght up, and since there is no back leg
	# the leg doesn't come back down which requires the back hip rotating 
	# soe we drop it artificially 
	distance_down = 1000 - subject2._left_toe_pos[0]
	for i in range(0, 50):
		plt.figure(1) 
		subject2.step_down(distance_down/50)
		subject2.forward_kinematics2(j, 0,0);
		x = [subject2._left_hip_pos[0], subject2._left_knee_pos[0], subject2._left_ankle_pos[0], subject2._left_toe_pos[0]]
		y = [subject2._left_hip_pos[2], subject2._left_knee_pos[2], subject2._left_ankle_pos[2], subject2._left_toe_pos[2]]
		data_frames.append([x, y])
		# plt.plot(subject2._left_hip_pos[0], subject2._left_hip_pos[2], 'ro')
		# plt.plot(subject2._left_knee_pos[0], subject2._left_knee_pos[2], 'go')
		# plt.plot(subject2._left_ankle_pos[0], subject2._left_ankle_pos[2], 'bo')
		# plt.plot(subject2._left_toe_pos[0], subject2._left_toe_pos[2], 'mo')
	# plt.show()
	# plt.pause(0.1)

print(len(stair_angles))
for i in stair_angles:
	print("step " + str(len(i)))
	for j in i:
		#disp(j) 
		
		subject2.forward_kinematics2(j, ((r*0.75)/len(i)), (h/(len(i))));

		x = [subject2._left_hip_pos[0], subject2._left_knee_pos[0], subject2._left_ankle_pos[0], subject2._left_toe_pos[0]]
		y = [subject2._left_hip_pos[2], subject2._left_knee_pos[2], subject2._left_ankle_pos[2], subject2._left_toe_pos[2]]
		data_frames.append([x, y])

	# 	plt.figure(1) 
	# 	plt.plot(subject2._left_hip_pos[0], subject2._left_hip_pos[2], 'ro')
	# 	plt.plot(subject2._left_knee_pos[0], subject2._left_knee_pos[2], 'go')
	# 	plt.plot(subject2._left_ankle_pos[0], subject2._left_ankle_pos[2], 'bo')
	# 	plt.plot(subject2._left_toe_pos[0], subject2._left_toe_pos[2], 'mo')
	# plt.show()
	# plt.pause(0.1)

Writer = animation.writers['ffmpeg']
writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
fig, ax = plt.subplots()
ax.axis([-500, 1200, -100, l + 1000])
line, = ax.plot([], [])
line2, = ax.plot([subject2._leg_lenth, subject2._leg_lenth, subject2._leg_lenth - h, subject2._leg_lenth - h],
				 [0, l, l, l+r])

def init():
	line, = ax.plot([], [])
	line2, = ax.plot([subject2._leg_lenth, subject2._leg_lenth, subject2._leg_lenth - h, subject2._leg_lenth - h],
				 [0, l, l, l+r])

	return line, line2,

def animate(i):
	#print(i)
	#print(data_frames[i])
	y = data_frames[i]
	line.set_data(y)	
	return line, line2,



ani = animation.FuncAnimation(fig, animate, init_func=init, interval=10, frames=len(data_frames), blit=True, save_count=50)
ani.save('walking.mp4', writer=writer)
plt.show()
input("asdf")

		
			


