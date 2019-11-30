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

def main():
	# l - distance to s
	# h - stair height
	# r - stair run
	# w = stair width
	# using 9in, 7in, 12in, and 24in in mm for base staircase (based on Al-b's staircase)
	staircase = World(228.6, 177.8, 304.8, 609.6)
	# using Al-b's biometric data for leg length and foot size
	subject = Person(1000, 279.4)
	

# def forward_kin(thi_len, shank_len, foot_len, q1, q2, q3, hip_pos):
#     q1 = float(q1 * (math.pi) / 180)
#     q2 = float(q2 * (math.pi) / 180)
#     q3 = float(q3 * (math.pi) / 180)

#     print(hip_pos[1])
#     hip_pos = np.array([0, hip_pos[1], hip_pos[2]])
#     knee_pos = np.array([0, (thi_len * cos(q1) + hip_pos[1]), -(thi_len * sin(q1) + hip_pos[2])])
#     ankle_pos = np.array([0, (shank_len * cos(q1 + q2) + thi_len * cos(q1) + hip_pos[1]),
#                           -(shank_len * sin(q1 + q2) + thi_len * sin(q1) + hip_pos[2])])
#     toe_pos = np.array([0, (shank_len * cos(q1 + q2) + thi_len * cos(q1) - foot_len * sin(q1 + q2 + q3) + hip_pos[1]),
#                         -(shank_len * sin(q1 + q2) + thi_len * sin(q1) + foot_len * sin(q1 + q2 + q3) + hip_pos[2])])
#     xpos = np.array([hip_pos[1], knee_pos[1], ankle_pos[1], toe_pos[1]])
#     ypos = np.array([hip_pos[2], knee_pos[2], ankle_pos[2], toe_pos[2]])
#     return np.array([xpos, ypos])


# def plot_leg(angles, hip_pos):
#     plt.ion()
#     fig = plt.figure()
#     axes = fig.add_subplot(111)
#     p, = plt.plot([], [], 'r-')
#     plt.xlabel('x')
#     plt.title('walk_one_leg')
#     axes.set_xlim([-1000,2000])
#     axes.set_ylim([-2500, 2000])
#     for m in range(0, len(angles[0])):
#         curr_hip_pos = np.array([0, hip_pos[1, m], hip_pos[2, m]])
#         positions = forward_kin(300.0, 300.0, 125.0, angles[0][m], (angles[1][m]+90), angles[2][m]-90, curr_hip_pos)
#         print(positions)
#         p.set_data(positions[0],positions[1])
#         fig.canvas.draw()


if __name__ == '__main__':
	main()