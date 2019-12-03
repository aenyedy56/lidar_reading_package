#!/usr/bin/env python
import numpy as np
#import math
from numpy import *
from math import sqrt
from math import sin
from math import cos
from sympy import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# contains methods for setting the foot, finding the trailing foot, and computing inverse kinematics on a given frame
# inverse kinematics assumes FOOT IS ON THE GROUND and not in an intermediate step (have to keep angle between ankle and floor 0-degrees)
# see inverse_kinematics_setup_diagram.png for image showing assumptions on angle locations/definitions 
class Person(object):
    """
    Creates a person for accessing data and makings simplified plots
    """

    # leg_length is float length of leg in mm, _foot_length is float length of foot in mm, height is float height of participant in mm
    def __init__(self, leg_length, foot_length):
        # biometric data
        self._leg_lenth = leg_length;
        self._thigh_length = leg_length*0.48
        self._shank_length = leg_length*0.52
        self._foot_length = foot_length  
        
        # global segment position data, using starting position of person as origin of world
        # each segment is an array of 3D coordinates, to be able to easily plot in 3D or in 2D
        self._left_hip_pos = [0, 0, leg_length]
        self._left_knee_pos = [0, 0, self._shank_length]
        self._left_ankle_pos = [0, 0, 0,]
        self._left_toe_pos = [self._foot_length, 0, 0]

        self._right_hip_pos = [0, 0, leg_length]
        self._right_knee_pos = [0, 0, self._shank_length]
        self._right_ankle_pos = [0, 0, 0,]
        self._right_toe_pos = [self._foot_length, 0, 0]
        
    # set foot's toe location, defined by 'L' or 'R' for left or right foot
    def set_foot(self, foot, x, y, z):
        if foot == 'L':
            self._left_toe_pos = [x, y, z]
            self._left_ankle_pos = [x-self._foot_length, y, z]
            # calculation of knee and hip position too?
        else:
            self._right_toe_pos = [x, y, z]
            self._right_ankle_pos = [x-self._foot_length, y, z]
            # calculation of knee and hip position too?

    # returns tuple of foot (L or R) and it's xyz location
    def getTrailingFoot(self):
        if self._left_toe_pos[0] < self._right_toe_pos[0]:
            return ('L', self._left_toe_pos)
        elif self._right_toe_pos[0] < self._left_toe_pos[0]:
            return ('R', self._right_toe_pos)
        # if called at beginning, when both toes are in same position, then assume sim will lead with right foot thus list left foot as trailing foot
        else:
            return ('L', self._left_toe_pos)

    # returns array of 6 joint angles in orker {lhips, lknee, lankle, rhips, rknee, rankle}
    def inverse_kinematics(self):
        # see inverse_kinematics_setup_diagram.png for visual of what each theta references
        # using equations from paper https://www.researchgate.net/publication/291955972_An_inverse_kinematic_algorithm_for_the_human_leg
        # ** specifically from section 3, using equations 3.1

        # I am assuming that the coordinate system is using x-z for 2D plot instead of x-y, and z is the vertical axis in 3D plot 

        # calculating inverse kinematics for left leg:
        lhip_angle = np.arctan((self._left_toe_pos[2] - self._foot_length)/self._left_toe_pos[0]) + np.arccos((self._thigh_length**2 - self._shank_length**2 + self._left_toe_pos[0]**2 + (self._left_toe_pos[2]-self._foot_length)**2)/(2*self._thigh_length*math.sqrt(self._left_toe_pos[0]**2 + (self._left_toe_pos[2]-self._foot_length)**2)))

        lknee_angle = np.arccos((self._left_toe_pos[0]**2 + (self._left_toe_pos[2] - self._foot_length)**2 - self._thigh_length**2 - self._shank_length**2)/(2*self._thigh_length*self._shank_length))

        lankle_angle = lhip_angle - lknee_angle + (math.pi/2)

        # calculating inverse kinematics for right leg:
        rhip_angle = np.arctan((self._right_toe_pos[2] - self._foot_length)/self._right_toe_pos[0]) + np.arccos((self._thigh_length**2 - self._shank_length**2 + self._right_toe_pos[0]**2 + (self._right_toe_pos[2]-self._foot_length)**2)/(2*self._thigh_length*math.sqrt(self._right_toe_pos[0]**2 + (self._right_toe_pos[2]-self._foot_length)**2)))

        rknee_angle = np.arccos((self._right_toe_pos[0]**2 + (self._right_toe_pos[2] - self._foot_length)**2 - self._thigh_length**2 - self._shank_length**2)/(2*self._thigh_length*self._shank_length))

        rankle_angle = rhip_angle - rknee_angle + (math.pi/2)


        # returning array of jangles
        jangles_array = [lhip_angle, lknee_angle, lankle_angle, rhip_angle, rknee_angle, rankle_angle]
        return jangles_array



    # vvvv :FORWARD KINEMATICS IF REQUIRED: vvvv
    
    # # root is a boolean indicating whether the foot is base frame instead of hip being base frame.  
    # # q1 is ankle angle, q2 is knee angle, q3 is hip angle
    # def left_forward_kinematics(self, q1, q2, q3, root):
    #     # left leg is root leg, thus left foot is base frame for left leg 
    #     if root == True:
    #         # Calculate positions of knee and hip (using self._left_ankle_pos[1] for y because everything is on x-z coordinate plane so y remains constant)
    #         self._left_knee_pos = [self._left_ankle_pos[0] + self._shank_length*math.cos(q1), self._left_ankle_pos[1], self._left_ankle_pos[2] + self._shank_length*math.sin(q1)]
    #         self._left_hip_pos = [self._left_knee_pos[0] + self._thigh_length*math.cos(q1+q2), self._left_ankle_pos[1], self._left_knee_pos[2] + self._thigh_length*math.sin(q1+q2)]

    #     # left leg is moving, thus left hip is base frame for left leg
    #     else: 
    #         # Calculate positions of knee and foot
    #         self._left_knee_pos = [self._left_hip_pos[0] - self._thigh_length*math.sin(q3), self._left_hip_pos[1], self._left_hip_pos[2] - self._thigh_length*math.cos(q3)]
    #         self._left_ankle_pos = [self._left_knee_pos[0] - self._shank_length*math.sin(q3-q2), self._left_hip_pos[1], self._left_knee_pos[2] - self._shank_length*math.cos(q3-q2)]

     # root is a boolean indicating whether the foot is base frame instead of hip being base frame.
     # q1 is ankle angle, q2 is knee angle, q3 is hip angle
    def right_forward_kinematics(self, q1, q2, q3, root):
        q1 = float(q1)
        q2 = float(q2)
        q3 = float(q3)
        # right leg is root leg, thus right foot is base frame for right leg
        if root == True:
        # Calculate positions of knee and hip (using self._right_ankle_pos[1] for y because everything is on x-z coordinate plane so y remains constant)
            self._right_knee_pos = [self._right_ankle_pos[0] + self._shank_length*math.cos(q1), self._right_ankle_pos[1], self._right_ankle_pos[2] + self._shank_length*math.sin(q1)]
            self._right_hip_pos = [self._right_knee_pos[0] + self._thigh_length*math.cos(q1+q2), self._right_ankle_pos[1], self._right_knee_pos[2] + self._thigh_length*math.sin(q1+q2)]
        # right leg is moving, thus right hip is base frame for right leg
        else:
        # Calculate positions of knee and foot
            self._right_knee_pos = [self._right_hip_pos[0] - self._thigh_length*math.sin(q3), self._right_hip_pos[1], self._right_hip_pos[2] - self._thigh_length*math.cos(q3)]
            self._right_ankle_pos = [self._right_knee_pos[0] - self._shank_length*math.sin(q3-q2), self._right_hip_pos[1], self._right_knee_pos[2] - self._shank_length*math.cos(q3-q2)]
