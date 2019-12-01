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


class Person(object):
    """
    Creates a person for accessing data and makings simplified plots
    """

    # leg_length is float length of leg in mm, foot_length is float length of foot in mm, height is float height of participant in mm
    def __init__(self, leg_length, foot_length):
        # biometric data
        self._thigh_length = leg_length*0.48
        self._shank_length = leg_length*0.52
        self._foot_length = foot_length  
        
        # global segment position data, using starting position of person as origin of world
        # each segment is an array of 3D coordinates, to be able to easily plot in 3D or in 2D
        self._left_hip_pos = [0, 0, leg_length]
        self._left_knee_pos = [0, 0, self._shank_length]
        self._left_ankle_pos = [0, 0, 0,]

        self._right_hip_pos = [0, 0, leg_length]
        self._right_knee_pos = [0, 0, self._shank_length]
        self._right_ankle_pos = [0, 0, 0,]
        
        #self._left_leg_pos = []
        #self._right_leg_pos = []
        
        # self._world = world
        # self._2d_fig = self.plot_2D_person(self._world)
        # self._3d_fig = self.plot_3D_person(self._world)
        # plt.plot()


    # root is a boolean indicating whether the foot is base frame instead of hip being base frame.  
    # q1 is ankle angle, q2 is knee angle, q3 is hip angle
    def left_forward_kinematics(self, q1, q2, q3, root):
        # left leg is root leg, thus left foot is base frame for left leg 
        if root == True:
            # Calculate positions of knee and hip (using self._left_ankle_pos[1] for y because everything is on x-z coordinate plane so y remains constant)
            self._left_knee_pos = [self._left_ankle_pos[0] + self._shank_length*math.cos(q1), self._left_ankle_pos[1], self._left_ankle_pos[2] + self._shank_length*math.sin(q1)]
            self._left_hip_pos = [self._left_knee_pos[0] + self._thigh_length*math.cos(q1+q2), self._left_ankle_pos[1], self._left_knee_pos[2] + self._thigh_length*math.sin(q1+q2)]

        # left leg is moving, thus left hip is base frame for left leg
        else: 
            # Calculate positions of knee and foot
            self._left_knee_pos = [self._left_hip_pos[0] - self._thigh_length*math.sin(q3), self._left_hip_pos[1], self._left_hip_pos[2] - self._thigh_length*math.cos(q3)]
            self._left_ankle_pos = [self._left_knee_pos[0] - self._shank_length*math.sin(q3-q2), self._left_hip_pos[1], self._left_knee_pos[2] - self._shank_length*math.cos(q3-q2)]

    # root is a boolean indicating whether the foot is base frame instead of hip being base frame.
    # q1 is ankle angle, q2 is knee angle, q3 is hip angle
    def right_forward_kinematics(self, q1, q2, q3, root):
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


    # def plot_2D_person(self, world):
    #     fig = plt.figure()
    #     plt.plot([0,self._l,self._l,self._l+self._r*1,self._l+self._r*1,self._l+self._r*2,self._l+self._r*2,self._l+self._r*3],
    #              [0,0,self._h*1,self._h*1,self._h*2,self._h*2,self._h*3,self._h*3])
    #     plt.axis([0,self._l+self._r,0,self._h*1.2])
    #     plt.show()
    #     return fig

    # def plot_3D_person(self):
    #     #need to incorporate width
    #     fig = plt.figure()
    #     plt.plot([0,self._l,self._l,self._l+self._r*1,self._l+self._r*1,self._l+self._r*2,self._l+self._r*2,self._l+self._r*3],
    #              [0,0,self._h*1,self._h*1,self._h*2,self._h*2,self._h*3,self._h*3])
    #     plt.axis([0,self._l+self._r,0,self._h*1.2])
    #     plt.show()
    #     return fig
    

    # assuming 3 axes in fig. First is world info like ground and stairs.
    # second is left leg and third is right leg.
    # dim is to update 2D vs 3D, 2 is 2D, 3 is 3D
    def update_leg_pos(self, fig, dim):
        if dim == 2:
            if len(fig.axes) == 1:
                fig.Axes.plot(self._left_leg_pos(0), self._left_leg_pos(0), 'ro',
                         self._right_leg_pos(0), self._right_leg_pos(0), 'blo')
            fig.axes(1).set_xdata(self._left_leg_pos(0)) #update x values for left leg
            fig.axes(1).set_ydata(self._left_leg_pos(1)) #update y values for left leg
            fig.axes(2).set_xdata(self._right_leg_pos(0)) #update y values for right leg
            fig.axes(2).set_xdata(self._left_leg_pos(1)) #update y values for right leg
        return fig
