#!/usr/bin/env python
#Should be renamed to World.py
import numpy as np
import math
from numpy import *
from math import sqrt
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class World(object):
	def __init__(self,l,h,r,w):
		# l - distance to s
		# h - stair height
		# r - stair run
		# w = stair width

		# these need to change later to incorporate the height data from the markers
		self._l = l
		self._h = h
		self._r = r
		self._w = w
		self._2d_fig = plot_2D_world()
		self._3d_fig = plot_3D_world()
		plt.plot()

	def plot_2D_world(self):
		fig = plt.figure()
		plt.plot([0,self._l,self._l,self._l+r*1,self._l+r*1,self._l+r*2,self._l+r*2,self._l+r*3],
				 [0,0,self._h*1,self._h*1,self._h*2,self._h*2,self._h*3,self._h*3])
		plt.axis([0,2.5,0,5])
		plt.show()
		return fig

	def plot_3D_world(self):
		#need to incorporate width
		fig = plt.figure()
		plt.plot([0,self._l,self._l,self._l+r*1,self._l+r*1,self._l+r*2,self._l+r*2,self._l+r*3],
				 [0,0,self._h*1,self._h*1,self._h*2,self._h*2,self._h*3,self._h*3])
		plt.axis([0,2.5,0,5])
		plt.show()
		return fig

"""
world = World()
floorL = 2
stepH = 0.133
stepR = 0.28
plot_world(floorL,stepH,stepR)
"""