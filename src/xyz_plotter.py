#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

# visualize the xyz data from the staircase plotting node
def xyz_vcallback(data):
	#rospy.loginfo("stair xyz data received")
	x = data.data[0:len(data.data)+1:3]
	z = data.data[2:len(data.data)+1:3]

	if len(x) > len(z):
		x = x[0:len(x)-1]

	xyz_vcallback.ax.plot(x, z)
	xyz_vcallback.ax.axhline(max(x), c='darkorange')
	xyz_vcallback.fig.canvas.draw()

	
# code uses info from: http://nigorojr.com/2019/01/28/using-matplotlib-to-plot-on-ros-subscriber-callback/
# initialize the xyz_plotter node for use with matplotlib visualizations
def main():
	# initialize xyz_plotter node and ensure rospy has unique node name by setting anonymous to true
	rospy.init_node('xyz_plotter', anonymous=True)

	# setup matplotlib figure
	xyz_vcallback.fig = plt.figure()
	xyz_vcallback.fig.suptitle('LiDAR Staircase Visualization')
	
	# define axes of matplotlib figure: when plotting data, use ax.plot(x, y)
	xyz_vcallback.ax = xyz_vcallback.fig.add_subplot(111)


	rospy.Subscriber("stair_xyz", Float32MultiArray, xyz_vcallback)

	# replaces rospy.spin() as the updating portion of the node
	# -> allows for realtime updates of matplotlib plot
	plt.show()

if __name__ == '__main__':
	main()