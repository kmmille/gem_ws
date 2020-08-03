#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from math import *
import numpy as np

from sensor import static_obstacles, dynamic_obstacles
from controller import bicycleModel
from planner import find_xref

# This is the node where everything will get planned
pt_list = []

def main(T_s=2, d_s=10, actor_list=[], max_segs=100, T_synth=0.5):
	global pt_list
	# Create the node to publish the path
	pub = rospy.Publisher('path', numpy_msg(Floats), queue_size=10)
	rospy.init_node('planner', anonymous=True)
	# r = rospy.Rate(10) # 10hz

	# Initiate the model
	model = bicycleModel()

	while not rospy.is_shutdown():
		# Sense the obstacles
		obs_list = static_obstacles(d_s) + dynamic_obstacles(actor_list, d_s, T_s)

		# Create the initial set
		currState = model.getModelState()
		current_heading = model.quaternion_to_euler(currState.pose.orientation.x, \
			currState.pose.orientation.y, currState.pose.orientation.z,\
			currState.pose.orientation.w)

		sx = currState.pose.position.x # [m]
		sy = currState.pose.position.y  # [m]
		stheta = current_heading[2] * 180 / pi

		gx = round(30) #(50)  # [m]
		gy = round(10) #(80)  # [m]
		gtheta = round(180)

		grid_size = 1.0

		bloat_list = [1, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
		theta = [(sx-0.5, sy-0.5), (sx+0.5, sy+0.5)]
		goal = [(gx-0.5, gy-0.5), (gx+0.5, gy+0.5)]

		# Compute the path
		pt_list = find_xref(theta, goal, obs_list, max_segs, 0, bloat_list, wypt = None)

		# Publish the new path
		a = np.array(pt_list, dtype = np.float32)
		pub.publish(a)
		rospy.sleep(T_synth)

	return None

if __name__ == '__main__':
	main()
