#!/usr/bin/env python
from __future__ import division
import polytope as pc
import numpy as np
from gurobipy import *
from math import *
import matplotlib.pyplot as plt

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

# All polytopes are rectangles
# The rectangles are defined as ((x_min, y_min), (x_max, y_max))

T_synth = 0.5

def bloat_poly(poly, bloat_factor):
	b = poly.b
	b_new = []
	for num in b:
		b_new.append(num + bloat_factor)
	return np.array(b_new)


def add_initial_constraints(model, x, Theta_rect):
# def add_initial_constraints(Theta_rect):
	dims = 2
	# Make sure to start in Theta
	x_min, y_min = Theta_rect[0]
	x_max, y_max = Theta_rect[1]

	# Find center of initial set
	x_c = (x_min + x_max)/2
	y_c = (y_min + y_max)/2
	x0 = [x_c, y_c]

	model.addConstrs(x[i] == x0[i] for i in range(dims))
	return None

def add_final_constraints(model, x, goal_rect):
	dims = 2
	# Make sure to end in goal
	x_min, y_min = goal_rect[0]
	x_max, y_max = goal_rect[1]

	goal = pc.box2poly([[x_min, x_max], [y_min, y_max]])

	x_c = (x_min + x_max)/2
	y_c = (y_min + y_max)/2
	xf = [x_c, y_c]

	model.addConstrs(x[i] == xf[i] for i in range(dims))
	return None

def add_obstacle_constraint(model, xlist, A, b, bloat_factor, ob_num, seg_num):
	# Avoid a polytope
	edges = len(b)
	dims = len(A[0])
	# Use bigM method to avoid polytopes
	M = 1e6
	alpha = model.addVars(edges, vtype=GRB.BINARY, name="alpha_ob"+str(ob_num)+"_seg"+str(seg_num))

	x1 = xlist[seg_num]
	x2 = xlist[seg_num+1]
	for edge in range(edges):
		tem_constr1 = 0
		tem_constr2 = 0
		h_norm = np.linalg.norm(A[edge])
		for i in range(dims):
			tem_constr1 += A[edge][i]*x1[i]
			tem_constr2 += A[edge][i]*x2[i]
		tem_constr1 = -tem_constr1 + (b[edge] + h_norm*bloat_factor)
		tem_constr2 = -tem_constr2 + (b[edge] + h_norm*bloat_factor)
		model.addConstr(tem_constr1 <= M*(1 - alpha[edge]))
		model.addConstr(tem_constr2 <= M*(1 - alpha[edge]))
	model.addConstr(alpha.sum() >= 1)
	return None

def add_avoidance_constraints(model, xlist, obs, max_segs, bloat_factors):
	# Avoid each polytope in the list
	for seg_num in range(max_segs):
		# bloat_factor = bloat_factors[seg_num]
		bloat_factor = 1.2

		for ob_num in range(len(obs)):
			x_min, y_min, z_min = obs[ob_num][0]
			x_max, y_max, z_max = obs[ob_num][1]

			ob = pc.box2poly([[min(x_min, x_max), max(x_min, x_max)], [min(y_min, y_max), max(y_min, y_max)]])

			add_obstacle_constraint(model, xlist, ob.A, ob.b, bloat_factor, ob_num, seg_num)
	return None

def add_space_constraints(model, xlist, limits):
	xlim, ylim = limits
	for x in xlist:
		model.addConstr(x[0] >= xlim[0])
		model.addConstr(x[1] >= ylim[0])
		model.addConstr(x[0] <= xlim[1])
		model.addConstr(x[1] <= ylim[1])

	return None

def find_xref(Theta, goal, obs, max_segs, l_min, bloat_list, wypt = None):

	for num_segs in range(1, max_segs):
		xlist = []
		m = Model("xref")
		for i in range(num_segs+1):
			xnew = m.addVars(2)
			xlist.append(xnew)

		obj = 0
		x_min, y_min = goal[0]
		x_max, y_max = goal[1]
		x_c = (x_min + x_max)/2
		y_c = (y_min + y_max)/2
		xf = [x_c, y_c]

		for i in range(len(xlist)):
			tem_obj = (x_c - xlist[i][0])*(x_c - xlist[i][0]) + (y_c - xlist[i][1])*(y_c - xlist[i][1])
			obj += tem_obj
		m.setObjective(obj, GRB.MINIMIZE)

		if wypt != None:
			obj = (wypt[0] - xlist[1][0])*(wypt[0] - xlist[1][0]) + (wypt[1] - xlist[1][1])*(wypt[1] - xlist[1][1])
			m.setObjective(obj, GRB.MINIMIZE)

		m.setParam(GRB.Param.OutputFlag, 0)

		add_initial_constraints(m, xlist[0], Theta)
		add_final_constraints(m, xlist[-1], goal)
		add_avoidance_constraints(m, xlist, obs, num_segs, bloat_list)
		add_space_constraints(m, xlist, ([0, 85], [0, 85]))

		m.write("test.lp")

		m.optimize()

		try:
			wypts = []
			for x in xlist:
				wypts = wypts + [float(x[0].X),float(x[1].X)]
			m.dispose()
			return wypts

		except:
			m.dispose()

def talker():
	pub = rospy.Publisher('floats', numpy_msg(Floats), queue_size=10)
	rospy.init_node('talker', anonymous=True)
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		# This is where the sensor should go
		bloat_factors = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		theta_rect = ((0, 0), (1, 1))
		goal_rect = ((5, 5), (6, 6))
		obs = [((3, 3, 0),(4, 4, 0))]

		pt_list = find_xref(theta_rect, goal_rect, obs, 10, 0, bloat_factors)
		a = np.array(pt_list, dtype = np.float32)
		print(a.reshape((3, 2)))
		pub.publish(a)
		rospy.sleep(T_synth)


if __name__ == '__main__':

	talker()
