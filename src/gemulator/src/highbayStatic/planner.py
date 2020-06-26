from __future__ import division
import polytope as pc
import numpy as np
from gurobipy import *
from math import *

# All polytopes are rectangles
# The rectangles are defined as ((x_min, y_min), (x_max, y_max))

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
# def add_final_constraints(goal_rect, bloat_factor):
	# Make sure to end in goal
	x_min, y_min = goal_rect[0]
	x_max, y_max = goal_rect[1]

	goal = pc.box2poly([[x_min, x_max], [y_min, y_max]])

	# b_new = bloat_poly(goal, -bloat_factor)
	# print(b_new)

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
		model.addConstr(tem_constr2 <= M*(1-alpha[edge]))
		model.addConstr(alpha.sum() >= 1)
	return None

def add_avoidance_constraints(model, xlist, obs, max_segs, bloat_factors):
	# Avoid each polytope in the list
	for seg_num in range(max_segs):
		bloat_factor = bloat_factors[seg_num]
	# print(bloat_factor)
	# bloat_factor = 10
	for ob_num in range(len(obs)):
		x_min, y_min, z_min = obs[ob_num][0]
		x_max, y_max, z_max = obs[ob_num][1]

		ob = pc.box2poly([[x_min, x_max], [y_min, y_max]])

		# A, b = obs[ob_num]
		# print(b)
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

def find_xref(Theta, goal, obs, max_segs, l_min, bloat_list):
	# Find the entire path
	# dims = len(Theta[0][0])
	# # Find center of goal:
	# dx = goal[1][0][0] + goal[1][1][0]
	# dy = goal[1][2][0] + goal[1][3][0]
	#
	# xf = [float(goal[1][1][0] - float(dx/2)), float(goal[1][3][0] - float(dy/2))]


	# p = pc.Polytope(goal[0], goal[1])
	# xf = p.chebXc

	for num_segs in range(1, max_segs):
		xlist = []
		m = Model("xref")
		# m.setParam(GRB.Param.OutputFlag, 0)
		# m.getEnv().set(GRB_IntParam_OutputFlag, 0)
		for i in range(num_segs+1):
			xnew = m.addVars(2)
			xlist.append(xnew)

		# obj = (xf[0] - xlist[-1][0])*(xf[0] - xlist[-1][0]) + (xf[1] - xlist[-1][1])*(xf[1] - xlist[-1][1])
		# m.setObjective(obj, GRB.MINIMIZE)
		m.setParam(GRB.Param.OutputFlag, 0)

		# print(bloat_list)

		add_initial_constraints(m, xlist[0], Theta)
		add_final_constraints(m, xlist[-1], goal)
		add_avoidance_constraints(m, xlist, obs, num_segs, bloat_list)
		add_space_constraints(m, xlist, ([-11, 20], [-11, 20]))
		# add_length_constraints(m, xlist, l_min)

		m.write("test.lp")

		# m.update()
		m.optimize()

		# print(m.getVars())

		try:
			wypts = []
			# x_array = []
			# y_array = []
			for x in xlist:
				wypts.append((x[0].X,x[1].X))

				# x_array.append(x[0].X)
				# y_array.append(x[1].X)
			m.dispose()
			# print(x_array, y_array)
			# return x_array, y_array
			return wypts
		except:
			# print('No solution')
			m.dispose()
			# return None, None


if __name__ == '__main__':

	# find_xref(Theta, goal, obs, max_segs, l_min, bloat_list)


	bloat_factors = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
	theta_rect = ((-0.5, -0.5), (0.5, 0.5))
	goal_rect = ((9.5, 9.5), (10.5, 10.5))
	obs = [((5.494794, 7.9738050000000005, 0), (9.152394, 8.126195000000001, 2.5)), ((-9.152394, 7.9738050000000005, 0), (1.8204059999999997, 8.126195000000001, 2.5)), ((-9.152394000039061, -7.977187499999627, 0), (-9.00000399996094, 7.977187499999627, 2.5)), ((-9.144, -8.126195000000001, 0), (9.144, -7.9738050000000005, 2.5)), ((9.00000399996094, -7.977187499999627, 0), (9.152394000039061, 7.977187499999627, 2.5)), ((-9.000004, 2.66838, 0), (-5.920254, 2.7604550000000003, 2.5)), ((-5.957416330894509, 2.6683798308951308, 0), (-5.865341669105491, 2.7604551691048695, 2.5)), ((-5.957406925147777, 2.7604548309124057, 0), (-5.8653510748522235, 7.973805169087595, 2.5)), ((-0.09160636731983571, -1.599999632678815, 0), (0.1083943673198357, -1.400000367321185, 2.5))]

	print(find_xref(theta_rect, goal_rect, obs, 10, 0, bloat_factors))

	# add_initial_constraints(theta_rect)
	# add_final_constraints(theta_rect, 0.1)
	# A = np.array([[1.0, 0.0],
	#               [0.0, 1.0],
	#               [-1.0, -0.0],
	#               [-0.0, -1.0]])
	#
	# b = np.array([2.0, 1.0, 0.0, 0.0])
	#
	# p = pc.Polytope(A, b)
	#
	# print(p)
