from __future__ import division
import polytope as pc
import numpy as np
from gurobipy import *
from math import *
import matplotlib.pyplot as plt

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
		model.addConstr(tem_constr2 <= M*(1 - alpha[edge]))
	model.addConstr(alpha.sum() >= 1)
	return None

# def add_obstacle_constraint(model, xlist, A, b, bloat_factor, ob_num, seg_num):
# 	# Avoid a polytope
# 	# print(A, b)
# 	edges = len(b)
# 	dims = len(A[0])
# 	# Use bigM method to avoid polytopes
# 	M = 1e6
# 	alpha = model.addVars(edges, vtype=GRB.BINARY, name="alpha_ob"+str(ob_num)+"_seg"+str(seg_num))
# 	print(alpha)
#
# 	x1 = xlist[seg_num]
# 	x2 = xlist[seg_num+1]
# 	for edge in range(edges):
# 		tem_constr1 = 0
# 		tem_constr2 = 0
# 		h_norm = np.linalg.norm(A[edge])
# 		# for i in range(dims):
# 		# 	print('A edge', A[edge][i])
# 		# 	tem_constr1 += A[edge][i]*x1[i]
# 		# 	tem_constr2 += A[edge][i]*x2[i]
# 		# print('b edge', b[edge])
# 		if edge == 0:
# 			tem_constr1 = x1[0]
# 			tem_constr2 = x2[0]
# 		elif edge == 1:
# 			tem_constr1 = x1[1]
# 			tem_constr2 = x2[1]
# 		elif edge == 2:
# 			tem_constr1 = -x1[0]
# 			tem_constr2 = -x2[0]
# 		else:
# 			tem_constr1 = -x1[1]
# 			tem_constr2 = -x2[1]
# 		tem_constr3 = -tem_constr1 + (b[edge] + h_norm*bloat_factor)
# 		tem_constr4 = -tem_constr2 + (b[edge] + h_norm*bloat_factor)
# 		model.addConstr(tem_constr3 <= M*(1 - alpha[edge]))
# 		model.addConstr(tem_constr4 <= M*(1 - alpha[edge]))
# 	model.addConstr(alpha.sum() >= 1)
# 	return None

def add_avoidance_constraints(model, xlist, obs, max_segs, bloat_factors):
	# Avoid each polytope in the list
	for seg_num in range(max_segs):
		# bloat_factor = bloat_factors[seg_num]
		bloat_factor = 1.2
		# print(bloat_factor)
		# bloat_factor = 10
		# print(len(obs))
		for ob_num in range(len(obs)):
			x_min, y_min, z_min = obs[ob_num][0]
			x_max, y_max, z_max = obs[ob_num][1]

			ob = pc.box2poly([[min(x_min, x_max), max(x_min, x_max)], [min(y_min, y_max), max(y_min, y_max)]])

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

def find_xref(Theta, goal, obs, max_segs, l_min, bloat_list, wypt = None):
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

		# print(bloat_list)

		add_initial_constraints(m, xlist[0], Theta)
		add_final_constraints(m, xlist[-1], goal)
		add_avoidance_constraints(m, xlist, obs, num_segs, bloat_list)
		add_space_constraints(m, xlist, ([0, 85], [0, 85]))
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
				wypts.append((float(x[0].X),float(x[1].X)))

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
	obs = [((5, 5, 0), (6, 6, 0))]

	pts = find_xref(theta_rect, goal_rect, obs, 10, 0, bloat_factors)
	# print(pts)
	x = [pt[0] for pt in pts]
	y = [pt[1] for pt in pts]
	plt.plot(x, y)

	for ob in obs:
		x1, y1, z1 = ob[0]
		x2, y2, z2 = ob[1]
		plt.plot([x1, x1, x2, x2, x1], [y1, y2, y2, y1, y1], color = 'red')

	plt.show()

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
