# Gurobi xref planner
# Written by: Kristina Miller
from __future__ import division
from gurobipy import *
from math import *
import numpy as np
import polytope as pc
import matplotlib.pyplot as plt
from utils import plot_rectangles
from bloating import *
import pypoman as ppm
from max_error import max_lyapunov
from car_model import car_sim


def add_initial_constraints(model, x, Theta):
	# Make sure to start in Theta
	A, b = Theta
	dims = len(A[0])
	# Find center of initial set
	dx = b[0][0] + b[1][0]
	dy = b[2][0] + b[3][0]

	x0 = [float(b[1][0] - float(dx/2)), float(b[3][0] - float(dy/2))]
	# p = pc.Polytope(A, b)
	# x0 = p.chebXc
	model.addConstrs(x[i] == x0[i] for i in range(dims))
	return None

def add_final_constraints(model, x, goal):
	# Make sure to end in goal
	A, b = goal
	dims = len(A[0])
	# Find center of goal:
	dx = b[0][0] + b[1][0]
	dy = b[2][0] + b[3][0]

	xf = [float(b[1][0] - float(dx/2)), float(b[3][0] - float(dy/2))]
	# p = pc.Polytope(A, b)
	# xf = p.chebXc
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
			A, b = obs[ob_num]
			# print(b)
			add_obstacle_constraint(model, xlist, A, b, bloat_factor, ob_num, seg_num)
	return None

def add_space_constraints(model, xlist, limits):
	xlim, ylim = limits
	for x in xlist:
		model.addConstr(x[0] >= xlim[0])
		model.addConstr(x[1] >= ylim[0])
		model.addConstr(x[0] <= xlim[1])
		model.addConstr(x[1] <= ylim[1])

	return None

def add_length_constraints(model, xlist, l_min):
	# Make sure each line segment is long enough
	# Use big M method
	M = 1e6
	for seg_num in range(1, len(xlist)):
		beta = model.addVars(4, vtype=GRB.BINARY, name="beta_seg"+str(seg_num))
		x1 = xlist[seg_num-1][0]
		y1 = xlist[seg_num-1][1]
		x2 = xlist[seg_num][0]
		y2 = xlist[seg_num][1]

		tem1 = (x2 - x1) + (y2 - y1) - l_min
		tem2 = (x1 - x2) + (y2 - y1) - l_min
		tem3 = (x2 - x1) + (y1 - y2) - l_min
		tem4 = (x1 - x2) + (y1 - y2) - l_min

		model.addConstr(-tem1 + l_min <= M*(1 - beta[0]))
		model.addConstr(-tem2 + l_min <= M*(1 - beta[1]))
		model.addConstr(-tem3 + l_min <= M*(1 - beta[2]))
		model.addConstr(-tem4 + l_min <= M*(1 - beta[3]))
		model.addConstr(beta.sum() >= 1)
	return None

def find_xref(Theta, goal, obs, max_segs, l_min, bloat_list):
	# Find the entire path
	dims = len(Theta[0][0])
	# Find center of goal:
	dx = goal[1][0][0] + goal[1][1][0]
	dy = goal[1][2][0] + goal[1][3][0]

	xf = [float(goal[1][1][0] - float(dx/2)), float(goal[1][3][0] - float(dy/2))]


	# p = pc.Polytope(goal[0], goal[1])
	# xf = p.chebXc

	for num_segs in range(1, max_segs):
		xlist = []
		m = Model("xref")
		# m.setParam(GRB.Param.OutputFlag, 0)
		# m.getEnv().set(GRB_IntParam_OutputFlag, 0)
		for i in range(num_segs+1):
			xnew = m.addVars(dims)
			xlist.append(xnew)

		obj = (xf[0] - xlist[-1][0])*(xf[0] - xlist[-1][0]) + (xf[1] - xlist[-1][1])*(xf[1] - xlist[-1][1])
		m.setObjective(obj, GRB.MINIMIZE)
		m.setParam(GRB.Param.OutputFlag, 0)

		# print(bloat_list)

		add_initial_constraints(m, xlist[0], Theta)
		add_final_constraints(m, xlist[-1], goal)
		add_avoidance_constraints(m, xlist, obs, num_segs, bloat_list)
		add_space_constraints(m, xlist, ([0.1, 10], [0.1, 10]))
		add_length_constraints(m, xlist, l_min)

		m.write("test.lp")

		# m.update()
		m.optimize()

		# print(m.getVars())
		
		try:	
			x_array = []
			y_array = []
			for x in xlist:
				x_array.append(x[0].X)
				y_array.append(x[1].X)
			m.dispose()
			return x_array, y_array
		except:
			# print('No solution')
			m.dispose()
			# return None, None


if __name__ == '__main__':
	from partition_scenario import *

	# obs, Theta, goal, search_area = problem()
	A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
	b0 = np.array([[-0.5], [1], [-0.5], [1]])
	Theta = (A, b0)

	bf = np.array([[-5], [6], [-5], [6]])
	goal = (A, bf)

	b1 = np.array([[-3], [4], [0], [1]], dtype = np.float64)
	b2 = np.array([[0], [1], [-3], [4]], dtype = np.float64)
	b3 = np.array([[-3], [4], [-3], [4]], dtype = np.float64)

	b4 = np.array([[0], [6], [-6], [7]], dtype = np.float64)
	b5 = np.array([[0], [6], [1], [0]], dtype = np.float64)
	b6 = np.array([[-6], [7], [0], [6]], dtype = np.float64)
	b7 = np.array([[1], [0], [0], [6]], dtype = np.float64)
	obs = [(A, b1), (A, b2), (A, b3), (A, b4), (A, b5), (A, b6), (A, b7)]
	# b2 = np.array([[1], [0], [0], [4]], dtype = np.float64)
	# b3 = np.array([[-4], [5], [0], [4]], dtype = np.float64)
	# b4 = np.array([[0], [4], [-4], [5]], dtype = np.float64)
	# b5 = np.array([[0], [4], [1], [0]], dtype = np.float64)
	# obs = [(A, b1), (A, b2), (A, b3), (A, b4), A, b5]

	pt1 = [0, 0]
	pt2 = [5, 5]
	v = 2
	
	initial_set = [(0, 0, 0), max_lyapunov(Theta)]
	dis_time_step = 0.1
	sim_time_step = 0.01
	time_bound = np.linalg.norm(np.array(pt2) - np.array(pt1))/v
	# bloat_list = get_bloat_list(1.5, 10, car_error, car_interval, initial_set, dis_time_step, sim_time_step)
	ellips_list, time_list, rmax_list, rmin_list = get_reachtube(1, 10, car_error, car_interval, initial_set, dis_time_step, sim_time_step)
	bloat_list = get_bloat_list(ellips_list, time_list, rmax_list, rmin_list, initial_set[1], 1, 10, car_error, car_interval, dis_time_step, sim_time_step)
	bloat_list = [bound+0.1 for bound in bloat_list]
	print(bloat_list)
	find_xref(Theta, goal, obs, 10, 1, bloat_list)
	try:
		xref,yref = find_xref(Theta, goal, obs, 10, 1, bloat_list)

	except:
		print('no values!')

	# def rand_state():
	# 	x0 = (np.random.rand(1)-0.5)*0.5 + 0.5
	# 	y0 = (np.random.rand(1)-0.5)*0.5 + 0.5
	# 	theta0 = (np.random.rand(1)-0.5)*2*pi

	# 	q0 = [x0[0], y0[0], theta0[0]]
	# 	return(q0)
	# nodes = [[xref[i], yref[i]] for i in range(len(xref))]
	# q0 = rand_state()
	# u0 = [0, 0]
	# sim_trace = car_sim(q0, u0, nodes, sim_time_step)

	# x_sim = [trace[1] for trace in sim_trace]
	# y_sim = [trace[2] for trace in sim_trace]
	# fig, ax = plot_problem()


	# fig, ax = plt.subplots()
	# ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A, b0), color = 'b')
	# ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A, bf), color = 'g')
	# ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A, b1), color = 'r')
	# ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A, b2), color = 'r')
	# ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A, b3), color = 'r')
	
	# plt.plot(xref, yref, color = 'darkslategrey')
	# plt.scatter(xref, yref, color = 'k')
	# # plt.plot(x_sim, y_sim, color = 'deeppink', linestyle = '--')
	# ax.set_xticklabels([])
	# ax.set_yticklabels([])
	# plt.xlim(0, 6)
	# plt.ylim(0, 6)
	# plt.show()