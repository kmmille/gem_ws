#!/usr/bin/env python
from __future__ import division
from math import *
import time
import sys
import pickle
import rospy
from gazebo_msgs.msg import  ModelState
from controller import bicycleModel
from planner import find_xref
# from a_star import a_star
# from hybrid_a_star import hybrid_a_star
import matplotlib.pyplot as plt
from simple_sensor import static_obstacles, dynamic_obstacles


# algorithm = sys.argv[1]
# environment = sys.argv[2]
d_s = 20
T_synth = 0.5
T_s = 2
actor_list = ['actor']
vrf = 3

if __name__ == "__main__":
    rospy.init_node("model_dynamics")

    model = bicycleModel()
    endList = 0

    targetState = ModelState()

    currState = model.getModelState()
    current_heading = model.quaternion_to_euler(currState.pose.orientation.x, \
        currState.pose.orientation.y, currState.pose.orientation.z,\
        currState.pose.orientation.w)

    min_x, max_x, min_y, max_y = -100, 100, -100, 100

    #### set the start of path planner ####
    sx = round(currState.pose.position.x ) # [m]
    sy = round(currState.pose.position.y)  # [m]
    stheta = round(current_heading[2] * 180 / pi)

    gx = round(30) #(50)  # [m]
    gy = round(10) #(80)  # [m]
    gtheta = round(180)

    grid_size = 1.0

    bloat_list = [1, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
    theta = [(sx-0.5, sy-0.5), (sx+0.5, sy+0.5)]
    goal = [(gx-0.5, gy-0.5), (gx+0.5, gy+0.5)]

    # t1 = time.time()
    obs = static_obstacles(d_s)
    # obs = obs + dynamic_obstacles(actor_list, d_s, T_s)
    # t2 = time.time()
    # print('obstime:', t2 - t1)
    # oblen = len(obs)

    # t1 = time.time()
    path = find_xref(theta, goal, obs, 10, 0, bloat_list)
    # path = [(0,0), (5, 10), (10, 20)]
    # t2 = time.time()
    # print('synthtime:', t2- t1)
    # print(oblen)
    print(path)

    statePath = []
    for coord_num in range(len(path[1:])):
        coord1 = path[coord_num]
        coord = coord1
        coord0 = path[coord_num-1]
        for i in range(1,11):
            newTargetState = ModelState()
            newTargetState.pose.position.x = (coord1[0]-coord0[0])*i/10 + coord0[0]
            newTargetState.pose.position.y = (coord1[1]-coord0[1])*i/10 + coord0[1]
            newTargetState.twist.linear.x = .25
            newTargetState.twist.linear.y = .25

            if len(coord) == 3:
                theta = np.atan2(coord1[1]- coord0[1], coord1[0]-coord0[0])
                [x, y, z, w] = model.euler_to_quaternion(0, 0, theta)
                newTargetState.pose.orientation.x = x
                newTargetState.pose.orientation.y = y
                newTargetState.pose.orientation.z = z
                newTargetState.pose.orientation.w = w

            statePath.append(newTargetState)

    model.addPlanedPath(statePath)

    start = time.time()
    t_s = start

    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state
        currState =  model.getModelState()
        if not currState.success:
            continue
        # targetState = model.popNextPoint()

        # if time.time() - t_s >= T_synth:
        #     t_s = time.time()
        #     sx = currState.pose.position.x  # [m]
        #     sy = currState.pose.position.y  # [m]
        #     stheta = round(current_heading[2] * 180 / pi)
        #
        #     # gx = round(10) #(50)  # [m]
        #     # gy = round(20) #(80)  # [m]
        #     # gtheta = round(180)
        #
        #     grid_size = 1.0
        #
        #     bloat_list = [1, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
        #     theta = [(sx-0.5, sy-0.5), (sx+0.5, sy+0.5)]
        #     goal = [(gx-0.5, gy-0.5), (gx+0.5, gy+0.5)]
        #
        #     obs = static_obstacles(d_s)
        #     # obs = obs + dynamic_obstacles(actor_list, d_s, T_s)
        #     # oblen = len(obs)
        #     # print(oblen)
        #
        #     path1 = find_xref(theta, goal, obs, 10, 0, bloat_list, path[1])
        #     # path = [(0, 0),(5, 10), (10, 20)]
        #     # print(oblen)
        #     print(path1)
        #     if path1 != None:
        #         path = path1
        #
        #         model.resetPath()
        #
        #         statePath = []
        #         for coord_num in range(len(path[1:])):
        #             coord1 = path[coord_num]
        #             coord = coord1
        #             coord0 = path[coord_num-1]
        #             for i in range(1,11):
        #                 newTargetState = ModelState()
        #                 newTargetState.pose.position.x = (coord1[0]-coord0[0])*i/10 + coord0[0]
        #                 newTargetState.pose.position.y = (coord1[1]-coord0[1])*i/10 + coord0[1]
        #                 newTargetState.twist.linear.x = .25
        #                 newTargetState.twist.linear.y = .25
        #
        #                 if len(coord) == 3:
        #                     theta = np.atan2(coord1[1]- coord0[1], coord1[0]-coord0[0])
        #                     [x, y, z, w] = model.euler_to_quaternion(0, 0, theta)
        #                     newTargetState.pose.orientation.x = x
        #                     newTargetState.pose.orientation.y = y
        #                     newTargetState.pose.orientation.z = z
        #                     newTargetState.pose.orientation.w = w
        #
        #                 statePath.append(newTargetState)
        #
        #         model.addPlanedPath(statePath)
        #
        #     print('new')
        # print(len(model.waypointList))
        if model.waypointList:
            targetState = model.waypointList[0]
        # if targetState == None:
        #     continue

        # print ("speed:", targetState.twist)
        currState =  model.getModelState()
        distToTargetX = abs(targetState.pose.position.x - currState.pose.position.x)
        distToTargetY = abs(targetState.pose.position.y - currState.pose.position.y)
        # print(targetState.pose.position.x,targetState.pose.position.y)
        if(distToTargetX < 1.5 and distToTargetY < 1.5):
            if not model.waypointList:
                newState = ModelState()
                newState.model_name = 'polaris_ranger_ev'
                newState.pose = currState.pose
                newState.twist.linear.x = 0
                newState.twist.linear.y = 0
                newState.twist.angular.z = 0
                model.modelStatePub.publish(newState)
                #only print time the first time waypontList is empty
                if(not endList):
                    endList = 1
                    end = time.time()
                    # print("Time taken:", end-start)
            else:
                if(endList):
                    start = time.time()
                    endList = 0
                targetState = model.waypointList.pop(0)
                markerState = ModelState()
                markerState.model_name = 'marker'
                markerState.pose = targetState.pose
                model.modelStatePub.publish(markerState)
        else:
            model.setModelState(currState, targetState)
            # print("Pose error:",distToTargetX,distToTargetY)
            markerState = ModelState()
            markerState.model_name = 'marker'
            markerState.pose = targetState.pose
            model.modelStatePub.publish(markerState)

    rospy.spin()

# from math import *
# import time
# import sys
# import pickle
# import rospy
# from gazebo_msgs.msg import  ModelState
# from controller import bicycleModel
# from planner import find_xref
# # from a_star import a_star
# # from hybrid_a_star import hybrid_a_star
# import matplotlib.pyplot as plt
# from simple_sensor import static_obstacles, dynamic_obstacles
#
#
# # algorithm = sys.argv[1]
# # environment = sys.argv[2]
# d_s = 20
# T_synth = 0.5
# T_s = 2
# actor_list = ['actor']
#
# if __name__ == "__main__":
#     rospy.init_node("model_dynamics")
#
#     model = bicycleModel()
#     endList = 0
#
#     targetState = ModelState()
#
#     currState = model.getModelState()
#     current_heading = model.quaternion_to_euler(currState.pose.orientation.x, \
#         currState.pose.orientation.y, currState.pose.orientation.z,\
#         currState.pose.orientation.w)
#
#     min_x, max_x, min_y, max_y = -100, 100, -100, 100
#
#     #### set the start of path planner ####
#     sx = currState.pose.position.x  # [m]
#     sy = currState.pose.position.y  # [m]
#     stheta = round(current_heading[2] * 180 / pi)
#
#     gx = round(30) #(50)  # [m]
#     gy = round(60) #(80)  # [m]
#     gtheta = round(0)
#
#     grid_size = 1.0
#
#     bloat_list = [1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2]
#     theta = [(sx-0.5, sy-0.5), (sx+0.5, sy+0.5)]
#     goal = [(gx-0.5, gy-0.5), (gx+0.5, gy+0.5)]
#
#     # t1 = time.time()
#     obs = static_obstacles(d_s)
#     # obs = obs + dynamic_obstacles(actor_list, d_s, T_s)
#     # t2 = time.time()
#     # print('obstime:', t2 - t1)
#     oblen = len(obs)
#
#     # t1 = time.time()
#     path = find_xref(theta, goal, obs, 100, 0, bloat_list)
#     # path = [(0,0), (5, 10), (10, 20)]
#     # t2 = time.time()
#     # print('synthtime:', t2- t1)
#     print(oblen)
#     print(path)
#
#     # path = [(0, 0), (70, 20)]
#
#     statePath = []
#     for coord1 in path:
#         # coord1 = path[coord_num]
#         # if coord_num != 0:
#         #     coord0 = path[coord_num - 1]
#         #     for i in range(1, 11):
#         #         newTargetState = ModelState()
#         #         newTargetState.pose.position.x = (coord1[0] - coord0[0])*i/10 + coord0[0]
#         #         newTargetState.pose.position.y = (coord1[1] - coord0[1])*i/10 + coord0[0]
#         #         newTargetState.twist.linear.x = .25
#         #         newTargetState.twist.linear.y = .25
#         #
#         #         if len(coord1) == 3:
#         #             [x, y, z, w] = model.euler_to_quaternion(0, 0, coord1[2]*math.pi/180)
#         #             newTargetState.pose.orientation.x = x
#         #             newTargetState.pose.orientation.y = y
#         #             newTargetState.pose.orientation.z = z
#         #             newTargetState.pose.orientation.w = w
#         #
#         #         statePath.append(newTargetState)
#         # else:
#         newTargetState = ModelState()
#         newTargetState.pose.position.x = coord1[0]
#         newTargetState.pose.position.y = coord1[1]
#         newTargetState.twist.linear.x = .25
#         newTargetState.twist.linear.y = .25
#
#         if len(coord1) == 3:
#             [x, y, z, w] = model.euler_to_quaternion(0, 0, coord1[2]*math.pi/180)
#             newTargetState.pose.orientation.x = x
#             newTargetState.pose.orientation.y = y
#             newTargetState.pose.orientation.z = z
#             newTargetState.pose.orientation.w = w
#
#         statePath.append(newTargetState)
#
#     model.addPlanedPath(statePath)
#
#     start = time.time()
#     t_s = start
#
#     rate = rospy.Rate(100)  # 100 Hz
#     while not rospy.is_shutdown():
#         rate.sleep()  # Wait a while before trying to get a new state
#         currState =  model.getModelState()
#         if not currState.success:
#             continue
#
#         # targetState = model.popNextPoint()
#         # print(targetState)
#
#         if time.time() - t_s >= T_synth:
#             t_s = time.time()
#             sx = currState.pose.position.x  # [m]
#             sy = currState.pose.position.y  # [m]
#             stheta = round(current_heading[2] * 180 / pi)
#         #
#         #     # gx = round(10) #(50)  # [m]
#         #     # gy = round(20) #(80)  # [m]
#         #     # gtheta = round(180)
#         #
#         #     grid_size = 1.0
#         #
#             bloat_list = [1, 1, 1, 1, 0.5, 0.5, 0.5, 0.5]
#             theta = [(sx-0.5, sy-0.5), (sx+0.5, sy+0.5)]
#             goal = [(gx-0.5, gy-0.5), (gx+0.5, gy+0.5)]
#         #
#             obs = static_obstacles(d_s)
#         #     # obs = obs + dynamic_obstacles(actor_list, d_s, T_s)
#         #     # oblen = len(obs)
#         #     print(oblen)
#         #
#             path = find_xref(theta, goal, obs, 10, 0, bloat_list, path[1])
#         #     # path = [(0, 0),(5, 10), (10, 20)]
#         #     # print(oblen)
#         #     print(path)
#         #     # path = [(0, 0), (70, 20)]
#         #
#         #     # model.resetPath()
#         #
#         #     statePath = []
#         #     # for coord_num in range(1, len(path)):
#         #     #     coord1 = path[coord_num]
#         #     #     coord0 = path[coord_num - 1]
#         #     #     for i in range(1, 11):
#         #     #         newTargetState = ModelState()
#         #     #         newTargetState.pose.position.x = (coord1[0] - coord0[0])*i/10 + coord0[0]
#         #     #         newTargetState.pose.position.y = (coord1[1] - coord0[1])*i/10 + coord0[0]
#         #     #         newTargetState.twist.linear.x = .25
#         #     #         newTargetState.twist.linear.y = .25
#         #     #
#         #     #         if len(coord1) == 3:
#         #     #             [x, y, z, w] = model.euler_to_quaternion(0, 0, coord1[2]*math.pi/180)
#         #     #             newTargetState.pose.orientation.x = x
#         #     #             newTargetState.pose.orientation.y = y
#         #     #             newTargetState.pose.orientation.z = z
#         #     #             newTargetState.pose.orientation.w = w
#         #     #
#         #     #         statePath.append(newTargetState)
#         #     #
#         #     # model.addPlanedPath(statePath)
#         #
#         #     print('new')
#
#         currState =  model.getModelState()
#         if model.waypointList:
#             targetState = model.waypointList[0]
#         # print(targetState.pose.position.x, targetState.pose.position.y)
#         # if targetState == None:
#         #     continue
#
#         # print ("speed:", targetState.twist)
#         distToTargetX = abs(targetState.pose.position.x - currState.pose.position.x)
#         distToTargetY = abs(targetState.pose.position.y - currState.pose.position.y)
#         # print(distToTargetX, distToTargetY)
#         # print(targetState.pose.position.x,targetState.pose.position.y)
#         if(distToTargetX < 2.0 and distToTargetY < 2.0):
#             if not model.waypointList:
#                 newState = ModelState()
#                 newState.model_name = 'polaris_ranger_ev'
#                 newState.pose = currState.pose
#                 newState.twist.linear.x = 0
#                 newState.twist.linear.y = 0
#                 newState.twist.angular.z = 0
#                 model.modelStatePub.publish(newState)
#                 #only print time the first time waypontList is empty
#                 if(not endList):
#                     endList = 1
#                     end = time.time()
#                     # print("Time taken:", end-start)
#             else:
#                 if(endList):
#                     start = time.time()
#                     endList = 0
#                 targetState = model.waypointList.pop(0)
#                 markerState = ModelState()
#                 markerState.model_name = 'marker'
#                 markerState.pose = targetState.pose
#                 model.modelStatePub.publish(markerState)
#         else:
#             model.setModelState(currState, targetState)
#             # print("Pose error:",distToTargetX,distToTargetY)
#             markerState = ModelState()
#             markerState.model_name = 'marker'
#             markerState.pose = targetState.pose
#             model.modelStatePub.publish(markerState)
#
#     rospy.spin()
