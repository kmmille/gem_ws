#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from obstacleToRect import rect_list
from math import *
from numpy.linalg import norm
import numpy as np
import matplotlib.pyplot as plt

vped = 1

def listener(mdl_str):
    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    object_coordinates = model_coordinates(mdl_str, "")
    z = object_coordinates.pose.position.z
    y = object_coordinates.pose.position.y
    x = object_coordinates.pose.position.x

    x1 = object_coordinates.pose.orientation.x
    y1 = object_coordinates.pose.orientation.y
    z1 = object_coordinates.pose.orientation.z
    w = object_coordinates.pose.orientation.w

    roll, pitch, yaw = quaternion_to_euler(x1, y1, z1, w)

    # print(x, y, z)
    return (x, y, z),(roll, pitch, yaw)

def static_obstacles(d_s = 10):
    x_pos, y_pos, z_pos = listener("polaris_ranger_ev")[0]
    rect_local_static = []
    for rect in rect_list:
        x_min, y_min, z_min = rect[0]
        x_max, y_max, z_max = rect[1]

        if (norm(np.array([x_pos, y_pos]) - np.array([x_min, y_min]))<=d_s):
            # print('yes')
            rect_local_static.append(rect)
        elif (norm(np.array([x_pos, y_pos]) - np.array([x_min, y_max]))<=d_s):
            # print('yes')
            rect_local_static.append(rect)
        elif (norm(np.array([x_pos, y_pos]) - np.array([x_max, y_min]))<=d_s):
            # print('yes')
            rect_local_static.append(rect)
        elif (norm(np.array([x_pos, y_pos]) - np.array([x_max, y_max]))<=d_s):
            # print('yes')
            rect_local_static.append(rect)
        elif (abs(x_pos - x_min) <= d_s or abs(x_pos - x_max)<= d_s): # and (y_min <= y_pos <= y_max):
            # print('yes')
            rect_local_static.append(rect)
        elif (abs(y_pos - y_min) <= d_s or abs(y_pos - y_max)<= d_s): # and (x_min <= x_pos <= x_max):
            # print('yes')
            rect_local_static.append(rect)
        # else:
        #     print('no')

    return rect_local_static

def dynamic_obstacles(actor_list, d_s=11, T_s=2):
    x_pos, y_pos, z_pos = listener("polaris_ranger_ev")[0]
    rect_local_dyn = []
    for actor in actor_list:
        x_act, y_act, z_act = listener(actor)[0]
        roll, pitch, yaw = listener(actor)[1]
        # print(roll, pitch, yaw)
        #
        if norm(np.array([x_act, y_act] - np.array([x_pos, y_pos]))) <= d_s:
            print('detect dyn!')
            # Speed is 0.5 in the x direction
            if yaw < 0:
                # rect = ((x_act-0.25-(2*T_s), y_act-0.25, 0), (x_act+0.25, y_act+0.25, 0.25))
                if x_act-0.5-vped*T_s>0:
                    rect = ((x_act-0.5-(vped*T_s), y_act-0.5, 0), (x_act+0.5, y_act+0.5, 0.25))
                else:
                    t = (x_act-vped*T_s)/2
                    rect = ((0-0.5, y_act-0.5, 0), (max(x_act+0.5, 0.5+vped*t), y_act+0.5, 0.25))

            else:
                if x_act+0.5+vped*T_s<10:
                    rect = ((x_act-0.5, y_act-0.5, 0), (x_act+0.5+(vped*T_s), y_act+0.5, 0.25))
                else:
                    t = (x_act+vped*T_s - 10)/2
                    rect = ((min(x_act-0.5, 10 - 0.5 - t*vped), y_act-0.5, 0), (10+0.5, y_act+0.5, 0.25))
            rect_local_dyn.append(rect)
    return(rect_local_dyn)


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)
    return [roll, pitch, yaw]




# if __name__ == '__main__':
    # rospy.spin()

    #
    # colorlist = ['red', 'blue', 'green', 'black', 'purple', 'orange', 'pink']
    # # while True:
    # x, y, z = listener("polaris_ranger_ev")[0]
    # # listener("ground_plane")
    # obs = static_obstacles(20)
    # for obnum in range(len(obs)):
    #     ob = obs[obnum]
    #     xmin, ymin, zmin = ob[0]
    #     xmax, ymax, zmax = ob[1]
    #     plt.plot([xmin, xmin, xmax, xmax, xmin], [ymin, ymax, ymax, ymin, ymin], color = colorlist[obnum % 7])
    # plt.scatter([x], [y])
    # plt.show()
    #
    #
    # print((rect_list))
    #
    # # actor_list = ["actor"]
    # # print(dynamic_obstacles(actor_list))
