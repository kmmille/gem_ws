#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from obstacleToRect import rect_list


def listener(mdl_str):
    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    object_coordinates = model_coordinates(mdl_str, "")
    z = object_coordinates.pose.position.z
    y = object_coordinates.pose.position.y
    x = object_coordinates.pose.position.x

    print(x, y, z)

def static_obstacles():
    for rect in rect_list:
        print(rect)



if __name__ == '__main__':
    # while True:
        # listener("ground_plane")
        # listener("polaris_ranger_ev")
    static_obstacles()
