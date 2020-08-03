#!/usr/bin/env python
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

from gazebo_msgs.msg import  ModelState
from controller import bicycleModel

from math import *
from numpy.linalg import norm
import numpy as np

path = []
updatePath = False
def callback(data):
    global path, updatePath
    path = data.data
    path = path.reshape(len(path)/2, 2)
    updatePath = True
    print('new thing')

def listener():
    global path, updatePath
    # Get the current path
    rospy.init_node('listener')
    rospy.Subscriber('path', numpy_msg(Floats), callback)

    # r = rospy.Rate()

    # Set up bicycle model
    model = bicycleModel()
    endList = 0
    targetState = ModelState()

    # Set rate to 100 Hz
    r = rospy.Rate(80)
    while not rospy.is_shutdown():
        r.sleep()

        statePath = []
        # print(path)
        path = list(path)
        print(path)
        if updatePath:
            for coord in path[1:]:
                newTargetState = ModelState()
                newTargetState.pose.position.x = coord[0]
                newTargetState.pose.position.y = coord[1]
                newTargetState.twist.linear.x = 0
                newTargetState.twist.linear.y = 0

                if len(coord) == 3:
                    [x, y, z, w] = model.euler_to_quaternion(0, 0, coord[2]*math.pi/180)
                    newTargetState.pose.orientation.x = x
                    newTargetState.pose.orientation.y = y
                    newTargetState.pose.orientation.z = z
                    newTargetState.pose.orientation.w = w

                statePath.append(newTargetState)

            model.addPlanedPath(statePath)

        currState = model.getModelState()
        if not currState.success:
            continue
        if model.waypointList:
            targetState = model.waypointList[0]

        print(targetState)
        distToTargetX = abs(targetState.pose.position.x - currState.pose.position.x)
        distToTargetY = abs(targetState.pose.position.y - currState.pose.position.y)
        if(distToTargetX < 0.55 and distToTargetY < 0.55):
            if not model.waypointList:
                print('waypoint_list is empty')
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
                    end = rospy.get_time()
                    # print("Time taken:", end-start)
            else:
                if(endList):
                    start = rospy.get_time()
                    endList = 0
                # Need new waypoint
                print('need new waypoint')
                targetState = model.waypointList.pop(0)
                crd = path.pop(0)
                print(crd)
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


if __name__ == '__main__':
    # main()
    listener()
