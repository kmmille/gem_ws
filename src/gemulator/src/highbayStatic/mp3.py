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
from simple_sensor import static_obstacles


# algorithm = sys.argv[1]
# environment = sys.argv[2]

if __name__ == "__main__":
    rospy.init_node("model_dynamics")

    model = bicycleModel()
    endList = 0

    targetState = ModelState()

    currState = model.getModelState()
    current_heading = model.quaternion_to_euler(currState.pose.orientation.x, \
        currState.pose.orientation.y, currState.pose.orientation.z,\
        currState.pose.orientation.w)

    obstacle = []
    # if (environment == "highbay"):
    #     min_x, max_x, min_y, max_y = -15, 15, -15, 15
    #
    #     for i in range(3,8):
    #         for j in range(3,7):
    #             obstacle.append((j,i))
    #             obstacle.append((j,i))
    #             obstacle.append((j,i))
    #             obstacle.append((j,i))
    #
    #     gx = round(10)  # [m]
    #     gy = round(10)  # [m]
    #     gtheta = round(90)
    #     targetState.pose.position.x = gx
    #     targetState.pose.position.y = gy
    #     targetState.twist.linear.x = .25
    #     targetState.twist.linear.y = .25
    # elif (environment == "eceb"):
    #     min_x, max_x, min_y, max_y = -100, 100, -100, 100
    #
    #     with open('obstacle_list.data', 'rb') as filehandle:
    #     # read the data as binary data stream
    #         obstacle = pickle.load(filehandle)
    #
    #     for i in range(3,8):
    #         for j in range(3,7):
    #             obstacle.append((j,i))
    #             obstacle.append((j,i))
    #             obstacle.append((j,i))
    #             obstacle.append((j,i))
    #
    #     gx = round(30) #(50)  # [m]
    #     gy = round(0) #(80)  # [m]
    #     gtheta = round(180)
    #     targetState.pose.position.x = gx
    #     targetState.pose.position.y = gy
    #     targetState.twist.linear.x = .25
    #     targetState.twist.linear.y = .25
    #     [x, y, z, w] = model.euler_to_quaternion(0, 0, gtheta*math.pi/180)
    #     targetState.pose.orientation.x = x
    #     targetState.pose.orientation.y = y
    #     targetState.pose.orientation.z = z # 0.3826834
    #     targetState.pose.orientation.w = w #0.9238795

    min_x, max_x, min_y, max_y = -100, 100, -100, 100

    #### set the start of path planner ####
    sx = round(currState.pose.position.x)  # [m]
    sy = round(currState.pose.position.y)  # [m]
    stheta = round(current_heading[2] * 180 / pi)

    gx = round(10) #(50)  # [m]
    gy = round(9) #(80)  # [m]
    gtheta = round(180)

    grid_size = 1.0

    bloat_list = [1, 1, 1, 1, 0.5, 0.5, 0.5, 0.5]
    theta = [(sx-0.5, sy-0.5), (sx+0.5, sy+0.5)]
    goal = [(gx-0.5, gy-0.5), (gx+0.5, gy+0.5)]

    obs = static_obstacles(d_s = 10)
    # print(len(obs))


    path = find_xref(theta, goal, obs, 10, 0, bloat_list)
    print(path)

    # print((sx, sy, stheta), (gx,gy,gtheta))

    # print("Environment is set up, start path planner")
    # if (algorithm == "a_star"):
    #     simple_a_star = a_star(min_x, max_x, min_y, max_y, obstacle=obstacle, \
    #         resolution=grid_size, robot_size=1)
    #     path = simple_a_star.find_path((sx,sy), (gx,gy))
    # elif (algorithm == "hybrid_a_star"):
    #     hybrid_a_star = hybrid_a_star(min_x, max_x, min_y, max_y, obstacle=obstacle, \
    #         resolution=grid_size, vehicle_length=2)
    #     path = hybrid_a_star.find_path((sx,sy,stheta), (gx,gy,gtheta))
    # else:
    #     print ("Algorithm name is not valid. Please shut it down and restart.")
    # print ("cur_path", sx, sy, gx, gy)

    # ox, oy = [], []
    # for (x,y) in obstacle:
    #     ox.append(x)
    #     oy.append(y)
    #
    # rx, ry = [], []
    # for node in path:
    #     rx.append(node[0])
    #     ry.append(node[1])
    #
    # plt.plot(ox, oy, ".k")
    # plt.plot(sx, sy, "xr")
    # plt.plot(gx, gy, "xb")
    # plt.plot(rx, ry, "-r")
    # plt.grid(True)
    # plt.axis("equal")
    # plt.show()


    # path = [(sx, sy), (gx, gy)]
    statePath = []
    for coord in path:
        newTargetState = ModelState()
        newTargetState.pose.position.x = coord[0]
        newTargetState.pose.position.y = coord[1]
        newTargetState.twist.linear.x = .25
        newTargetState.twist.linear.y = .25

        if len(coord) == 3:
            [x, y, z, w] = model.euler_to_quaternion(0, 0, coord[2]*math.pi/180)
            newTargetState.pose.orientation.x = x
            newTargetState.pose.orientation.y = y
            newTargetState.pose.orientation.z = z
            newTargetState.pose.orientation.w = w

        statePath.append(newTargetState)

    model.addPlanedPath(statePath)

    start = time.time()

    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state
        currState =  model.getModelState()
        if not currState.success:
            continue
        # targetState = model.popNextPoint()
        if model.waypointList:
            targetState = model.waypointList[0]
        # if targetState == None:
        #     continue

        # print ("speed:", targetState.twist)
        distToTargetX = abs(targetState.pose.position.x - currState.pose.position.x)
        distToTargetY = abs(targetState.pose.position.y - currState.pose.position.y)
        #print(targetState.pose.position.x,targetState.pose.position.y)
        if(distToTargetX < 1.0 and distToTargetY < 1.0):
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
                    print("Time taken:", end-start)
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
