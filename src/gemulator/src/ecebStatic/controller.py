import math
import numpy as np
import time

import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive

import math
import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive

import csv


class bicycleModel():
    def __init__(self):

        self.length = 1.88
        pt1 = ModelState()
        pt2 = ModelState()
        pt3 = ModelState()

        # self.t_s = time.time()

        pt1.pose.position.x = -10
        pt1.pose.position.y = -10
        pt1.twist.linear.x = .25
        pt1.twist.linear.y = .25
        #pt1.twist.angular = 0

        pt2.pose.position.x = 10
        pt2.pose.position.y = 10
        pt2.twist.linear.x = .25
        pt2.twist.linear.y = .25
        #pt2.twist.angular = 0

        pt3.pose.position.x = 0
        pt3.pose.position.y = 0
        pt1.twist.linear.x = .25
        pt1.twist.linear.y = .25
        #pt3.twist.angular = 0

        self.waypointList = []

        self.waypointSub = rospy.Subscriber("/gem/waypoint", ModelState, self.__waypointHandler, queue_size=1)
        self.waypointPub = rospy.Publisher("/gem/waypoint", ModelState, queue_size=1)

        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)

    def getModelState(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris_ranger_ev')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def rearWheelModel(self, ackermannCmd):
        currentModelState = self.getModelState()

        if not currentModelState.success:
            return

        velocity = ackermannCmd.speed

        euler = self.quaternion_to_euler(currentModelState.pose.orientation.x,
                                    currentModelState.pose.orientation.y,
                                    currentModelState.pose.orientation.z,
                                    currentModelState.pose.orientation.w)

        #bicycle model
        xVelocity = velocity * math.cos(euler[2])
        yVelocity = velocity * math.sin(euler[2])
        thetaVelocity = ackermannCmd.steering_angle_velocity

        #print("vel:", velocity, "headA:", euler[2], "thetaVel:", thetaVelocity)
        #print(xVelocity, yVelocity)
        return [xVelocity, yVelocity, thetaVelocity]

    def rearWheelFeedback(self, currentPose, targetPose):

        #Gain Values
        k1 = 0.2
        k2 = 1
        k3 = 1

        #give targetVel and targetAngVel
        # targetVel = math.sqrt((targetPose.twist.linear.x*targetPose.twist.linear.x) + ((targetPose.twist.linear.y*targetPose.twist.linear.y)))
        targetVel = 2
        targetAngVel = targetPose.twist.angular.z
        #print (targetVel, targetAngVel)
        currentEuler = self.quaternion_to_euler(currentPose.pose.orientation.x,
                                           currentPose.pose.orientation.y,
                                           currentPose.pose.orientation.z,
                                           currentPose.pose.orientation.w)

        targetEuler = self.quaternion_to_euler(targetPose.pose.orientation.x,
                                          targetPose.pose.orientation.y,
                                          targetPose.pose.orientation.z,
                                          targetPose.pose.orientation.w)

        #compute errors
        xError = ((targetPose.pose.position.x - currentPose.pose.position.x) * math.cos(currentEuler[2])) + ((targetPose.pose.position.y - currentPose.pose.position.y) * math.sin(currentEuler[2]))
        yError = ((targetPose.pose.position.x - currentPose.pose.position.x) * math.sin(currentEuler[2]) * -1) + ((targetPose.pose.position.y - currentPose.pose.position.y) * math.cos(currentEuler[2]))
        thetaError = targetEuler[2] - currentEuler[2]

        #print("Error:", xError, yError, thetaError)

        #give blank ackermannCmd
        newAckermannCmd = AckermannDrive()

        # if (targetVel * math.cos(thetaError)) + (k1 * xError) <= targetVel:
        newAckermannCmd.speed = (targetVel * math.cos(thetaError)) + (k1 * xError)
        # else:
        # newAckermannCmd.speed = targetVel
        newAckermannCmd.steering_angle_velocity = (targetAngVel) + ((targetVel)*((k2*yError) + (k3*math.sin(thetaError))))


        #print(newAckermannCmd)
        return newAckermannCmd

    def setModelState(self, currState, targetState):
        # T_synth = 2
        # if time.time() - self.t_s >= T_synth:
        #     print('must recompute controller')
        #     del self.waypointList[:]
        #     print(len(self.waypointList))
        #     self.t_s = time.time()
        #control = AckermannDrive()
        control = self.rearWheelFeedback(currState, targetState)
        values = self.rearWheelModel(control)

        # print(control.speed)
        # print(control.steering_angle_velocity)

        with open('inputs.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow([str(time.time()), str(control.speed), str(control.steering_angle_velocity)])

        # with open('inputs.csv','a') as fd:
        #     fd.write(str([control.speed, control.steering_angle_velocity]))

        newState = ModelState()
        newState.model_name = 'polaris_ranger_ev'
        newState.pose = currState.pose
        newState.twist.linear.x = values[0]
        newState.twist.linear.y = values[1]
        newState.twist.angular.z = values[2]
        self.modelStatePub.publish(newState)

    def quaternion_to_euler(self, x, y, z, w):
        x, y, z, w = float(x), float(y), float(z), float(w)

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def __waypointHandler(self, data):
        self.waypointList.append(data)

    #add a list of points in ModelState
    def addPlanedPath(self, path):
        self.waypointList = path + self.waypointList
    def resetPath(self):
        del self.waypointList[:]
        # print(len(self.waypointList))

    def popNextPoint(self):
        if self.waypointList:
            return self.waypointList.pop(0)
        else:
            return None

# import math
# import numpy as np
# import time
#
# import rospy
# from gazebo_msgs.srv import GetModelState
# from gazebo_msgs.msg import ModelState
# from ackermann_msgs.msg import AckermannDrive
#
# import math
# import rospy
# from gazebo_msgs.srv import GetModelState
# from gazebo_msgs.msg import ModelState
# from ackermann_msgs.msg import AckermannDrive
#
#
# class bicycleModel():
#     def __init__(self):
#
#         self.length = 1.88
#         pt1 = ModelState()
#         pt2 = ModelState()
#         pt3 = ModelState()
#
#         # self.t_s = time.time()
#
#         pt1.pose.position.x = -10
#         pt1.pose.position.y = -10
#         pt1.twist.linear.x = .25
#         pt1.twist.linear.y = .25
#         #pt1.twist.angular = 0
#
#         pt2.pose.position.x = 10
#         pt2.pose.position.y = 10
#         pt2.twist.linear.x = .25
#         pt2.twist.linear.y = .25
#         #pt2.twist.angular = 0
#
#         pt3.pose.position.x = 0
#         pt3.pose.position.y = 0
#         pt1.twist.linear.x = .25
#         pt1.twist.linear.y = .25
#         #pt3.twist.angular = 0
#
#         self.waypointList = []
#
#         self.waypointSub = rospy.Subscriber("/gem/waypoint", ModelState, self.__waypointHandler, queue_size=1)
#         self.waypointPub = rospy.Publisher("/gem/waypoint", ModelState, queue_size=1)
#
#         self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
#
#     def getModelState(self):
#         rospy.wait_for_service('/gazebo/get_model_state')
#         try:
#             serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
#             modelState = serviceResponse(model_name='polaris_ranger_ev')
#         except rospy.ServiceException as exc:
#             rospy.loginfo("Service did not process request: "+str(exc))
#         return modelState
#
#     def rearWheelModel(self, ackermannCmd):
#         currentModelState = self.getModelState()
#
#         if not currentModelState.success:
#             return
#
#         velocity = ackermannCmd.speed
#
#         euler = self.quaternion_to_euler(currentModelState.pose.orientation.x,
#                                     currentModelState.pose.orientation.y,
#                                     currentModelState.pose.orientation.z,
#                                     currentModelState.pose.orientation.w)
#
#         #bicycle model
#         xVelocity = velocity * math.cos(euler[2])
#         yVelocity = velocity * math.sin(euler[2])
#         thetaVelocity = ackermannCmd.steering_angle_velocity
#
#         #print("vel:", velocity, "headA:", euler[2], "thetaVel:", thetaVelocity)
#         #print(xVelocity, yVelocity)
#         return [xVelocity, yVelocity, thetaVelocity]
#
#     def rearWheelFeedback(self, currentPose, targetPose):
#
#         #Gain Values
#         k1 = 2
#         k2 = 2
#         k3 = 2
#
#         #give targetVel and targetAngVel
#         # targetVel = math.sqrt((targetPose.twist.linear.x*targetPose.twist.linear.x) + ((targetPose.twist.linear.y*targetPose.twist.linear.y)))
#         targetVel = 3
#         # targetAngVel = 0
#         targetAngVel = targetPose.twist.angular.z
#         #print (targetVel, targetAngVel)
#         currentEuler = self.quaternion_to_euler(currentPose.pose.orientation.x,
#                                            currentPose.pose.orientation.y,
#                                            currentPose.pose.orientation.z,
#                                            currentPose.pose.orientation.w)
#
#         targetEuler = self.quaternion_to_euler(targetPose.pose.orientation.x,
#                                           targetPose.pose.orientation.y,
#                                           targetPose.pose.orientation.z,
#                                           targetPose.pose.orientation.w)
#
#         #compute errors
#         xError = ((targetPose.pose.position.x - currentPose.pose.position.x) * math.cos(currentEuler[2])) + ((targetPose.pose.position.y - currentPose.pose.position.y) * math.sin(currentEuler[2]))
#         yError = ((targetPose.pose.position.x - currentPose.pose.position.x) * math.sin(currentEuler[2]) * -1) + ((targetPose.pose.position.y - currentPose.pose.position.y) * math.cos(currentEuler[2]))
#         thetaError = targetEuler[2] - currentEuler[2]
#
#         #print("Error:", xError, yError, thetaError)
#
#         #give blank ackermannCmd
#         newAckermannCmd = AckermannDrive()
#
#         # if (targetVel * math.cos(thetaError)) + (k1 * xError) <= targetVel:
#         newAckermannCmd.speed = (targetVel * math.cos(thetaError)) + (k1 * xError)
#         # else:
#         newAckermannCmd.speed = targetVel
#         newAckermannCmd.steering_angle_velocity = (targetAngVel) + ((targetVel)*((k2*yError) + (k3*math.sin(thetaError))))
#         # if abs((targetAngVel) + ((targetVel)*((k2*yError) + (k3*math.sin(thetaError))))) >= np.pi/4:
#         #     newAckermannCmd.steering_angle_velocity = np.sign((targetAngVel) + ((targetVel)*((k2*yError) + (k3*math.sin(thetaError)))))*np.pi/4
#
#
#         #print(newAckermannCmd)
#         return newAckermannCmd
#
#     def setModelState(self, currState, targetState):
#         # T_synth = 2
#         # if time.time() - self.t_s >= T_synth:
#         #     print('must recompute controller')
#         #     del self.waypointList[:]
#         #     print(len(self.waypointList))
#         #     self.t_s = time.time()
#         #control = AckermannDrive()
#         control = self.rearWheelFeedback(currState, targetState)
#         values = self.rearWheelModel(control)
#
#         newState = ModelState()
#         newState.model_name = 'polaris_ranger_ev'
#         newState.pose = currState.pose
#         newState.twist.linear.x = values[0]
#         newState.twist.linear.y = values[1]
#         newState.twist.angular.z = values[2]
#         self.modelStatePub.publish(newState)
#
#     def quaternion_to_euler(self, x, y, z, w):
#         x, y, z, w = float(x), float(y), float(z), float(w)
#
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + y * y)
#         roll = math.atan2(t0, t1)
#         t2 = +2.0 * (w * y - z * x)
#         t2 = +1.0 if t2 > +1.0 else t2
#         t2 = -1.0 if t2 < -1.0 else t2
#         pitch = math.asin(t2)
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         yaw = math.atan2(t3, t4)
#         return [roll, pitch, yaw]
#
#     def euler_to_quaternion(self, roll, pitch, yaw):
#         qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
#         qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
#         qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
#         qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
#         return [qx, qy, qz, qw]
#
#     def __waypointHandler(self, data):
#         self.waypointList.append(data)
#
#     #add a list of points in ModelState
#     def addPlanedPath(self, path):
#         self.waypointList = path + self.waypointList
#     def resetPath(self):
#         del self.waypointList[:]
#
#     def popNextPoint(self):
#         if self.waypointList:
#             return self.waypointList.pop(0)
#         else:
#             return None
