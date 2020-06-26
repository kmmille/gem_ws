import math
import numpy as np

import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive

import math
import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive


class bicycleModel():

    def __init__(self):

        self.length = 1.88
        pt1 = ModelState()
        pt2 = ModelState()
        pt3 = ModelState()

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
        k1 = 4
        k2 = 15
        k3 = 4

        #give targetVel and targetAngVel
        targetVel = math.sqrt((targetPose.twist.linear.x*targetPose.twist.linear.x) + ((targetPose.twist.linear.y*targetPose.twist.linear.y)))
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
        newAckermannCmd.speed = (0.25 * math.cos(thetaError)) + (k1 * xError)
        newAckermannCmd.steering_angle_velocity = (targetAngVel) + ((02.5)*((k2*yError) + (k3*math.sin(thetaError))))


        #print(newAckermannCmd)
        return newAckermannCmd

    def setModelState(self, currState, targetState):

        #control = AckermannDrive()
        control = self.rearWheelFeedback(currState, targetState)
        values = self.rearWheelModel(control)

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

    def popNextPoint(self):
        if self.waypointList:
            return self.waypointList.pop(0)
        else:
            return None











# class bicycleModel():
#
#     def __init__(self):
#
#         self.waypointList = []
#         # data = np.loadtxt("laneWayPts.dat")
#         # i = 1
#         # data = [(0, 0), (10, 10)]
#         # for pt in data:
#         #     if i % 30:
#         #         wayPt = ModelState()
#         #         wayPt.pose.position.x = pt[0]
#         #         wayPt.pose.position.y = pt[1]
#         #         wayPt.twist.linear.x = .01
#         #         wayPt.twist.linear.y = .01
#         #         self.waypointList.append(wayPt)
#             # i += 1
#
#         # wayPt = ModelState()
#         # wayPt.pose.position.x = 2
#         # wayPt.pose.position.y = 8
#         # wayPt.twist.linear.x = .0001
#         # wayPt.twist.linear.y = .0001
#         # self.waypointList.append(wayPt)
#
#         self.waypointSub = rospy.Subscriber("/gem/waypoint", ModelState, self.__waypointHandler, queue_size=1)
#         self.waypointPub = rospy.Publisher("/gem/waypoint", ModelState, queue_size=1)
#
#
#         self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
#
#
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
#         k1 = 1
#         k2 = 1
#         k3 = 1
#
#         #give targetVel and targetAngVel
#         targetVel = math.sqrt((targetPose.twist.linear.x*targetPose.twist.linear.x) + ((targetPose.twist.linear.y*targetPose.twist.linear.y)))
#         targetAngVel = targetPose.twist.angular.z
#         #print(targetVel)
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
#         newAckermannCmd.speed = (targetVel * math.cos(thetaError)) + (k1 * xError)
#         newAckermannCmd.steering_angle_velocity = (targetAngVel) + ((targetVel)*((k2*yError) + (k3*math.sin(thetaError))))
#
#
#         #print(newAckermannCmd)
#         return newAckermannCmd
#
#     def setModelState(self, currState, targetState):
#
#         #control = AckermannDrive()
#         control = self.rearWheelFeedback(currState, targetState)
#         values = self.rearWheelModel(control)
#
#         newState = ModelState()
#         newState.model_name = 'polaris_ranger_ev'
#         newState.pose = currState.pose
#         newState.twist.linear.x =0#values[0]
#         newState.twist.linear.y = .5#values[1]
#         newState.twist.angular.z = values[2]
#         self.modelStatePub.publish(newState)
#
#
#
#
#
#     def quaternion_to_euler(self, x, y, z, w):
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
#
#     def __waypointHandler(self, data):
#         self.waypointList.append(data)
