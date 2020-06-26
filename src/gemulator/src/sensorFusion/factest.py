#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState



model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
object_coordinates = model_coordinates("ground_plane", "ground_plane")
z = object_coordinates.pose.position.z
y = object_coordinates.pose.position.y
x = object_coordinates.pose.position.x

print(x, y, z)





# def callback(data):
#     rospy.loginfo(rospy.get_caller_id()+" I heard %s",data)
#
# def listener():
#     rospy.init_node('listener', anonymous=True)
#     rospy.Publisher("/rosout", ModelState, callback)
#     # rospy.Subscriber("/gazebo/model_states", ModelState, callback)
#     # rospy.Subscriber("/gem/waypoint", ModelState, __waypointHandler, queue_size=1)
#     rospy.spin()
#
# def __waypointHandler(self, data):
#     self.waypointList.append(data)
#
# if __name__ == '__main__':
#     listener()
