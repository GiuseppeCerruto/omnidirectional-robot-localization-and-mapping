#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from omnirobot_loc_and_mapping.msg import AmclPoses
from omnirobot_loc_and_mapping.srv import TrajectoryService


class AmclPosesSaver:

    def __init__(self):

        rospy.init_node('trajectory_saver', anonymous=True)
        
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.subscriber_callback)
        self.poses = []

        self.service = rospy.Service('save_trajectory', TrajectoryService, self.service_callback)

        # self.pub = rospy.Publisher("amcl_pose_history", AmclPoses, queue_size=1)

    def subscriber_callback(self, data):
        self.poses.append(data)
        # self.pub.publish(self.poses)

    def service_callback(self, req):
        # print(self.poses[-1].header.seq)
        return self.poses[-1].header.seq
    

if __name__ == '__main__':
    AmclPosesSaver()
    rospy.spin()
