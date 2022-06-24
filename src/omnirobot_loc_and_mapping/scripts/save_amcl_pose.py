#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from omnirobot_loc_and_mapping.msg import AmclPoses


class AmclPosesSaver:

    def __init__(self):

        rospy.init_node('save_amcl_pose', anonymous=True)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback)

        self.pub = rospy.Publisher("amcl_pose_history", AmclPoses, queue_size=1)
        self.poses = []

    def callback(self, data):
        self.poses.append(data)
        self.pub.publish(self.poses)
    

if __name__ == '__main__':
    AmclPosesSaver()
    rospy.spin()
