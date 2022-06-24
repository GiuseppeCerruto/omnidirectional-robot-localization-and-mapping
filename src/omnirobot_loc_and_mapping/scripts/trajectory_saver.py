#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from omnirobot_loc_and_mapping.srv import TrajectoryService
import cv2 as cv
import os


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
        script_directory_path = os.path.dirname(__file__)
        map_path = os.path.join(script_directory_path, '../maps/map2.pgm')
        dest_path = os.path.join(script_directory_path, '../maps/map_with_trajectory.jpg')

        img = cv.imread(map_path)

        rospy.loginfo("NUMBER OF POSES: " + str(len(self.poses)) + " ------>")

        for i in range(len(self.poses)-1):
            curr_x = self.poses[i].pose.pose.position.x
            curr_y = self.poses[i].pose.pose.position.y
            next_x = self.poses[i+1].pose.pose.position.x
            next_y = self.poses[i+1].pose.pose.position.y
            cv.line(img,(int(curr_x), int(curr_y)),(int(next_x), int(next_y)), (0, 0, 255), 3)

            rospy.loginfo("\ncurr_x " + str(curr_x) + "\ncurr_y " + str(curr_y) + "\nnext_x " + str(next_x) + "\nnext_y " + str(next_y) + "\n")

        cv.imwrite(dest_path, img)

        # cv.imshow("Output", img);
        # cv.waitKey(0)

        return True
    

if __name__ == '__main__':
    AmclPosesSaver()
    rospy.spin()
