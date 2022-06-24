#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from omnirobot_loc_and_mapping.srv import TrajectoryService
from nav_msgs.msg import MapMetaData
import cv2 as cv
import os


class AmclPosesSaver:

    def __init__(self):

        rospy.init_node('trajectory_saver', anonymous=True)

        rospy.Subscriber("/map_metadata", MapMetaData, self.metadata_reader_callback)
        
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_poe_reader_callback)
        self.poses = []

        self.service = rospy.Service('save_trajectory', TrajectoryService, self.service_callback)


    def metadata_reader_callback(self, data):
        self.origin_x = 0.0 - float(data.origin.position.x)
        self.origin_y = 0.0 - float(data.origin.position.y)
        self.resolution = float(data.resolution)


    def amcl_poe_reader_callback(self, data):
        self.poses.append(data)


    def service_callback(self, req):    
        script_directory_path = os.path.dirname(__file__)
        map_path = os.path.join(script_directory_path, '../maps/map2.pgm')
        dest_path = os.path.join(script_directory_path, '../maps/map_with_trajectory.jpg')

        img = cv.imread(map_path)

        for i in range(len(self.poses)-1):
            curr_x_px = (self.origin_x / self.resolution) + (self.poses[i].pose.pose.position.x / self.resolution)
            curr_y_px = (self.origin_y / self.resolution) - (self.poses[i].pose.pose.position.y / self.resolution)
            next_x_px = (self.origin_x / self.resolution) + (self.poses[i+1].pose.pose.position.x / self.resolution)
            next_y_px = (self.origin_y / self.resolution) - (self.poses[i+1].pose.pose.position.y / self.resolution)

            cv.line(img,(int(curr_x_px), int(curr_y_px)),(int(next_x_px), int(next_y_px)), (0, 0, 255), 1)

        cv.imwrite(dest_path, img)

        return True
    

if __name__ == '__main__':
    AmclPosesSaver()
    rospy.spin()
