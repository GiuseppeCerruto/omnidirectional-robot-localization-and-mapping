#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
import os


cwd = os.getcwd()
pub = rospy.Publisher('map_smoothed', OccupancyGrid, queue_size=1, latch=True)


def callback(data):
    width = data.info.width
    height = data.info.height
    size =  (width , height)
    image = np.zeros(size)
    
    counter = 0
    #print (len(data.data))
    for p in data.data:
      image[counter%width][int(counter/width)] = p
      counter+=1
    
    
    
    kernel = np.ones((2,2),np.uint8)
    image = cv2.dilate(image,kernel,iterations = 1) # deletes obstacles of one pixel only (improbable the existence of them)
    image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel) # uniforms the area, in case the sensor misses an obstacle for a pixel
    custom_map = OccupancyGrid()

    custom_map.header = data.header
    custom_map.info = data.info
    custom_map.data =  np.transpose(image).flatten().astype (int)
    pub.publish(custom_map)

    print("Custom map published in topic /map_smoothed")


    

 
  

def listener():

    rospy.init_node('map_smoother', anonymous=False)

    rospy.Subscriber("/map", OccupancyGrid, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
