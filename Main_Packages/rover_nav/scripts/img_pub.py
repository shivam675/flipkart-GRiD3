#!/usr/bin/python3



import cv_bridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
import cv2
import rospy
import numpy as np
import rospkg



rp = rospkg.RosPack()
path = rp.get_path('rover_nav')
map_path = path + '/scripts/round_2_map.png'



if __name__ == '__main__':
    rospy.init_node('Temp')
    b = CvBridge()
    # temp = cv2.imread('map_Moment.jpg')
    # temp = cv2.imread('material.png')
    temp = cv2.imread(map_path)
    m = rospy.Publisher('/temp_topic', Image, queue_size=10)
    msg = b.cv2_to_imgmsg(temp)
    while not rospy.is_shutdown():
        m.publish(msg)
    # rospy.spin()
    # pass