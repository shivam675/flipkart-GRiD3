from re import template
import cv_bridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
import cv2
import rospy
import numpy as np










if __name__ == '__main__':
    rospy.init_node('Temp')
    b = CvBridge()
    temp = cv2.imread('map_Moment.jpg')
    m = rospy.Publisher('/temp_topic', Image, queue_size=10)
    msg = b.cv2_to_imgmsg(temp)
    while not rospy.is_shutdown():
        m.publish(msg)
    # rospy.spin()
    # pass