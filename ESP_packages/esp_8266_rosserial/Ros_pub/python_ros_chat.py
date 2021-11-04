#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talk():
    pub = rospy.Publisher('message', String, queue_size=10)
    rospy.init_node('talk')
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        freq = "8"
        rospy.loginfo(freq)
        pub.publish(freq)
        rate.sleep()

if __name__ == '__main__':
    try:
        talk()
    except rospy.ROSInterruptException:
        pass