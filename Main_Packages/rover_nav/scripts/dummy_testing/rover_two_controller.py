#!/usr/bin/python3

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Bool


def path_callback(msg):
    rospy.sleep(5)
    status = Bool()
    status.data = True
    status_report.publish(status)
    print(len(msg.poses))


if __name__ == '__main__':
    rospy.init_node('path_two_dummy_controller', anonymous=True)
    rospy.Subscriber('robot2/exec_path', Path, callback=path_callback)
    status_report = rospy.Publisher('/waiting_for_response_two', Bool, queue_size=1)
    rospy.spin()