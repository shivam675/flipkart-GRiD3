#!/usr/bin/python
import cv2
import cv_bridge
# import requests
# import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

#url = "http://192.168.1.5:8080/video"
#url = 'http://localhost:4747/mjpegfeed'
# url  = 'http://56.161.165.117:8080/video'
# url = 'http://192.168.43.147:4747/video'
url = "http://192.168.0.102:8080/video"

cap = cv2.VideoCapture(url)
bridge = CvBridge()
info = CameraInfo()


# info.

def video_cap():
    ip_pub_compressed = rospy.Publisher("/ip_cam/image_compressed", CompressedImage, queue_size=10)
    ip_pub_raw = rospy.Publisher("/ip_cam/image_raw", Image, queue_size=10)
    # ip_pub_caminfo = rospy.Publisher("/ip_cam/camera_info", CameraInfo, queue_size=10)
    rospy.loginfo("INIT DONE !")
    while not rospy.is_shutdown():
        rospy.loginfo_once("IMAGE PUBLISING RN")
        # width=1920
        # height=1080
        width=1280
        height=960
        ret, frame = cap.read()
        dim = (width, height)
        img = cv2.resize(frame, dim)
        # img = cv2.rotate(img, cv2.ROTATE_180)
        if img is not None:
            # cv2.imshow("Frame", img)
            # rospy.loginfo("first if cleared")
            img_msg_comp = bridge.cv2_to_compressed_imgmsg(img)
            img_msg_raw = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            ip_pub_compressed.publish(img_msg_comp)
            ip_pub_raw.publish(img_msg_raw)
    cv2.destroyAllWindows()



if __name__ == "__main__":
    rospy.init_node("ipcamera_python", anonymous=True)
    rospy.sleep(1)
    video_cap()
    # rospy.spin()