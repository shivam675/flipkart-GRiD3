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

bridge = CvBridge()
info = CameraInfo()


# info.

def video_cap(w,h,id):
    ip_pub_compressed = rospy.Publisher("/usb_cam/image_raw/compressed", CompressedImage, queue_size=10)
    ip_pub_raw = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=10)
    # ip_pub_caminfo = rospy.Publisher("/ip_cam/camera_info", CameraInfo, queue_size=10)
    rospy.loginfo("INIT DONE !")
    while not rospy.is_shutdown():
        rospy.loginfo_once("IMAGE PUBLISING RN")
        # width=1920
        # height=1080
        # width=1280
        # height=720
        ret, frame = cap.read()
        dim = (w, h)
        img = cv2.resize(frame, dim)
        # img = cv2.rotate(img, cv2.ROTATE_180)
        if img is not None:
            # cv2.imshow("Frame", img)
            # rospy.loginfo("first if cleared")
            img_msg_comp = bridge.cv2_to_compressed_imgmsg(img)
            img_msg_raw = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_msg_raw.header.frame_id = id
            img_msg_comp.header.frame_id = id
            ip_pub_compressed.publish(img_msg_comp)
            ip_pub_raw.publish(img_msg_raw)
    cv2.destroyAllWindows()



if __name__ == "__main__":
    rospy.init_node("ipcamera_python", anonymous=True)
    w = rospy.get_param('image_width', default=1280)
    h = rospy.get_param('image_height', default=720)
    id = rospy.get_param('camera_frame_id', default='camera_link')
    ip = rospy.get_param('ip_of_camera', default='192.168.0.35')
    port = rospy.get_param('port_of_camera', default=8080)
    
    url = 'http://' + ip + ':' + str(port) + '/video'
    cap = cv2.VideoCapture(url)

    
    rospy.sleep(1)
    video_cap(w,h,id)
    # rospy.spin()