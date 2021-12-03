#!/usr/bin/python3
import rospy 
from sensor_msgs.msg import CompressedImage




def image_sub(msg):
    msg.header.frame_id = 'camera_link'
    # image_publisher.publish(msg)
    print(msg.header.frame_id)

if __name__ == "__main__":
    rospy.init_node('image_reversal', anonymous=True)
    image_publisher = rospy.Publisher('/rectified/image_raw/compressed', CompressedImage, queue_size=3)
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, callback=image_sub, queue_size=3)
    rospy.spin()