#!/usr/bin/python3

import cv_bridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
import cv2
import rospy
import numpy as np
# import rosparam

class Image_to_map:
    def __init__(self):
        rospy.init_node('Map_ServEr', anonymous=True)
        rospy.sleep(1)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_callback)

        self.map_publihser = rospy.Publisher('/map', OccupancyGrid, queue_size=4)
        self.img_publihser = rospy.Publisher('/map_img', Image, queue_size=4)
        self.bridge = CvBridge()
        self.cords = []
        pass


    def img_callback(self, img_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg)

        ########### All Cv2 Actions begin here ##########
        # Reading image with OpenCv
        # and processing it for better
        # extraction of contours and
        # corodinates
        
        # thresholds 
        TAMin = rospy.get_param('thresholding_area_min', default=80)
        TAMax = rospy.get_param('thresholding_area_max', default=2000)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.bilateralFilter(gray, 13, 73, 95) 
        edged = cv2.Canny(blur, 47, 170)  
        thresh1 = cv2.adaptiveThreshold(edged, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21, 10) 

        # Fetching and drawing contours 
        contours, _ = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(img, contours, 0, (0, 0, 255), 2)

        # Creating white background image with
        # dimesions similar to input image
        # for mapping the coordinates and
        # drawing the coordinates
        h_img, w_img, _ = img.shape
        self.blank = np.zeros([h_img, w_img, 3], dtype = np.uint8)
        self.blank [:, :] = [0, 0, 0] 

        # Filtering through each contour and
        # then getting the coordinates to 
        # add to the above list
        area = set()
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.001 * cv2.arcLength(contour, True), True)

            # if len(approx) == 4:
            M = cv2.moments(contour)
            # if M['m00'] > 1000 and M['m00'] < 2500:
            if M['m00'] > TAMin and M['m00'] < TAMax:
            # if True:
                if M['m00'] == 0:
                    cX, cY = 0, 0,
                else:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                area.add(M['m00'])

                x, y, w, h = cv2.boundingRect(approx)
                
                # print([x, y, w, h])
                aspectRatio = float(w)/h
                if aspectRatio >= 0.8 and aspectRatio <= 1.20:
                    cv2.drawContours(self.blank, [approx], 0, (0, 0, 255), 2)
                    cv2.circle(self.blank, (cX, cY), 5, (255, 0, 0), 1)
                    if (cX, cY) not in self.cords:
                        self.cords.append([cX, cY])

        # cv2.imshow(f"Original", img)
        # cv2.imshow(f"Rects", self.blank)

        img_msg_updated = self.bridge.cv2_to_imgmsg(self.blank)
        self.img_publihser.publish(img_msg_updated)

        img_m = OccupancyGrid()
        img_m.header.frame_id = 'map'
        img_m.header.stamp = rospy.Time.now()

        gray = cv2.cvtColor(self.blank, cv2.COLOR_BGR2GRAY)
        h, w =  gray.shape
        img_m.info.height = h
        img_m.info.width = w
    

        img_m.info.resolution = 0.005

        img_m.info.origin.position.x = -1.6
        img_m.info.origin.position.y = -1.2

        temp_2 = gray.flatten()
        temp_2 = temp_2.tolist()
        # print(len(temp_2))
        img_m.data = temp_2
        self.map_publihser.publish(img_m)
        # print(np.array(cords))
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        print(self.cords[10])
        self.cords = []

if __name__ == '__main__':
    temp = Image_to_map()
    rospy.spin()
    # pass