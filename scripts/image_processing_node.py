#!/usr/bin/env python
# image_processing_node.py

import rospy
import cv2 
import numpy as np 
import imutils
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from nav_msgs.msg import Odometry
import transformations as trans
from cv_bridge import CvBridge, CvBridgeError

# Constants
DEPTH_SCALE = 0.001     # Depth is given in integer values on 1mm


class image_processing_node:
    def __init__(self):
        self.bridge = CvBridge()
        self.colour_frame = None
        self.depth_frame = None
        self.beacons_colour = rospy.get_param("~beacon_colours")
        self.sub_colour_image = rospy.Subscriber('camera/rgb/image_raw/compressed', CompressedImage, self.callback_colour_image)
        self.subscriber_camera_info = rospy.Subscriber('camera/camera_info',CameraInfo,self.callback_camera_info)
        self.subscriber_camera_info = rospy.Subscriber('camera/depth/image_raw',Image,self.callback_depth)
        self.subscriber_odometry = rospy.Subscriber('odom', Odometry ,self.callback_odometry)
       
        self.beaconsLeft= 10
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()


    def loop(self):
        if self.colour_frame != None:
            cv2.imshow ('Colour Image', self.colour_frame )
            #cv2.imshow('masked Image', mask)
            cv2.waitKey(2)


    def callback_depth(self, depth_image):
        rospy.loginfo('[image processing] callback depth')
        self.depth_frame=self.bridge.imgmsg_to_cv2(depth_image,desired_encoding = "passthrough")

    def calculate_centre_of_Square(self, x, y, w, h):
        centre_xdir = (x+w/2)
        centre_ydir = (y+h/2)
        return centre_xdir, centre_ydir

    def callback_colour_image(self , colour_image): 
        colour_np_arr = np.fromstring(colour_image.data , np.uint8)
        colour_mat = cv2.imdecode(colour_np_arr, cv2.IMREAD_COLOR)
        yellow=False
        red=False
        green=False
        blue=False

        
        colour_mat, found_red, red_center_coord = self.get_colour_position(colour_mat, 'red')
        colour_mat, found_blue, blue_center_coord  = self.get_colour_position(colour_mat, 'blue')
        colour_mat, found_green, green_center_coord  = self.get_colour_position(colour_mat, 'green')
        colour_mat, found_yellow, yellow_center_coord  = self.get_colour_position(colour_mat, 'yellow')
        x_dir_accuracy =30
            # check colour what color that is found 
        if found_red and found_green:
            #check that the center of both color is on the same x value
            if abs(red_center_coord[0] -green_center_coord[0]) <x_dir_accuracy:
                #if red is above green 
                if red_center_coord[1] <green_center_coord[1] :
                    cv2.putText(colour_mat ,"Red top and green bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if red_center_coord[1] >green_center_coord[1]:
                    cv2.putText(colour_mat ,"Green top and red bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
        if found_red and found_yellow: 
            if abs(red_center_coord[0] -yellow_center_coord[0]) <x_dir_accuracy:
                if red_center_coord[1] <yellow_center_coord[1]:
                    cv2.putText(colour_mat ,"Red top and yellow bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if red_center_coord[1] >yellow_center_coord[1]:
                    cv2.putText(colour_mat ,"Yellow top and red bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
        if found_red and found_blue: 
            if abs(red_center_coord[0] -blue_center_coord[0]) <x_dir_accuracy:
                if red_center_coord[1] <blue_center_coord[1]:
                    cv2.putText(colour_mat ,"Red top and Blue bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if red_center_coord[1] >blue_center_coord[1]:
                    cv2.putText(colour_mat ,"Blue top and red bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
        if found_green and found_blue: 
            if abs(blue_center_coord[0] -green_center_coord[0]) <x_dir_accuracy:
                if green_center_coord[1] <blue_center_coord[1]:
                    cv2.putText(colour_mat ,"Green top and blue bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if green_center_coord[1] >blue_center_coord[1]:
                    cv2.putText(colour_mat ,"Blue top and green bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
        if found_green and found_yellow: 
            if abs(green_center_coord[0] -yellow_center_coord[0]) <x_dir_accuracy:
                if green_center_coord[1] <yellow_center_coord[1]:
                    cv2.putText(colour_mat ,"Green top and yellow bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if green_center_coord[1] >yellow_center_coord[1]:
                    cv2.putText(colour_mat ,"Yellow top and green bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
        if found_yellow and found_blue: 
            if abs(yellow_center_coord[0] -blue_center_coord[0]) <x_dir_accuracy:
                if yellow_center_coord[1] <blue_center_coord[1]:
                    cv2.putText(colour_mat ,"yellow top and Blue beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if yellow_center_coord[1] >blue_center_coord[1]:
                    cv2.putText(colour_mat ,"Blue top and yellow beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)

        #save the image to the node 
        self.colour_frame = colour_mat   
  
            
    
    # check if the color is found and return the position centre of the color
    #  
    def get_colour_position(self, image_frame, color):
        #Definition of the hsv values 
        lower_hsv_limit = (self.beacons_colour[color]["hueMin"], self.beacons_colour[color]["satMin"],  self.beacons_colour[color]["valMin"])
        upper_hsv_limit = (self.beacons_colour[color]["hueMax"], self.beacons_colour[color]["satMax"],  self.beacons_colour[color]["valMax"])
     
        # romove noise in the image

        blurred = cv2.GaussianBlur(image_frame ,(11,11),0)
        hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv_limit, upper_hsv_limit)
        mask = cv2.erode(mask,None,iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        found_colour =False
        contours = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        centerCoordinate = None
        # check that the colour is found 
        if len(contours) != 0:
            #find the largest cotour of the colour 
            largest_contours =max(contours,key=cv2.contourArea)
            found_colour=True
            x,y,w,h =cv2.boundingRect(largest_contours)
            centerCoordinate = self.calculate_centre_of_Square(x,y,w,h)
            image_frame  = cv2.rectangle(image_frame ,(x,y),(x+w,y+h),(0,0,255),2)
            cv2.putText(image_frame ,color, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)


        return image_frame, found_colour, centerCoordinate
        
     

    def callback_camera_info(self,camera_info):
        self.K=np.array(camera_info.K).reshape([3,3])

    def callback_odometry(self,odometry):
        self.transform_cam_to_world = trans.msg_to_se3(odometry.pose.pose)



if __name__ == '__main__': 
    try:
        rospy.init_node('image_processing_node') 
        ipn = image_processing_node()
    except rospy.ROSInterruptException:
        pass