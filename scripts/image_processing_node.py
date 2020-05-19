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
        self.colour_frame=None
        self.depth_frame = None
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

    #def callback_colour_image(self , colour_image): 
     #   rospy.loginfo("[image processing] Received colour image")
        # Convert to NumPy and OpenCV formats
      #  colour_np_arr = np.fromstring(colour_image.data , np.uint8)
       # self.colour_frame = cv2.imdecode(colour_np_arr , cv2.IMREAD_COLOR)

    def callback_depth(self, depth_image):
        rospy.loginfo('[image processing] callback depth')
        self.depth_frame=self.bridge.imgmsg_to_cv2(depth_image,desired_encoding = "passthrough")
    def calculate_centre(self, x,y,w,h):
        

    def callback_colour_image(self , colour_image): 
        colour_np_arr = np.fromstring(colour_image.data , np.uint8)
        colour_frame = cv2.imdecode(colour_np_arr , cv2.IMREAD_COLOR)
        yellow=False
        red=False
        green=False
        blue=False

        if colour_frame != None:
            blurred = cv2.GaussianBlur(colour_frame ,(11,11),0)
            hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
            #red colour 
            red_lower = (0,227,108)#new
            red_upper = (22, 255, 206)
            # green 
            green_lower = (57,71,97)#new
            green_upper = (72, 255, 197)
            # yellow 
            yellow_lower = (25,249,128) #new 
            yellow_upper = (41, 255, 213)
            # blue 
            blue_lower = (106,124,81)#new
            blue_upper = (153, 255, 206)

            mask = cv2.inRange(hsv, red_lower, red_upper)
            mask = cv2.erode(mask,None,iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            #red
            red_contours = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            red_contours = imutils.grab_contours(red_contours)
            if len(red_contours) != 0:
        
                red=True   
                largest_contours_red =max(red_contours,key=cv2.contourArea)

                x,y,w,h =cv2.boundingRect(largest_contours_red)
                colour_frame  = cv2.rectangle(colour_frame ,(x,y),(x+w,y+h),(0,0,255),2)
                cv2.putText(colour_frame ,"Red", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
            #blue
            mask = cv2.inRange(hsv, blue_lower, blue_upper)
            mask = cv2.erode(mask,None,iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            blue_contours = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            blue_contours = imutils.grab_contours(blue_contours)
            if len(blue_contours) ==0:
                
                print "blue"
            else:
                blue=True   
                largest_contours_blue =max(blue_contours,key=cv2.contourArea)

                x,y,w,h =cv2.boundingRect(largest_contours_blue)
                colour_frame  = cv2.rectangle(colour_frame ,(x,y),(x+w,y+h),(0,0,255),2)
                cv2.putText(colour_frame ,"Blue", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                #

            #green
            mask = cv2.inRange(hsv, green_lower, green_upper)
            mask = cv2.erode(mask,None,iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            green_contours = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            green_contours = imutils.grab_contours(green_contours)
            if len(green_contours) ==0:
                print "green"
            else:
                green=True   
                largest_contours_green =max(green_contours,key=cv2.contourArea)

                x,y,w,h =cv2.boundingRect(largest_contours_green)
                colour_frame  = cv2.rectangle(colour_frame ,(x,y),(x+w,y+h),(0,0,255),2)
                cv2.putText(colour_frame ,"Green", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                #

            #yellow
            mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
            mask = cv2.erode(mask,None,iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            yellow_contours = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            yellow_contours = imutils.grab_contours(yellow_contours)
            if len(yellow_contours) ==0:
                print "yellow"
            else:
                yellow=True   
                largest_contours_yellow =max(yellow_contours,key=cv2.contourArea)

                yellow_x,yellow_y,w,h =cv2.boundingRect(largest_contours_yellow)
                colour_frame  = cv2.rectangle(colour_frame ,(yellow_x,yellow_y),(yellow_x+w,yellow_y+h),(0,0,255),2)
                cv2.putText(colour_frame ,"Yellow", (yellow_x,yellow_y), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
            # check colour and position 
            if red and green: 
                cv2.putText(colour_frame ,"Red and green beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
            if red and blue: 
                cv2.putText(colour_frame ,"Red and blue beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
            if blue and yellow: 
                cv2.putText(colour_frame ,"Blue and yellow beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
            if red and yellow: 
                cv2.putText(colour_frame ,"Red and yellow beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
            if blue and yellow: 
                cv2.putText(colour_frame ,"Blue and yellow beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
            self.colour_frame = colour_frame
            # Display the image
        
        #def get_color_sequence(self, BeamColor):


    def callback_camera_info(self,camera_info):
        self.K=np.array(camera_info.K).reshape([3,3])

    def callback_odometry(self,odometry):
        self.transform_cam_to_world = trans.msg_to_se3(odometry.pose.pose)

    # to convert the image position in to depth info is the code presented in tutorial 3.
    """
    def callback_depth(self, data):
        #print "callback_depth()" # For testing, can be removed
        try:
            # Convert image into OpenCV frame and numpy array
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32)

            # Find the depth of the centre pixel in metres
            center_idx = np.array(depth_array.shape) / 2
            print "center depth: {0}".format(DEPTH_SCALE * depth_array[center_idx[0], center_idx[1]])

            # Display the result
            cv2.imshow('Depth Image', depth_image)
            cv2.waitKey(2)
        except CvBridgeError, e:
            print e
    """

if __name__ == '__main__': 
    try:
        rospy.init_node('image_processing_node') 
        ipn = image_processing_node()
    except rospy.ROSInterruptException:
        pass