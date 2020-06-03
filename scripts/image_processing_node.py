#!/usr/bin/env python
# image_processing_node.py

import rospy
import cv2 
import numpy as np 
import imutils
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Point
import transformations as trans
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_matrix
import tf


# Constants
DEPTH_SCALE = 1     # Depth is given in integer values on 1mm
class XYPoint():
    def __init__(self, x, y):
        self.x = x
        self.y = y

class image_processing_node:
    def __init__(self):
        self.bridge = CvBridge()
        self.colour_frame = None
        self.depth_frame = None
        self.K = None
        self.marker_array = MarkerArray()
        self.beacons_colour = rospy.get_param("~beacon_colours")
        self.beacons = rospy.get_param("~beacons")
        self.sub_colour_image = rospy.Subscriber('camera/rgb/image_raw/compressed', CompressedImage, self.callback_colour_image)
        self.subscriber_camera_info = rospy.Subscriber('camera/rgb/camera_info',CameraInfo,self.callback_camera_info)
        self.subscriber_camera_info = rospy.Subscriber('camera/depth/image_raw',Image,self.callback_depth_image)
        self.subscriber_odometry = rospy.Subscriber('odom', Odometry ,self.callback_odometry)
        self.tflistener = tf.TransformListener()
        self.publish_beacon_pos = rospy.Publisher('/ecte477/beacons', MarkerArray, queue_size=1)
        self.beaconsLeft= 10
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.colour_frame != None and self.depth_frame !=None:
                self.loop()
            r.sleep()


    def loop(self):
        #copy the newest mat, this is needed because the callbacks run quicker then the loop. 
        colour_mat = self.colour_frame
        depth_mat = self.depth_frame
        odom_transform = self.transform_cam_to_world
        yellow=False
        red=False
        green=False
        blue=False
        beaconPoints = []
        colour_mat, found_red, red_centre_point = self.get_colour_position(colour_mat, 'red')
        colour_mat, found_blue, blue_centre_point  = self.get_colour_position(colour_mat, 'blue')
        colour_mat, found_green, green_centre_point  = self.get_colour_position(colour_mat, 'green')
        colour_mat, found_yellow, yellow_centre_point  = self.get_colour_position(colour_mat, 'yellow')
        x_dir_accuracy =30
            # check colour what color that is found 
        if found_red and found_green:
            #check that the center of both color is on the same x value
            if abs(red_centre_point.x -green_centre_point.x) <x_dir_accuracy:
                #if red is above green 
                if red_centre_point.y <green_centre_point.y :
                    cv2.putText(colour_mat ,"Red top and green bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if red_centre_point.y >green_centre_point.y:
                    cv2.putText(colour_mat ,"Green top and red bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                beaconPoints.append(red_centre_point)  
        if found_red and found_yellow: 
            if abs(red_centre_point.x -yellow_centre_point.x) <x_dir_accuracy:
                if red_centre_point.y <yellow_centre_point.y:
                    cv2.putText(colour_mat ,"Red top and yellow bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if red_centre_point.y >yellow_centre_point.y:
                    cv2.putText(colour_mat ,"Yellow top and red bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                beaconPoints.append(red_centre_point) 
        if found_red and found_blue: 
            if abs(red_centre_point.x -blue_centre_point.x) <x_dir_accuracy:
                if red_centre_point.y <blue_centre_point.y:
                    cv2.putText(colour_mat ,"Red top and Blue bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if red_centre_point.y >blue_centre_point.y:
                    cv2.putText(colour_mat ,"Blue top and red bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                beaconPoints.append(red_centre_point) 
        if found_green and found_blue: 
            if abs(blue_centre_point.x -green_centre_point.x) <x_dir_accuracy:
                if green_centre_point.y <blue_centre_point.y:
                    cv2.putText(colour_mat ,"Green top and blue bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if green_centre_point.y >blue_centre_point.y:
                    cv2.putText(colour_mat ,"Blue top and green bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                beaconPoints.append(green_centre_point) 
        if found_green and found_yellow: 
            if abs(green_centre_point.x -yellow_centre_point.x) <x_dir_accuracy:
                if green_centre_point.y <yellow_centre_point.y:
                    cv2.putText(colour_mat ,"Green top and yellow bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if green_centre_point.y >yellow_centre_point.y:
                    cv2.putText(colour_mat ,"Yellow top and green bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                beaconPoints.append(green_centre_point) 
        if found_yellow and found_blue: 
            if abs(yellow_centre_point.x -blue_centre_point.x) <x_dir_accuracy:
                if yellow_centre_point.y <blue_centre_point.y:
                    cv2.putText(colour_mat ,"yellow top and Blue beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if yellow_centre_point.y >blue_centre_point.y:
                    cv2.putText(colour_mat ,"Blue top and yellow beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                beaconPoints.append(yellow_centre_point)

        #check that a beacon is found 
        if  len(beaconPoints) >0 :
            #for all beacons find the depth
            for i in beaconPoints:   
                depth = self.find_dept_at_pix(depth_mat, i, 5)
             
                beacon_location = self.get_beacon_location(depth, i , odom_transform)
               

                self.callback_publishBeacons(beacon_location, self.beacons) 

     
        
            
       
        #cv2.imshow('Colour Image', colour_mat )
        #cv2.imshow('masked Image', mask)
        #cv2.waitKey(2)

    # Callback for a depth image 
    def callback_depth_image(self , depth_image):
        """
        Callback for depth image from camera. 
        :param depth_image: A depth image from the camera. 
        :type depth_image: Image
        """
        # rospy.loginfo('[Image  Processing] callback_depth')
        # Convert to NumPy and 
        depth_frame = self.bridge.imgmsg_to_cv2(depth_image , desired_encoding ="passthrough")
        self.depth_frame = np.array(depth_frame, dtype=np.float32)

    def calculate_centre_of_Square(self, x, y, w, h):
        centrePoint  = XYPoint((x+w/2), (y+h/2))
    
        return centrePoint

    def callback_colour_image(self , colour_image): 
        colour_np_arr = np.fromstring(colour_image.data , np.uint8)
        self.colour_frame  = cv2.imdecode(colour_np_arr, cv2.IMREAD_COLOR)

     
  
         
    
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
        centerPoint = None
        # check that the colour is found 
        if len(contours) != 0:
            #find the largest cotour of the colour 
            largest_contours =max(contours,key=cv2.contourArea)
            found_colour=True
            x,y,w,h =cv2.boundingRect(largest_contours)
            centerPoint = self.calculate_centre_of_Square(x,y,w,h)
            image_frame  = cv2.rectangle(image_frame ,(x,y),(x+w,y+h),(0,0,255),2)
            cv2.putText(image_frame ,color, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)


        return image_frame, found_colour, centerPoint
        
     

    def callback_camera_info(self,camera_info):
        self.K=np.array(camera_info.K).reshape([3,3])

    def callback_odometry(self,odometry):
        self.pose = odometry.pose.pose
        #rob3cam = np.array([np.array([0, -1, 0, 0.069]),np.array([0, 0, 1, -0.047]),np.array([1, 0, 0, 0.117]),np.array([0, 0, 0, 1])])
        rob3cam = np.array([np.array([0, 0, 1, 0.069]),np.array([-1, 0, 0, -0.047]),np.array([0, -1, 0, 0.117]),np.array([0, 0, 0, 1])])    


        #self.transform_cam_to_world = trans.msg_to_se3(odometry.pose.pose)
        #self.transform_cam_to_world = np.matmul(trans.msg_to_se3(odometry.pose.pose),rob3cam)

        self.transform_cam_to_world = np.matmul(trans.msg_to_se3(odometry.pose.pose),rob3cam)

    ### Color detection methods. 

    # Filter a color image with the provided HSV colour spectrum
    def HSV_color_mask(self, color_image, HSV_lower_values, HSV_upper_values):
        """
        Filter a color image with the provided HSV colour spectrum to generate a binary image 
        :param color_image: A color image 
        :type color_image: CompressedImage
        :param HSV_lower_values: The H, S and V lower range values (H, S, V). e.g. (0, 160, 99)
        :param HSV_upper_values: The H, S and V upper range values (H, S, V). e.g. (3, 255, 255)
        :return a binary image with the filtered color is white
        """
        # Blur image
        blurred = cv2.GaussianBlur(self.colour_frame, (11, 11), 0)

        # Convert to HSV 
        hsv_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) 
        # Filter the image such that only the red is left
        filtered_image = cv2.inRange(hsv_image, HSV_lower_values, HSV_upper_values)
        # Use closing to reomve noise in the image 
        filtered_image = cv2.erode(filtered_image, None, iterations = 2)
        filtered_image = cv2.dilate(filtered_image, None, iterations = 2)
        return filtered_image

    # Adding a boundingbox ta an image from a given mask. 
    def add_boudingbox_to_image(self, mask, image):
        """
        Draw a bounding bounding box around the mask on the image
        :param mask: Binary image of an object
        :param image: image to be drawn on. 
        """
        # Adding a contour to the image
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)


        if len(contours) != 0: 
            largest_contour = max(contours, key=cv2.contourArea)
            x, y , w, h = cv2.boundingRect(largest_contour)
            colour_frame  = cv2.rectangle(image ,(x,y),(x+w,y+h),(0,0,255),2)
            return colour_frame
        
        else: 
            return image

    ### Depth detection mathods. 

    # 
    def find_dept_at_pix(self, depth_image,  pointwh, mask_size):
        """
        Used to find the median depth around a single pixel with given mask 
        :param depth_image: Depth image. 
        :type depth_image: np.float32
        :param point: The x and y pixel coordinates in the image
        :type point: XYPoint (Internal Struct)
        :param mask_size: the square size in pixel number (mask x and y)
        :type mask_size: int
        """
        #change the direction and x and y as opencv uses (x,y) and array uses (y,x)
        point = XYPoint(pointwh.y,pointwh.x)
       
        x = point.x - int(mask_size/2) 
        y = point.y - int(mask_size/2)
        xSign = 1
        ySign = 1
        rows,cols = depth_image.shape
        # if the mask hits the image border. 
        if (x < 0): 
            x = point.x
        elif (point.x + int(mask_size/2) > rows): 
            x = point.x
            xSign = -1
        if (y < 0):
            y = point.y 
        elif (point.y + int(mask_size/2) > cols):
            y = point.y 
            ySign = -1

        # If the pixel value is in range
        if not np.isnan(depth_image[x, y]):

            depth_vals = []
            for i in range(mask_size):
                for j in range(mask_size):
                    depth_vals.append(depth_image[x + xSign*i, y + ySign*j])
            
            avg_depth = np.median(depth_vals) * DEPTH_SCALE

            return avg_depth

        # If the pixel depth is out of range return -1 indicating out of range. 

        else: 
            return -1


        # get beacon location. 
    def get_beacon_location(self, depth, pix_pos, Tcam_world):
        if self.K == None:
            rospy.loginfo("K missing")
            return
        if  self.transform_cam_to_world  == None: 
            rospy.loginfo("Camera trans missing")
            return
        if self.depth_frame  == None:
            rospy.loginfo(" depth frame missing")
            return

        # set pixel position 
        x = pix_pos.x
        y = pix_pos.y


        p_h = np.array ([[x], [y], [1]])
        
        p3d = depth * np.matmul(np.linalg.inv(self.K), p_h)
        p3d_h = np.array ([[ p3d [0][0]] , [p3d [1][0]] , [p3d [2][0]] ,  [1]])
        p3d_w_h = np.matmul(Tcam_world, p3d_h)
        p3d_w = np.array ([[ p3d_w_h [0][0]/ p3d_w_h [3][0]] , [p3d_w_h [1][0]/ p3d_w_h [3][0]] , [p3d_w_h [2][0]/ p3d_w_h [3][0]]]) 

        beacon_pose = Pose()
        beacon_pose.position.x = p3d_w[0] 
        beacon_pose.position.y = p3d_w[1] 

        print("K: \n{}".format(self.K))
        print("K-1: \n{}".format(np.linalg.inv(self.K)))
        print("T: \n{}".format(self.transform_cam_to_world))
        print("T-1: \n{}".format(np.linalg.inv(self.transform_cam_to_world)))
        print(" K-1 * [x y 1]: \n{}".format(np.matmul(np.linalg.inv(self.K), p_h)))
        print("P_h \n{}".format(p_h))
        print("Pcam: \n{}".format(p3d_h))
        print ("sPw pos: \n{}".format(p3d_w))
        print("Robot Pos: \n{}".format(self.pose.position))
        print ("Beacon pos: \n{}".format(beacon_pose.position))


        return beacon_pose
    
    def callback_publishBeacons(self, beacon_location, beacons): # just testing -> def publishBeacons(self, data):

        marker = Marker()
        marker.header.seq = len(self.marker_array.markers) + 1
        marker.header.frame_id = "/map"

        marker.action = marker.ADD

        marker.scale = Vector3(0.1, 0.1, 0.1)


        marker.color.a = 1.0 # must be non-zero
        marker.color.g = 1.0


        marker.pose = beacon_location


        self.marker_array.markers.append(marker)


        self.publish_beacon_pos.publish(self.marker_array)

        
        if len(self.marker_array.markers) == len(self.beacons):
            print("all beacons found. Now need to return home")

if __name__ == '__main__': 
    try:
        rospy.init_node('image_processing_node') 
        ipn = image_processing_node()
    except rospy.ROSInterruptException:
        pass