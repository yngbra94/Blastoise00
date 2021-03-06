#!/usr/bin/env python
# image_processing_node.py

"""
This node is responsible for detecting the nodes in the image from then converting the found pixel in to real world coordinate. 
when a beacon is found is there sent a stop sign to the command server, this is for making the robot stop so the transformation is as stable as possible 
The beacon is published to the topic /ecte477/beacons. The number of beacons to find is defined in the beacon_colours.yaml. 
The number of beacons left is published to the topic beacons_left. 

The beacons is found using the larges square of each colour with using HSV colour space. The beacon position is chosen as the center of the square.
the code also checks that the found boxes is above each other, and will ignore the beacon if the coordinates is not within a threshold. 
 


    Subscribed: camera/rgb/image_raw/compressed, camera/rgb/camera_info, camera/depth/image_raw, odom
    Publishes:  cmd/, beacons_left/ /ecte477/beacons

"""

import rospy
import cv2 
import numpy as np 
import imutils
from std_msgs.msg import String, Int16
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Point
import transformations as trans
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray




# Constants
DEPTH_SCALE = 1     # Depth is given in integer values on 1mm
#point in the image 
class XYPoint():
    def __init__(self, x, y):
        self.x = x
        self.y = y

class beacon_detector_node:
    def __init__(self):
        #duration for how long the robot will stop to find the correct position of the beacon 
        self.wait_for_sec = 3 
        self.bridge = CvBridge()
        self.colour_frame = None
        self.depth_frame = None
        self.K = None
        self.stop_flag = False # 
        self.wait_time = rospy.Time.now() # time when the robot got stop signal
        self.marker_array = MarkerArray()
        self.beacons_colour = rospy.get_param("~beacon_colours")
        self.beacons = rospy.get_param("~beacons")
        self.sub_colour_image = rospy.Subscriber('camera/rgb/image_raw/compressed', CompressedImage, self.callback_colour_image)
        self.subscriber_camera_info = rospy.Subscriber('camera/rgb/camera_info',CameraInfo,self.callback_camera_info)
        self.subscriber_camera_info = rospy.Subscriber('camera/depth/image_raw',Image,self.callback_depth_image)
        self.subscriber_odometry = rospy.Subscriber('odom', Odometry ,self.callback_odometry)
        self.publish_beacon_pos = rospy.Publisher('/ecte477/beacons', MarkerArray, queue_size=1)
        self.publisher_command = rospy.Publisher('cmd/',String, queue_size=1)
        self.publisher_beacons_left = rospy.Publisher('beacons_left/',Int16, queue_size=1)
      
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            #check that we have received an image and a depth image 
            if self.colour_frame != None and self.depth_frame !=None:
                self.loop()
            r.sleep()


    def loop(self):
        #copy the newest mat, this is needed because the callbacks run quicker then the loop. 
        colour_mat = self.colour_frame
        depth_mat = self.depth_frame
        odom_transform = self.transform_cam_to_world

        beaconPoints = []
        # find the centre pixel coordinate of the largest box of each colour
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
                if red_centre_point.y <green_centre_point.y:
                    beaconPoints.append([red_centre_point, "red", "green"]) 
                    cv2.putText(colour_mat ,"Red top and green bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                #if red is above green 
                if red_centre_point.y >green_centre_point.y:
                    beaconPoints.append([red_centre_point, "green", "red"]) 
                    cv2.putText(colour_mat ,"Green top and red bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
         # check colour what color that is found         
        if found_red and found_yellow: 
            #check that the center of both color is on the same x value
            if abs(red_centre_point.x -yellow_centre_point.x) <x_dir_accuracy:
                if red_centre_point.y <yellow_centre_point.y:
                    beaconPoints.append([red_centre_point, "red", "yellow"]) 
                    cv2.putText(colour_mat ,"Red top and yellow bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if red_centre_point.y >yellow_centre_point:
                    beaconPoints.append([red_centre_point, "yellow", "red"]) 
                    cv2.putText(colour_mat ,"Yellow top and red bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
        # check colour what color that is found        
        if found_red and found_blue: 
             #check that the center of both color is on the same x value
            if abs(red_centre_point.x -blue_centre_point.x) <x_dir_accuracy:
                if red_centre_point.y <blue_centre_point.y:
                    beaconPoints.append([red_centre_point, "red", "blue"]) 
                    cv2.putText(colour_mat ,"Red top and Blue bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if red_centre_point.y >blue_centre_point.y:
                    beaconPoints.append([red_centre_point, "blue", "red"]) 
                    cv2.putText(colour_mat ,"Blue top and red bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
        # check colour what color that is found        
        if found_green and found_blue:
             #check that the center of both color is on the same x value 
            if abs(blue_centre_point.x -green_centre_point.x) <x_dir_accuracy:
                if green_centre_point.y <blue_centre_point.y:
                    beaconPoints.append([green_centre_point, "green", "blue"]) 
                    cv2.putText(colour_mat ,"Green top and blue bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if green_centre_point.y >blue_centre_point.y:
                    beaconPoints.append([green_centre_point, "blue", "green"]) 
                    cv2.putText(colour_mat ,"Blue top and green bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
         # check colour what color that is found       
        if found_green and found_yellow: 
             #check that the center of both color is on the same x value
            if abs(green_centre_point.x -yellow_centre_point.x) <x_dir_accuracy:
                if green_centre_point.y <yellow_centre_point.y:
                    beaconPoints.append([green_centre_point, "green", "yellow"]) 
                    cv2.putText(colour_mat ,"Green top and yellow bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if green_centre_point.y >yellow_centre_point.y:
                    beaconPoints.append([green_centre_point, "yellow", "green"]) 
                    cv2.putText(colour_mat ,"Yellow top and green bottom beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
        # check colour what color that is found     
        if found_yellow and found_blue: 
             #check that the center of both color is on the same x value
            if abs(yellow_centre_point.x -blue_centre_point.x) <x_dir_accuracy:
                if yellow_centre_point.y <blue_centre_point.y:
                    beaconPoints.append([yellow_centre_point, "yellow", "blue"]) 
                    cv2.putText(colour_mat ,"yellow top and Blue beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                if yellow_centre_point.y >blue_centre_point.y:
                    beaconPoints.append([yellow_centre_point, "blue", "yellow"]) 
                    cv2.putText(colour_mat ,"Blue top and yellow beacon", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                
        #check that a beacon is found 
        if  len(beaconPoints) >0 :
            #for all beacons find the depth
            for i in beaconPoints: 
            
                for beacon in self.beacons:
                #check that the colour combination is not found before 
                    if beacon["top"] ==  i[1] and beacon["bottom"] == i[2]:
                        self.publisher_command.publish("stop")
                        print "stopping robot  "
                    #if the stop flag is false then first stop the robot 
                        if  self.stop_flag == False:
                            self.stop_flag = True  
                            self.wait_time = rospy.Time.now().to_sec()  
                            #check that the robot has stoped. estimates that it will take wait for sec secounds
                        if  rospy.Time.now().to_sec() - self.wait_time > self.wait_for_sec:
                            print "found beacon "
                            self.beacons.remove(beacon)
                            self.stop_flag = False  
                            depth = self.find_dept_at_pix(depth_mat, i[0], 5)
                            beacon_location = self.get_beacon_location(depth, i[0] , odom_transform)
                            self.publish_marker(beacon_location, i[1], beacon["id"]) 
                            self.publisher_command.publish("start")

        # if the beacon is lost while stopping, start to move again 
        elif self.stop_flag == True:
            print "stop flag true but lost the beacon"
            self.stop_flag= False
            self.publisher_command.publish("start")

        # publish how many beacons that is left 
        self.publisher_beacons_left.publish(len(self.beacons))
    


    

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




    def callback_colour_image(self , colour_image): 
        colour_np_arr = np.fromstring(colour_image.data , np.uint8)
        self.colour_frame  = cv2.imdecode(colour_np_arr, cv2.IMREAD_COLOR)

     
  
    def calculate_centre_of_Square(self, x, y, w, h):
        centrePoint  = XYPoint((x+w/2), (y+h/2))
        return centrePoint
         
    
    # check if the color is found and 
    # return the image frame with the bounding box added of the largest square.  
    # return the centre position of the largest contour
    #  return found_colour=  true if the colour is found 
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
        """
        Callback the get the odometry. 
        Used to make a transformation matrix between the camera to the world
        """
        self.pose = odometry.pose.pose
        # This transformation matrix is used to get the position and orientation of the camera. 
        # Its rotaion os first +90deg around the base_footprint Y axis and then -90deg around the base_footprint x axis
        rob3cam = np.array([np.array([0, 0, 1, 0.069]),np.array([-1, 0, 0, -0.047]),np.array([0, -1, 0, 0.117]),np.array([0, 0, 0, 1])])    
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
        """
        This method is used to get world coordinates of an object detected in the x,y image pixel with ad given depth. 
        :@param depth: The measure depth to the object. 
        :@param pix_pos: The pixel position in x and y given as a Point. 
        :@param Tcam_world: The transforamtion matrix between the camera an world. 
        """
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
        # Fund the [X, Y, Z] position of object based on the image frame 
        p3d = depth * np.matmul(np.linalg.inv(self.K), p_h)
        p3d_h = np.array ([[ p3d [0][0]] , [p3d [1][0]] , [p3d [2][0]] ,  [1]])
        # Find the X, Y and Z position to the object relative to the world
        p3d_w_h = np.matmul(Tcam_world, p3d_h)
        p3d_w = np.array ([[ p3d_w_h [0][0]/ p3d_w_h [3][0]] , [p3d_w_h [1][0]/ p3d_w_h [3][0]] , [p3d_w_h [2][0]/ p3d_w_h [3][0]]]) 

        beacon_pose = Pose()
        beacon_pose.position.x = p3d_w[0] 
        beacon_pose.position.y = p3d_w[1] 
        return beacon_pose

    #    
    def publish_marker(self, marker_pose, colour, ID):
        """
        Publishes a maker at the given maker pose, colors and ID 
        :@param marker_pose: the position to place the marker
        :@type marker_pose: Pose()
        :@param colour: Colour of marker. red, green, blue or yellow
        :@type colour: String. (e.g. "yellow")
        :@param ID: The ID of the ID of the marker. 
        :@type ID: int

        """
        marker = Marker()
        marker.header.seq = len(self.marker_array.markers) + 1 
        marker.id = ID
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.pose = marker_pose
        marker.scale = Vector3(0.2, 0.2, 0.2)
        marker.color.a = 1.0 # must be non-zero
        if colour == "red":
            marker.color.r = 1.0
        elif colour == "blue":
            marker.color.b = 1.0
        elif colour == "green":
            marker.color.g = 1.0
        elif colour == "yellow": 
            marker.color.r = 1.0
            marker.color.g = 1.0

        self.marker_array.markers.append(marker)
        self.publish_beacon_pos.publish(self.marker_array)


if __name__ == '__main__': 
    try:
        rospy.init_node('image_processing_node') 
        ipn = beacon_detector_node()
    except rospy.ROSInterruptException:
        pass