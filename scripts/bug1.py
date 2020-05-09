#!/usr/bin/env python

import rospy
import numpy as np 
import tf
import math
from enum import Enum
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse
from movement_starter.srv import SetPoint, SetPointResponse

# Constants
YAW_PRECISION = math.pi / 90    # +/- 2 degrees in radians
DIST_PRECISION = 0.1            # metres
MAX_SIDE_LIMIT = 0.50           # This furthest distance we 'see' the wall to the side
MAX_APPROACH_DIST = 0.80        # The closest we want to get to a wall from the front

class Bug1State(Enum):
    GO_TO_POINT = 1
    CIRCUMNAVIGATE = 2
    GO_TO_CLOSEST =3
    DONE = 4

class bug1_node:
    def __init__(self):
         # Node State
        self.active = False
        self.target_point = Point()
        self.state = Bug1State.GO_TO_POINT # Placeholder initial state
        self.circumnavigate_start_point = None  
        self.circumnavigate_closest_point = None
        self.circumnavigate_state_counter = 0
        # Robot State
        self.position = Point()
        self.yaw = 0
        self.regions = None
        # Services
        self.service_start_stop = rospy.Service('~start_stop', SetBool , self.callback_state)      # Service this node offers
        self.service_set_point = rospy.Service('~set_point', SetPoint , self.callback_set_point)   # Service this node offers
        self.start_go_to_point = rospy.ServiceProxy('/go_to_point_node/start_stop', SetBool)
        self.start_wall_follower = rospy.ServiceProxy('/wall_follower_node/start_stop', SetBool)
        self.go_to_point_set_point = rospy.ServiceProxy('/go_to_point_node/set_point', SetPoint)
        rospy.wait_for_service('/go_to_point_node/start_stop')
        rospy.wait_for_service('/wall_follower_node/start_stop')
        rospy.wait_for_service('/go_to_point_node/set_point')
        # Subscribers
        self.subscriber_laser_scan = rospy.Subscriber('scan/', LaserScan, self.callback_laser_scan)
        self.subscriber_odometry = rospy.Subscriber('odom/', Odometry, self.callback_odometry)
        
    
        # Loop
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    
    def normalise_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle
    # eucleadean distance 
    def distance_points(self, point1, point2):
        dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
        return dist

    def callback_state(self, start_stop):
        if start_stop.data==True and not self.active:
            self.active = True
            rospy.logdebug('[Bug 1] Starting')
            if self.state == Bug1State.GO_TO_POINT:
                rospy.logdebug('[Bug 1] Starting Go To Point')
                resp = self.start_go_to_point(True)
                resp = self.start_wall_follower(False)
            elif self.state == Bug1State.CIRCUMNAVIGATE or self.state ==Bug1State.GO_TO_CLOSEST:
                rospy.logdebug('[Bug 1] Starting Wall Follow')
                resp = self.start_go_to_point(False)
                resp = self.start_wall_follower(True)
            return SetBoolResponse(True, 'Starting Bug 1')
        elif start_stop.data==False and self.active:
            self.active = False
            rospy.logdebug('[Bug 1] Stopping')
            resp = self.start_go_to_point(False)
            resp = self.start_wall_follower(False)
            return SetBoolResponse(True, 'Stopping Bug 1')
        else:
            if start_stop.data==True:
                return SetBoolResponse(False, 'Already Doing Bug 1')
            else:
                return SetBoolResponse(False, 'Already Stopped')

    def change_state(self, state):
        if self.state == state:
            return
        if state == Bug1State.GO_TO_POINT:
            rospy.logdebug('[Bug 1] Swapping to Go To Point')
            self.state = state
            resp = self.start_go_to_point(True)
            resp = self.start_wall_follower(False)
        elif state == Bug1State.CIRCUMNAVIGATE:
            rospy.logdebug('[Bug 1] Swapping to Wall Follow')
            self.state = state
            resp = self.start_go_to_point(False)
            resp = self.start_wall_follower(True)
        elif state == Bug1State.GO_TO_CLOSEST:
            rospy.logdebug('[Bug 1] go to closest point')
            self.state = state
            resp = self.start_go_to_point(False)
            resp = self.start_wall_follower(True) 
        elif state == Bug1State.DONE:
            rospy.logdebug('[Bug 1] done')
            self.state = state
            resp = self.start_go_to_point(False)
            resp = self.start_wall_follower(False)
            self.pub_done_move_base_fake(True) # Tell move_base_fake that bug is done

    # Tell move_base_fake that bug is done 
    def pub_done_move_base_fake(self,is_done):
        move_base_fake_service = '/move_base_fake/is_bug_done/'
        rospy.wait_for_service(move_base_fake_service)
        try:
            # Create a service to tell move_base_fake that bug is done
            pub_bug_is_done = rospy.ServiceProxy(move_base_fake_service,SetBool)
            pub_bug_is_done(is_done)
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    def callback_set_point(self, point_message):
        self.target_point = point_message.point
        resp = self.go_to_point_set_point(point_message.point)
        self.state = Bug1State.GO_TO_POINT
        rospy.logdebug('[Bug 1] Swapping to Go To Point')
        return SetPointResponse(True, 'Target Point Set')

    def callback_laser_scan(self, scan):
        self.regions = {
        'right':  min(min(scan.ranges[270:305]), 3.5),
        'fright': min(min(scan.ranges[306:341]), 3.5),
        'front':  min(min(min(scan.ranges[342:359]), min(scan.ranges[0:17])), 3.5),
        'fleft':  min(min(scan.ranges[18:53]), 3.5),
        'left':   min(min(scan.ranges[54:90]), 3.5),
        }

    def callback_odometry(self, odom):
        self.position = odom.pose.pose.position
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]


    def loop(self):
        # If we are no longer navigating don't proceed
        if not self.active:
            return
        # Otherwise, proceed.

        if self.distance_points(self.target_point, self.position) <= DIST_PRECISION and self.state !=Bug1State.DONE:
            self.change_state(Bug1State.DONE)
        # go the point
        if self.state == Bug1State.GO_TO_POINT:
            # if there is a wall
            if self.regions['front']< MAX_APPROACH_DIST:
                self.circumnavigate_start_point =self.position
                self.circumnavigate_closest_point =self.position
                self.circumnavigate_state_counter = 0
                self.change_state(Bug1State.CIRCUMNAVIGATE)

        elif self.state == Bug1State.CIRCUMNAVIGATE:
            self.circumnavigate_state_counter +=1
            #check if we are closer to the goal 
            if self.distance_points(self.position,self.target_point) < self.distance_points(self.circumnavigate_closest_point,self.target_point):
                self.circumnavigate_closest_point = self.position
                # check if we are back at the start point and that there has passed some time 
            if self.circumnavigate_state_counter > 50 and self.distance_points(self.position,self.circumnavigate_start_point) < DIST_PRECISION:
                self.change_state(Bug1State.GO_TO_CLOSEST)
                # go to the closest point 
        elif self.state == Bug1State.GO_TO_CLOSEST:
            # check if we are close to the point we want to leave the wall
            if self.distance_points(self.position, self.circumnavigate_closest_point)< DIST_PRECISION:
                self.change_state(Bug1State.GO_TO_POINT)
        # check if we are finished
        elif self.state == Bug1State.DONE:
            pass
        else: 
            rospy.logerr('[bug 1] Invalid state')


if __name__ == '__main__':
    print "Starting ROS Bug1 module"
    rospy.init_node('bug1_node', anonymous=True, log_level=rospy.DEBUG)
    bug0 = bug1_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Bug1 module"