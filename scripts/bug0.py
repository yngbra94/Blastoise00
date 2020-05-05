#!/usr/bin/env python
"""
    bug0.py
    Created: 2020/04/25
    Author: Brendan Halloran
"""

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

class Bug0State(Enum):
    GO_TO_POINT = 1
    WALL_FOLLOW = 2
    DONE = 3

class bug0_node:
    def __init__(self):
        # Node State
        self.active = False
        self.target_point = Point()
        self.state = Bug0State.GO_TO_POINT # Placeholder initial state
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

    def change_state(self, state):
        if self.state == state:
            return
        if state == Bug0State.GO_TO_POINT:
            rospy.logdebug('[Bug 0] Swapping to Go To Point')
            self.state = state
            resp = self.start_go_to_point(True)
            resp = self.start_wall_follower(False)
        elif state == Bug0State.WALL_FOLLOW:
            rospy.logdebug('[Bug 0] Swapping to Wall Follow')
            self.state = state
            resp = self.start_go_to_point(False)
            resp = self.start_wall_follower(True)
        elif state == Bug0State.DONE:
            rospy.logdebug('[Bug 0] Done')
            self.state = state
            resp = self.start_go_to_point(False)
            resp = self.start_wall_follower(False) 

    def loop(self):
        # If we are no longer navigating don't proceed
        if not self.active:
            return
        # Otherwise, proceed.

        desired_yaw = math.atan2(self.target_point.y - self.position.y, self.target_point.x - self.position.x)
        yaw_error = self.normalise_angle(desired_yaw - self.yaw)
        pos_error = math.sqrt(pow(self.target_point.x - self.position.x, 2) + pow(self.target_point.y - self.position.y, 2))

        if pos_error <= DIST_PRECISION and self.state != Bug0State.DONE:
            self.change_state(Bug0State.DONE)

        if self.state == Bug0State.GO_TO_POINT:
            if self.regions['front'] < MAX_APPROACH_DIST:
                self.change_state(Bug0State.WALL_FOLLOW)
        elif self.state == Bug0State.WALL_FOLLOW:
            if math.fabs(yaw_error) < (math.pi / 6) and self.regions['front'] > MAX_APPROACH_DIST and self.regions['fright'] > MAX_SIDE_LIMIT and self.regions['fleft'] > MAX_SIDE_LIMIT:
                self.change_state(Bug0State.GO_TO_POINT)
            if yaw_error > 0 and math.fabs(yaw_error) > (math.pi / 6) and math.fabs(yaw_error) < (math.pi / 2) and self.regions['left'] > MAX_APPROACH_DIST and self.regions['fleft'] > MAX_SIDE_LIMIT:
                self.change_state(Bug0State.GO_TO_POINT)
            if yaw_error < 0 and math.fabs(yaw_error) > (math.pi / 6) and math.fabs(yaw_error) < (math.pi / 2) and self.regions['right'] > MAX_APPROACH_DIST and self.regions['fright'] > MAX_SIDE_LIMIT:
                self.change_state(Bug0State.GO_TO_POINT)
        elif self.state == Bug0State.DONE:
            pass
        else:
            rospy.logerr('[Bug 0] Invalid state.')


    def callback_state(self, start_stop):
        if start_stop.data==True and not self.active:
            self.active = True
            rospy.logdebug('[Bug 0] Starting')
            if self.state == Bug0State.GO_TO_POINT:
                rospy.logdebug('[Bug 0] Starting Go To Point')
                resp = self.start_go_to_point(True)
                resp = self.start_wall_follower(False)
            elif self.state == Bug0State.WALL_FOLLOW:
                rospy.logdebug('[Bug 0] Starting Wall Follow')
                resp = self.start_go_to_point(False)
                resp = self.start_wall_follower(True)
            return SetBoolResponse(True, 'Starting Bug 0')
        elif start_stop.data==False and self.active:
            self.active = False
            rospy.logdebug('[Bug 0] Stopping')
            resp = self.start_go_to_point(False)
            resp = self.start_wall_follower(False)
            return SetBoolResponse(True, 'Stopping Bug 0')
        else:
            if start_stop.data==True:
                return SetBoolResponse(False, 'Already Doing Bug 0')
            else:
                return SetBoolResponse(False, 'Already Stopped')

    def callback_set_point(self, point_message):
        self.target_point = point_message.point
        resp = self.go_to_point_set_point(point_message.point)
        self.state = Bug0State.GO_TO_POINT
        rospy.logdebug('[Bug 0] Swapping to Go To Point')
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

if __name__ == '__main__':
    print "Starting ROS Bug0 module"
    rospy.init_node('bug0_node', anonymous=True, log_level=rospy.DEBUG)
    bug0 = bug0_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Bug0 module"