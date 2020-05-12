#!/usr/bin/env python
"""
    go_to_point_node.py
    Created: 2020/04/25
    Author: Brendan Halloran
"""

import rospy
import numpy as np 
import tf
import math
from enum import Enum
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse
from movement_starter.srv import SetPoint, SetPointResponse

# Constants
YAW_PRECISION = math.pi / 45    # +/- 4 degrees in radians
DIST_PRECISION = 0.1            # metres
TURN_SPEED = 0.7
DRIVE_SPEED = 0.25

class GoToPointState(Enum):
    FIX_YAW = 1
    GO_STRAIGHT = 2
    DONE = 3

class go_to_point_node:
    def __init__(self):
        # Node State
        self.stopped = False    # Assume robot is moving and needs to be stopped first
        self.active = False
        self.target_point = Point()
        self.state = GoToPointState.FIX_YAW # Placeholder initial state
        # Robot State
        self.position = Point()
        self.yaw = 0
        # Services
        self.service_start_stop = rospy.Service('~start_stop', SetBool , self.callback_state)      # Service this node offers
        self.service_set_point = rospy.Service('~set_point', SetPoint , self.callback_set_point)   # Service this node offers
        # Publishers
        self.publisher_twist = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # Subscribers
        self.subscriber_odometry = rospy.Subscriber('odom/', Odometry, self.callback_odometry)

        # Loop
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()
        self.stopped = True
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.publisher_twist.publish(vel_msg)
        rospy.logdebug('[Go To Point] Stopped')

    
    def loop(self):
        # If we are no longer navigating don't proceed
        if not self.active:
            # If we aren't navigating, but we haven't told the robot to stop, do so now.
            if not self.stopped:
                self.stopped = True
                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                self.publisher_twist.publish(vel_msg)
                rospy.logdebug('[Go To Point] Stopped')
            return
        # Otherwise, proceed.

        if self.state == GoToPointState.FIX_YAW:
            self.fix_yaw()
        elif self.state == GoToPointState.GO_STRAIGHT:
            self.go_straight()
        elif self.state == GoToPointState.DONE:
            self.done()
        else:
            rospy.logerr('[Go To Point] Invalid state.')

    def fix_yaw(self):
        rospy.logdebug('[Go To Point] Fix Yaw')
        desired_yaw = math.atan2(self.target_point.y - self.position.y, self.target_point.x - self.position.x)
        yaw_error = desired_yaw - self.yaw

        vel_msg = Twist()
        if math.fabs(yaw_error) > YAW_PRECISION:
            vel_msg.angular.z = TURN_SPEED if yaw_error > 0 else -TURN_SPEED
        self.publisher_twist.publish(vel_msg)

        if math.fabs(yaw_error) <= YAW_PRECISION:
            rospy.logdebug('[Go To Point] Swapping to Go Straight')
            self.state = GoToPointState.GO_STRAIGHT

    def go_straight(self):
        rospy.logdebug('[Go To Point] Go Straight')
        desired_yaw = math.atan2(self.target_point.y - self.position.y, self.target_point.x - self.position.x)
        yaw_error = desired_yaw - self.yaw
        pos_error = math.sqrt(pow(self.target_point.x - self.position.x, 2) + pow(self.target_point.y - self.position.y, 2))

        if pos_error > DIST_PRECISION:
            vel_msg = Twist()
            vel_msg.linear.x = DRIVE_SPEED
            self.publisher_twist.publish(vel_msg)
        else:
            rospy.logdebug('[Go To Point] Swapping to Done')
            self.state = GoToPointState.DONE

        if math.fabs(yaw_error) > YAW_PRECISION:
            rospy.logdebug('[Go To Point] Swapping to Fix Yaw')
            self.state = GoToPointState.FIX_YAW

    def done(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.publisher_twist.publish(vel_msg)
        self.stopped = True
        self.active = False
        rospy.logdebug('[Go To Point] Done - Stopped')

    def callback_state(self, start_stop):
        if start_stop.data==True and not self.active:
            self.active = True
            rospy.logdebug('[Go To Point] Starting')
            return SetBoolResponse(True, 'Starting Go To Point')
        elif start_stop.data==False and self.active:
            self.stopped = False
            self.active = False
            # setting active to false and stopped to false will cause the next loop
            # callback to send a stop command then set stopped to true.
            rospy.logdebug('[Go To Point] Stopping')
            return SetBoolResponse(True, 'Stopping Go To Point')
        else:
            if start_stop.data==True:
                return SetBoolResponse(False, 'Already Doing Go To Point')
            else:
                return SetBoolResponse(False, 'Already Stopped')

    def callback_set_point(self, point_message):
        self.target_point = point_message.point
        self.state = GoToPointState.FIX_YAW
        rospy.logdebug('[Go To Point] Swapping to Fix Yaw')
        return SetPointResponse(True, 'Target Point Set')

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
    print "Starting ROS Go To Point module"
    rospy.init_node('go_to_point_node', anonymous=True, log_level=rospy.INFO)
    gtp = go_to_point_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Go To Point module"