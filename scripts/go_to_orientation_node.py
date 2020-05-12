#!/usr/bin/env python
"""
    go_to_orientation_node.py
    Created: 2020/04/25
    Author: Brendan Halloran
    Modified from go_to_point_node.py by: group 1 ECTE9477 - Ole-Martin Hanstveit
"""

import rospy
import numpy as np 
import tf
import math
from enum import Enum
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse
from Blastoise00.srv import SetOrientation, SetOrientationResponse

# Constants
YAW_PRECISION = math.pi / 90    # +/- 1 degrees in radians
TURN_SPEED = 0.7

class GoToOrientationState(Enum):
    FIX_YAW = 1
    DONE = 2

class go_to_orientation_node:
    def __init__(self):

        # Node State
        self.stopped = False    # Assume robot is moving and needs to be stopped first
        self.active = False
        self.target_orientation = 0
        self.state = GoToOrientationState.FIX_YAW # Placeholder initial state
        # Robot State
        self.orientation = 0
        self.yaw = 0
        # Services
        self.service_set_orientation = rospy.Service('set_orientation', SetOrientation , self.callback_set_orientation)   # Service this node offers
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
        rospy.logdebug('[Go To Orientation] Stopped')

    
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
                rospy.logdebug('[Go To Orientation] Stopped')
            return
        # Otherwise, proceed.

        if self.state == GoToOrientationState.FIX_YAW:
            self.fix_yaw()
        elif self.state == GoToOrientationState.DONE:
            self.done()
        else:
            rospy.logerr('[Go To Orientation] Invalid state.')

    def fix_yaw(self):
        vel_msg = Twist()
        yaw_error = self.target_orientation - self.orientation
        if math.fabs(yaw_error) > YAW_PRECISION:
            print yaw_error
            vel_msg.angular.z = TURN_SPEED if yaw_error > 0 else -TURN_SPEED
        self.publisher_twist.publish(vel_msg)

        self.active = True
        rospy.logdebug('[Go To Orientation] Fix Yaw')

        if math.fabs(yaw_error) <= YAW_PRECISION:
            rospy.logdebug('[Go To Orientation] Swapping to Done')
            self.state = GoToOrientationState.DONE


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
        rospy.logdebug('[Go To Orientation] Done - Stopped')
        # Wait for service
        rospy.wait_for_service('/go_to_orientation/is_done/')
        try:
            # Create a service to tell the navigation node that w eare done fixing orientation
            pub_explore_state = rospy.ServiceProxy('/go_to_orientation/is_done/',SetBool)
            pub_explore_state(True) # tell navigation node that we are done fixing orientation
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e



    def callback_set_orientation(self, orientation):
        self.target_orientation = orientation.orientation
        self.state = GoToOrientationState.FIX_YAW
        self.active = True
        rospy.logdebug('[Go To Orientation] Swapping to Fix Yaw')
        return SetOrientationResponse(True, 'Target Orientation Set')

    def callback_odometry(self, odom):
        self.orientation = odom.pose.pose.orientation.z

if __name__ == '__main__':
    print "Starting ROS Go To Orientation module"
    rospy.init_node('go_to_orientation_node', anonymous=True, log_level=rospy.DEBUG)
    gto = go_to_orientation_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Go To Orientation module"