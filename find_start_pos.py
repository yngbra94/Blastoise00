#!/usr/bin/env python

# This program intends to subscribe on a massage from the turtlebot with the initial position and orientation. 

# NOTES: can rosmsg be used to find the pose of the robot? 
        # http://wiki.ros.org/rosmsg


import rospy # ros library for python. 
from geometry_msgs.msg import Pose, Point, Quaternion # 
from nav_msgs.msg import Odometry

from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseFeedback

class initpos_subscriber_node: 


    # Subscribing to the /move_base/...
    def __init__(self):
        # Variables: 
        self.initPose = "empty"
        self.posCnt = 0

        # Suvscribing in the feedback data of the robot. 
        self.sub_pose = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.pose_callback)
        rospy.spin()

    def pose_callback(self, data):
        self.initPose = data.feedback.base_position.pose
        if(self.initPose != "empty" and self.posCnt == 0):
            print(self.initPose)
            self.posCnt += 1

        
        



if __name__ == '__main__': 
    try:
        rospy.init_node('robot_location_listener')
        sn = initpos_subscriber_node()
    except rospy.ROSInternalException:
       pass




