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
        self.poseExist = False
        self.initPose = "empty"

        # Suvscribing in the feedback data of the robot. 
        self.sub_pose = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.pose_callback)
        print(self.sub_pose)
        rospy.spin()

    
    def pose_callback(self, data):
    
        # Store initial pose
        if(self.poseExist == False):
            self.initPose = data.feedback.base_position.pose
            # print(self.initPose)
            self.poseExist = True


        


if __name__ == '__main__': 
    try:
        rospy.init_node('robot_location_listener')
        sn = initpos_subscriber_node()
    except rospy.ROSInternalException:
       pass




