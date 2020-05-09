#!/usr/bin/env python
"""
    move_base_fake.py
    Created: 2020/05/3
    Author: Brendan Halloran
    Edited: Group 1 ecte477 2020  
"""

import rospy
from actionlib import SimpleActionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult, MoveBaseActionGoal
from std_srvs.srv import SetBool, SetBoolResponse
from movement_starter.srv import SetPoint, SetPointResponse
from geometry_msgs.msg import PoseStamped, Point, Pose
from nav_msgs.msg import Odometry
from enum import Enum
class bugType(Enum):
    BUG0 = '/bug0'
    BUG1 = '/bug1'
    BUG2 = '/bug2'

class move_base_fake_node:
    def __init__(self):
        self.action_server = SimpleActionServer('move_base', MoveBaseAction, execute_cb = self.execute_callback, auto_start = False)    # Simple action server will pretend to be move_base

        # Movement goal state
        self.goal = None                        # Is a goal set
        self.valid_goal = False                 # Is this a valid goal
        self.current_goal_started = False       # Has the goal been started (i.e. have we told our Bug algorithm to use this point and start)
        self.current_goal_complete = False      # Has the Bug algorithm told us it completed 
        self.position = None                    # move_base feedback reports the current direction
        self.bugType = bugType.BUG2             # set the bug type to be used 

        # Service for the Bug algorithm to tell us it is done
        self.bug_done_service = rospy.Service('/move_base_fake/is_bug_done/', SetBool, self.callback_complete)

        self.subscriber_odometry = rospy.Subscriber('odom/', Odometry, self.callback_odometry)                              # We need to read the robots current point for the feedback
        self.subscriber_simple_goal = rospy.Subscriber('/move_base_simple/goal/', PoseStamped, self.callback_simple_goal)   # Our return goal is done with /move_base_simple/goal/
        self.goal_pub = rospy.Publisher('/move_base/goal/', MoveBaseActionGoal, queue_size=10)                              # /move_base_simple/goal/ gets published here
        
        self.action_server.start()

    # publish the state to the bug node, either start or stop. 
    def pub_state_to_bug(self,state,bugType):
        bug_service =bugType.value +'_node/start_stop'
        
        rospy.wait_for_service(bug_service)
       
        try:
            # Create a service for the bug service to toggle state
            pub_bug_state = rospy.ServiceProxy(bug_service,SetBool)
            pub_bug_state(state) # tell bug node to change the exploration 
      
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    # publish the goal for the bug algorithm
    def pub_goal_to_bug(self,newGoal,bugType):
       #get the point from the pose 
        goalPoint = newGoal.position
         # /bug0 or /bug1 or /bug2
        bug_service =bugType.value +'_node/set_point'
        rospy.wait_for_service(bug_service)
        try:
           
            # Create a service for the bug service to toggle state
            pub_bug_goal = rospy.ServiceProxy(bug_service,SetPoint)
            pub_bug_goal(goalPoint) # tell bug node to change the exploration 
            rospy.logdebug("[move base fake] sending new position goal")
           
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    # 
    def execute_callback(self, move_base_goal):
        self.goal = move_base_goal.target_pose.pose                                                 # Set the provided goal as the current goal
        rospy.logdebug('[Move Base Fake] Execute Callback: {}'.format(str(self.goal.position)))
        self.valid_goal = True                                                                      # Assume it is valid
        self.current_goal_started = False                                                           # It hasnt started yet
        self.current_goal_complete = False                                                          # It hasnt been completed

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Always start by checking if there is a new goal that preempts the current one
            if self.action_server.is_preempt_requested():

                # Tell Bug algorithm to stop before changing or stopping goal
                self.pub_state_to_bug(False, self.bugType)  # Stop bug algorithm

                if self.action_server.is_new_goal_available():
                    new_goal = self.action_server.accept_new_goal()                                     # There is a new goal
                    rospy.logdebug('[Move Base Fake] New Goal: {}'.format(str(self.goal.position)))
                    self.goal = new_goal.target_pose.pose       # Set the provided goal as the current goal
                    self.valid_goal = True                      # Assume it is valid
                    self.current_goal_started = False           # It hasnt started yet
                    self.current_goal_complete = False          # It hasnt been completed
                else:
                    self.action_server.set_preempted()          # No new goal, we've just been told to stop
                    self.goal = None                            # Stop everything
                    self.valid_goal = False
                    self.current_goal_started = False
                    self.current_goal_complete = False
                    return

            # Start goal
            if self.valid_goal and not self.current_goal_started:
                rospy.logdebug('[Move Base Fake] Starting Goal')
                #set the goal 
                self.pub_goal_to_bug(self.goal,self.bugType)
                #start to go to the goal 
                self.pub_state_to_bug(True, self.bugType)
                self.current_goal_started = True        # Only start once

            # Feedback ever loop just reports current location
            feedback = MoveBaseFeedback()
            feedback.base_position.pose.position = self.position
            self.action_server.publish_feedback(feedback)

            # Completed is set in a callback that you need to link to a service or subscriber
            if self.current_goal_complete:
                rospy.logdebug('[Move Base Fake] Finishing Goal')
                #stop the bug algorithm
                self.pub_state_to_bug(False, self.bugType)
                self.goal = None                        # Stop everything
                self.valid_goal = False
                self.current_goal_started = False
                self.current_goal_complete = False
                self.action_server.set_succeeded(MoveBaseResult(), 'Goal reached')  # Send success message
                return

            r.sleep()

        # Shutdown
        rospy.logdebug('[Move Base Fake] Shutting Down')
        
    
        self.goal = None                        # Stop everything
        self.valid_goal = False
        self.current_goal_started = False
        self.current_goal_complete = False

    # Callback for when Bug is done with the current goal.
    def callback_complete(self, success):
        self.current_goal_complete = success.data

    def callback_odometry(self, odom):
        self.position = odom.pose.pose.position

    # Simple goals get republished to the correct topic
    def callback_simple_goal(self, goal):
        rospy.logdebug('[Move Base Fake] Simple Goal: {}'.format(str(goal.pose.position)))
        action_goal = MoveBaseActionGoal()
        action_goal.header.stamp = rospy.Time.now()
        action_goal.goal.target_pose = goal
        self.goal_pub.publish(action_goal)


if __name__ == '__main__':
    print "Starting ROS Move Base Fake module"
    rospy.init_node('move_base_fake_node', anonymous=True, log_level=rospy.DEBUG)
    move_base = move_base_fake_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Move Base Fake module"