#!/usr/bin/env python

"""
    navigation_node.py
    ================================================================================================

    This node is responsible for controlling the navigational aspect of the turtlebot this node listens 
    to the state/ from the commandserver node and based on this if performs the given task. 
    The tasks is: explore the maze with sending true to the /explore/explore_service, 
    return back to start point, and stop (pause) the robot. 

    ================================================================================================
    Subscribed:  state/, move_base/feedback, move_base/status  
    Publishes:   returning_done/, exploring_finished/, move_base_simple/goal
    ================================================================================================

"""

import rospy # ros library for python. 

# Msg datatypes
from std_msgs.msg import String, Int16, Bool
from actionlib_msgs.msg import GoalStatusArray #, GoalStatus
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseFeedback
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool 
from actionlib_msgs.msg import GoalID


# Robot Commands
from commands import RobotState, RobotCommand

class navigation_node: 

    # Subscribing to the /move_base/...
    def __init__(self):
        # Variables: 
        self.debug = True 
        self.poseExist = False
        # make a pose in pos;[0 0 0] orientation; [0 0 0 1]
        self.initPose = PoseStamped()
        self.initPose.pose.orientation.w = 1.0 # 
        self.currentPose = PoseStamped()
        self.robotCurrentState = RobotState.WAITING_TO_START # Initial starting state

        # Sub to command server status 
        self.sub_server_cmd = rospy.Subscriber('state', String, self.server_cmd)

        # Sub to move_base/feedback for pose information. 
        self.sub_pose = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.pose_callback)

        # Sub to move_base/status from the robot. 
        self.sub_status = rospy.Subscriber('move_base/status', GoalStatusArray, self.robot_status_callback)

        # Sub to /explore/frontiers from the robot. 
        self.sub_frontiers = rospy.Subscriber('explore/frontiers', MarkerArray, self.frontiers_callback)


        #
        # Publisher 
        # Publishes new goals to the robot. 
        self.pub_new_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.pub_cancel_all_goals = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

        # Publish the explire command 
        if(self.debug):
            print("Waiting for /explore/explore_service... ")
        rospy.wait_for_service('/explore/explore_service')
        
        try:
            # make the service to the explore state. This tells it to start or stop exploring
            self.pub_explore_state = rospy.ServiceProxy('/explore/explore_service',SetBool)
            self.pub_explore_state(False) # tell explore node to stop the exploration 
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e


        # Publish curret robot action
        self.pub_robotCurrentState = rospy.Publisher('robot_current_state/', String , queue_size=10)

        if(self.debug):
            print("Init done")

        # DJ, Spin that ROS 
        rospy.spin()


    # Callback from the server node. 
    # Saves current robot state and executes the command
    # @param state, Cuttent robot state , Type String
    def server_cmd(self, cmd): 
        # If the command is to explore, start exploring
        if (cmd.data == RobotState.EXPLORING.value):
            self.robot_explore()
        # If the command is to Pause the exploring, do so 
        if (cmd.data == RobotState.PAUSED.value):
            self.robot_pause()

        # If the command is to return home, do return home 
        if (cmd.data == RobotState.RETURNING.value):
            self.robot_return_home()

    # Make robot start exploring 
    def robot_explore(self):
        # try to start the exploring. If not, cast exception
        try:
            self.pub_explore_state(True)
            # Change state to EXPLORING
            self.set_robot_action_and_pub(RobotState.EXPLORING)
            if(self.debug):
                print("Exploring...")

        except rospy.ServiceException, e:
            print "Could not starte exploring. \n Service call failed: %s" %e



    # Set the goal of the robot to the home position. 
    def robot_return_home(self):
        # check if there is no stored home pose
        if not self.initPose:
            print("No home pose found. \n  check bringup order. ")
        # Publish the initial pose to return home
        else: 
            self.initPose.header = self.currentPose.header
            self.pub_new_goal.publish(self.initPose) # Send goal to return home (the first position that was stored)
            # Set robot action to RETURNING_HOME and publish
            self.set_robot_action_and_pub(RobotState.RETURNING)
            if (self.debug):
                print("Returning to home from pose: {0} \n To home pose: {1} ".format(self.currentPose, self.initPose))


    
    # Pause the current operation of the robot. 
    def robot_pause(self):
        # try to stop the exploring. If not, print exception
        try:
            self.pub_explore_state(False)
        except rospy.ServiceException, e:
            print "Could not stop exploring. \n Service call failed: %s" %e
        # Cancel all current goals. stop the robot    
        self.pub_cancel_all_goals.publish()
        # Change state to PAUSED and publish
        self.set_robot_action_and_pub(RobotState.PAUSED)

        if(self.debug):
            print("Paused...")


    # Callback for the robot pose.
    # Updating the current position  
    # @param data, Info from move_base feedback, Type MoveBaseActionFeedback. 
    def pose_callback(self, data):

        # Update current pose
        self.currentPose = data.feedback.base_position
        
    
    # Callback for the robot state
    # listens to move_base and check if the robot is returned home (Goal reached)
    def robot_status_callback(self, status):

        # If the robot is on it's way home and it has reached its goal, the robot is home
        if (self.robotCurrentState == RobotState.RETURNING and status.status_list[0].status == 3):
            # Set action to robot is home and publish 
            self.set_robot_action_and_pub(RobotState.DONE)
           
            # Used with debug
            if (self.debug):
                print("Robot parked in the garage. ")
        
    # Returning to home call back checks is there is any more frontiers left. 
    # 
    # @param frontiers, The subscribed frointer data from the robot, Type MarkerArray
    def frontiers_callback(self, frontiers):
        if(frontiers.markers == [] and self.robotCurrentState == RobotState.EXPLORING):

            # Used with debug
            if(self.debug):
                print("No more frontiers.")
            # Change the state state to DONE and publish
            self.set_robot_action_and_pub(RobotState.DONE)
         

    
    # Set the current state of the robot and publishes it
    #
    # @param newState, the new state to be set. 
    def set_robot_action_and_pub(self, newState):
        self.robotCurrentState = newState
        self.pub_robotCurrentState.publish(self.robotCurrentState.value)



# Main function
# Initalisation of the navigation node 
if __name__ == '__main__': 
    try:
        rospy.init_node('navigation_node')
        sn = navigation_node()
        print("Navigation Node Started... ")
    except rospy.ROSInternalException:
       pass





