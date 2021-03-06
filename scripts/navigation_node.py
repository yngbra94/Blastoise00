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
from nav_msgs.msg import OccupancyGrid, Odometry, Path


# Robot Commands
from commands import RobotState, RobotCommand

class navigation_node: 

    # Subscribing to the /move_base/...
    def __init__(self):
        # Variables: 
        self.debug = False 
        # make a pose in pos;[0 0 0] orientation; [0 0 0 1]
        self.poseExist = False
        self.initPose = PoseStamped()
        self.initPose.pose.orientation.w = 1.0 # 
        self.currentPose = PoseStamped()
        self.robotCurrentState = RobotState.WAITING_TO_START # Initial starting state
        self.wallfollowing_state = False # Wall following state

        # Publishers 
        # Publishes new goals to the robot. 
        self.pub_new_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.pub_cancel_all_goals = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.pub_map = rospy.Publisher('ecte477/map', OccupancyGrid, queue_size=10)

        # Publish current robot action
        self.pub_robotCurrentState = rospy.Publisher('robot_current_state/', String , queue_size=10)

        # Sub to command server status 
        self.sub_server_cmd = rospy.Subscriber('state', String, self.server_cmd)

        # Sub to move_base/feedback for pose information. 
        self.sub_pose = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.pose_callback)

        # Sub to move_base/status from the robot. 
        self.sub_status = rospy.Subscriber('move_base/status', GoalStatusArray, self.robot_status_callback)

        # Sub to /explore/frontiers from the robot. 
        self.sub_frontiers = rospy.Subscriber('explore/frontiers', MarkerArray, self.frontiers_callback)

        # Sub to /map
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Sub to odom for path messages -> publish to ecte477/path
        self.subscriber_odometry = rospy.Subscriber('odom/', Odometry, self.callback_get_path)  # Added callback for 'path'
        self.publisher_path = rospy.Publisher('/ecte477/path', Path, latch=True, queue_size=10) # Publisher for path msgs
        self.path = Path()  # -> callback_path

        if(self.debug):
            print("Init done")

        # DJ, Spin that ROS 
        rospy.spin()

    # Callback for publishing new map
    def map_callback(self, map):
        self.pub_map.publish(map)

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

    # Set state of explore service to either true or false
    def set_explore_state(self, state):
        if(self.debug):
            print("Waiting for /explore/explore_service... ")

        # Wait for service
        rospy.wait_for_service('/explore/explore_service')
        try:
            # Create a service for the explore service to toggle state
            pub_explore_state = rospy.ServiceProxy('/explore/explore_service',SetBool)
            pub_explore_state(state) # tell explore node to stop the exploration 
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e


    # Make robot start exploring 
    def robot_explore(self):
        # Tell explore service to start exploring
        self.set_explore_state(True)

        # Change state to EXPLORING
        self.set_robot_action_and_pub(RobotState.EXPLORING)
        if(self.debug):
            print("Exploring...")

    # Set the goal of the robot to the home position. 
    def robot_return_home(self):
        # Make sure wall follower is stopped before returning
        self.toggle_wallfollowing(False)
        # check if there is no stored home pose
        if not self.initPose:
            print("No home pose found. \n  check bringup order. ")
        # Publish the initial pose to return home
        else: 
            if self.robotCurrentState != RobotState.RETURNING:
                self.initPose.header = self.currentPose.header
                self.pub_new_goal.publish(self.initPose) # Send goal to return home (the first position that was stored)
                # Set robot action to RETURNING_HOME and publish
                self.set_robot_action_and_pub(RobotState.RETURNING)
            if (self.debug):
                print("Returning to home from pose: {0} \n To home pose: {1} ".format(self.currentPose, self.initPose))

    
    # Pause the current operation of the robot. 
    def robot_pause(self):
        # Tell explore service to stop exploring
        self.set_explore_state(False)
        # Cancel all current goals. stop the robot    
        self.pub_cancel_all_goals.publish()
        # Change state to PAUSED and publish
        self.set_robot_action_and_pub(RobotState.PAUSED)
        # Tell wall follower to stop
        self.toggle_wallfollowing(False)

        if(self.debug):
            print("Paused...")


    # Callback for the robot pose.
    # Updating the current position  
    # @param data, Info from move_base feedback, Type MoveBaseActionFeedback. 
    def pose_callback(self, data):

        # Update current pose
        self.currentPose = data.feedback.base_position
        
        # Store initial pose
        if(self.poseExist == False):
            # Set init pose
            self.initPose = data.feedback.base_position
            # Used with debug
            if(self.debug):
                print("Init pose saved as:  %s "% (self.initPose.pose))
            # Pose exist flag
            self.poseExist = True


    
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
        
    # If there are no frontiers left, switch to wall following to continue exploring 
    # 
    # @param frontiers, The subscribed frointer data from the robot, Type MarkerArray
    def frontiers_callback(self, frontiers):
        if(frontiers.markers == [] and self.robotCurrentState == RobotState.EXPLORING and self.wallfollowing_state == False):
            # Notify command server that all frontiers are explored
            self.set_robot_action_and_pub(RobotState.DONE)

            # Make sure explore node is stopped
            self.set_explore_state(False)

            # Continue exploring by starting wall following
            self.toggle_wallfollowing(True)
         
         # Used with debug
            if(self.debug):
                print("No more frontiers.")


    # Start wall following
    def toggle_wallfollowing(self, state):
        # Wait for service
        rospy.wait_for_service('start_stop')
        try:
            # Create a service for the wallfollower service to toggle state
            pub_wallfollow_start_top = rospy.ServiceProxy('start_stop',SetBool)
            pub_wallfollow_start_top(state) # tell wall follower to start or stop
            self.wallfollowing_state = state
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    
    # Set the current state of the robot and publishes it
    #
    # @param newState, the new state to be set. 
    def set_robot_action_and_pub(self, newState):
        self.robotCurrentState = newState
        self.pub_robotCurrentState.publish(self.robotCurrentState.value)


    def callback_get_path(self, odom):   # Accumulates nav_msgs/Odometry messages, publishes on nav_msgs/Path
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path.header = odom.header
        self.path.poses.append(pose)
        self.publisher_path.publish(self.path)


# Main function
# Initalisation of the navigation node 
if __name__ == '__main__': 
    try:
        rospy.init_node('navigation_node')
        sn = navigation_node()
        print("Navigation Node Started... ")
    except rospy.ROSInternalException:
       pass





