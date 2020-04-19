#!/usr/bin/env python
# getandchange_goal.py
"""

"""


import rospy

from visualization_msgs.msg import MarkerArray
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from commands import RobotState, Commands

"""
    navigation_node.py
    This node is responsible for controlling the navigational aspect of the turtlebot 
    this node listens to the state/ from the commandserver node and based on this performs the set task. 
    The tasks is: explore the maze with sending true to the /explore/explore_service, 
    return back to start point, and stop (pause) the robot. 
    The code needs to implement getandchange_goal.py in a nicer way. not global variables
    The command_server node needs to know when the maze is explored using exploring_finished/
     and when the robot is back in start position using returning_done/
     > store the starting position
     > the code for starting the exploring is commented out in command server node. near the bottom
     > Pausing / stoping of the robot is not implemented for the go back home part. 
    NOTE!!: currently the code "getandchange_goal.py" listens to move_base/status goal status and text,
    this should be changed to listen to marker array for frontiers as explained in message from Brendan
    below.
    
    Subscribed:  state/, move_base/feedback, move_base/status  
    Publishes:   returning_done/, exploring_finished/, move_base_simple/goal
    
    
    Message from Brendan: on which node should be subscribed to in order to know when
    exploring is finished.
    ================================================================================================
    You should use the topics published by the explore server itself. All of the topics/services are
    under the "/explore/" namespace. The only topic published is an marker array called "frontiers",
    and only when the "visualize" parameter is set to true.
    I have noticed a bug that can prevent this from publishing as often as it should, so I've made a
    quick fix and pushed the change. Try doing a git pull in your explore_lite folder and if you don't
    get a new commit d0b130b then you might have to download the repository again.
    Once you get the fix, have a look at the last message published before exploration stops and see how
    it's different to the ones published before stopping.
    ================================================================================================
"""
class navigation_node:

    def __init__(self):
        self.returnGoalSet = False        
        self.firstGoal = 0
        self.returningHome = False
        self.exploringFinished = False
        global SetBool = True

        self.robotCurrentState = rospy.Subscriber('state/', String, self.server_state)    # Get state from command server
 #       self.robotCurrentState = RobotState.WAITING_TO_START

        self.subFeedback = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.setGoalCallback)
        self.subStatus = rospy.Subscriber('move_base/status', GoalStatusArray, self.return_to_baseConfirm)
        
        self.subFrontierStatus = rospy.Subscriber('explore/frontiers', MarkerArray, self.return_to_baseCallback)
        self.pubNewGoal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

        self.pubExploringDone = rospy.Publisher('exploring_finished', Bool, queue_size=10) # publish to /exploring_finished topic
        self.pubReturningDone = rospy.Publisher('returning_done', Bool, queue_size=10) # publish to /returning_done when returning is done 

        
        
        self.pubRobotState = rospy.Publisher('robotCurrentState', SetBool, queue_size=10)
                
        rospy.wait_for_service('/explore/explore_service')
        try:
            startFunction = rospy.ServiceProxy('/explore/explore_service', SetBool)
            startFunction(False)
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

        
        rospy.spin()


    def server_state(self, state):

       print (state) 
       if(state == RobotState.WAITING_TO_START):
        print "waiting to start" 
       if (state == RobotState.EXPLORING):
        print "Exploring"
        startFunction(True)
            

     #Listen for the first position of the Turtlebot to store the information as a goal that can be returned to
    def setGoalCallback(self, data):
        self.returnGoalSet
        
        #Store information about the first position of the Turtlebot
        if(self.returnGoalSet == False):

            self.firstGoal = data.feedback.base_position
            print("goal now set to...")
            print(self.firstGoal)
            self.returnGoalSet = True #Goal has been stored

    def return_to_baseCallback(self, data):        

        #Check if array is empty which means that exploring is finished
        if(data.markers == []):
 #           self.exploringFinished = True
            self.pubExploringDone.publish(True)
        




        #Check if finished exploring, have stored a goal and not returning home yet.
        if(self.exploringFinished and self.returnGoalSet == True and self.returningHome == False):
            print("Returning to base...")

            self.pubNewGoal.publish(self.firstGoal) # Send goal to return home (to first position that was stored)
            self.returningHome = True

   
   
    def return_to_baseConfirm(self, data):  # Tell command server that Blastoise is home!
        if(data.status_list[0].status == 3):
            print "Robot returned"
            self.pubReturningDone.publish(True)

            

if __name__ == '__main__':
	try:
		rospy.init_node('navigation_node_node')
		sn = navigation_node()
	except rospy.ROSInterruptException:
		pass