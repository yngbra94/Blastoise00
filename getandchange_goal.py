#!/usr/bin/env python
# getandchange_goal.py
"""

"""


import rospy
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped

class getandchange_goal:
    returnGoalSet = False
    returningHome = False
    def __init__(self):
        global returnGoalSet
        returnGoalSet = False
        global firstGoal
        global returningHome
        returningHome = False
        
        self.subFeedback = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.setGoalCallback)
        self.subCurrentStatus = rospy.Subscriber('move_base/status', GoalStatusArray, self.return_to_baseCallback)
        self.pubNewGoal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.spin()

    #Listen for the first position of the Turtlebot to store the information as a goal that can be returned to
    def setGoalCallback(self, data):
        global returnGoalSet
        
        #Store information about the first position of the Turtlebot
        if(returnGoalSet == False):
            global firstGoal
            firstGoal = data.feedback.base_position
            print("goal now set to...")
            print(firstGoal)
            returnGoalSet = True #Goal has been stored

    #Listen to move_base/status node for goal status and when to return home
    def return_to_baseCallback(self, data):
        global returningHome
        exploringFinished = False
        statusText = "empty"
        #Check if status is 2 which means that all goals have been cancelled
        if(data.status_list[0].status == 2):
            statusText = data.status_list[0].text
            #Check if length of status is 0, when goals cancelled while exploring is true, it will return a text. But if exploring is finished, there will be no text.
            if(len(statusText) == 0):
                exploringFinished = True
            
        #Check if finished exploring, have stored a goal and not returning home yet.
        if(exploringFinished and returnGoalSet == True and returningHome == False):
            print("Returning to base...")
            #print(firstGoal)
            self.pubNewGoal.publish(firstGoal) # Send goal to return home (to first position that was stored)
            returningHome = True

if __name__ == '__main__':
	try:
		rospy.init_node('getandchange_goal_node')
		sn = getandchange_goal()
	except rospy.ROSInterruptException:
		pass