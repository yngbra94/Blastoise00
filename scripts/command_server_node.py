#!/usr/bin/env python
"""
    command_server_node.py

    A ROS node that sends commands based on the current state of the 
    USAR problem. 

    Listens to the start and stop commands on the cmd/ topic. These can be sent with:
    rostopic pub -1 /cmd std_msgs/String -- 'start'
    rostopic pub -1 /cmd std_msgs/String -- 'stop'

    The server starts the search for the beacons when getting 'start' 
    and searches until the map is fully explored or all beacons is found.
    It then returns back to the original starting point. 
    If it get 'stop' does the robot stop and restarts the same task it was doing when 'start' is sent.  

    Subscribed: cmd/, beacons_left/, returning_done/, move_base/status, exploring_finished/
    Publishes: state/ 

    Created: 2020/02/04
    Author: Brendan Halloran 
    Changed by: Sondre Iveland
"""

import rospy

from commands import Commands, RobotState
from std_msgs.msg import String, Int16, Bool
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatusArray,  GoalStatus
  

class command_server_node:
    def __init__(self):
        self.state = RobotState.EXPLORING
        self.previusState= self.state
        # list of all ros topic subscription and publications 
        self.subscriber_command = rospy.Subscriber('cmd/', String, self.callback_command)
        self.publisher_state = rospy.Publisher('state/', String, queue_size=1)
        self.subscriber_beacons = rospy.Subscriber('beacons_left/',Int16,self.callback_beacons)
        self.subscriber_robot_state = rospy.Subscriber('robot_current_state', String, self.callback_robot_state) 

        # Publish the current state at 10Hz to make sure other nodes get the correct info
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    # Run at 10Hz
    def loop(self):

        state_msg = String()
        state_msg.data = self.state.value
        self.publisher_state.publish(state_msg)


    # listens to the robot current state 
    # changes state to return home if the the exploring is finished. 
    # if it was returning and it is finished it is stopped and prints reached home. 
    def callback_robot_state(self, data):
        # If robot is exploring and is done, the maze is fully explored. Start returning home.
        if data.data == RobotState.DONE.value and self.state == RobotState.EXPLORING:
            self.state = RobotState.RETURNING
            self.previusState=RobotState.RETURNING
            rospy.loginfo("Finished with exploring the maze")
        
        # If the robot is returning home and is done, the robot has reached home. Start fixing orientation.
        elif data.data == RobotState.DONE.value and self.state == RobotState.RETURNING:
            self.previusState = self.state
            self.state = RobotState.ORIENTING
            
            #printing that the robot is finished. Added many lines to make it easy to see
            rospy.loginfo("\n \n \n \n \n \n The robot has returned back to home.  \n \n \n \n \n \n")
            rospy.loginfo("\n \n \n \n \n \n The robot has returned back to home.  \n \n \n \n \n \n")

        # If the robot is fixing orientation at home and is done, the robot is finished and should stop.
        elif data.data == RobotState.DONE.value and self.state == RobotState.ORIENTING:
            self.previusState = self.state
            self.state = RobotState.DONE
            

    #listens to beacons left and stops exploring when all beacons is found and returns to 
    # start position        
    def callback_beacons(self,beaconsLeft):
        if beaconsLeft.data == 0 and self.state == RobotState.EXPLORING:
            self.state = RobotState.RETURNING
            self.previusState=RobotState.RETURNING
            rospy.loginfo("beacons = 0 robot is returning")

 

            
    # listens to if 'start' or 'stop' is recived.
    # starts exploring if the maze is not explored. 
    # starts returning if the maze is explored.   
    def callback_command(self, data):
        command = Commands(data.data)
     
        #start exploring the maze if not all beacons is found
        if command is Commands.START and self.previusState != RobotState.RETURNING and self.previusState != RobotState.DONE:
            self.state = RobotState.EXPLORING
        
        # start the robot and make it return if the maze is explored
        elif command is Commands.START and self.previusState == RobotState.RETURNING:
            self.state = RobotState.RETURNING
        #stop the robot     
        elif command is Commands.STOP:
            self.state = RobotState.PAUSED
   

if __name__ == '__main__':
    print "Starting ROS Command Server module"
    rospy.init_node('command_server_node', anonymous=True)
    cs = command_server_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Command Server module"
