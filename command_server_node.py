#!/usr/bin/env python
"""
    command_server_node.py

    A ROS node that sends commands based on the current state of the 
    USAR problem. 

    Listens to the start and stop commands on the cmd/ topic. These can be sent with:
    rostopic pub -1 /cmd std_msgs/String -- 'start'
    rostopic pub -1 /cmd std_msgs/String -- 'stop'

    Subscribed: cmd/
    Publishes:

    Created: 2020/02/04
    Author: Brendan Halloran 
    Changed by: Sondre
"""

import rospy

from commands import Commands, RobotState
from std_msgs.msg import String, Int16, Bool
from actionlib_msgs.msg import GoalStatusArray,  GoalStatus

class command_server_node:
    def __init__(self):
        self.state = RobotState.WAITING_TO_START
        self.previusState= RobotState.WAITING_TO_START

        self.subscriber_command = rospy.Subscriber('cmd/', String, self.callback_command)
        self.publisher_state = rospy.Publisher('state/', String, queue_size=1)
        self.subscriber_beacons = rospy.Subscriber('beacons_left/',Int16,self.callback_beacons)
        self.subscriber_returning = rospy.Subscriber('returning_done/',Bool, self.callback_returning)
        self.subscriber_movebase_status =rospy.Subscriber('move_base/status',GoalStatusArray,self.callback_goalstatus)
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

    # Function that detects when the robot is finished with exploring 
    def callback_goalstatus(self, data):
       # if data.status == 3:
        #index=str(data.status_list[0]).find("status: ")
        i=1
        #print "callback status"
        #print str(data.status_list[0])[index+8]
       # if str(data.status_list[0])[index+8]=='3':
        #    print "3 com n"
           # self.state = RobotState.DONE

            
    def callback_beacons(self,beaconsLeft):
        if beaconsLeft.data == 0 and self.state == RobotState.EXPLORING:
            self.state = RobotState.RETURNING
            self.previusState=RobotState.RETURNING
            print "beacons = 0 robot is returning"

    # sends true if the robot is returned
    def callback_returning(self,data):
        if data.data == True and self.state == RobotState.RETURNING: 
            self.state = RobotState.DONE
            self.previusState = RobotState.DONE
            print "robot is returned"


            

    def callback_command(self, data):
        print "heiheis"
        command = Commands(data.data)
        print "heihei"
        #print command
        if command is Commands.START and self.previusState != RobotState.RETURNING:
            self.state = RobotState.EXPLORING
        elif command is Commands.START and self.previusState == RobotState.RETURNING:
            self.state = RobotState.RETURNING
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