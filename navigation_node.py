#!/usr/bin/env python

"""
    navigation_node.py

    This node is responsible for controlling the navigational aspect of the turtlebot 
    The listens to the state/ and based on this performs the set task. 
    The tasks is: explore the maze with sending true to the /explore/explore_service, 
    return back to start point, and stop (pause) the robot. 


    The code needs to implement getandchange_goal.py in a nicer way. not global variables
    The command_server node needs to know when the maze is explored using exploring_finished/
     and when the robot is back in start position using returning_done/
     > store the starting position
     > the code for starting the exploring is commented out in command server node. near the bottom
     > Pausing / stoping of the robot is not implemented for the go back home part. 



    Subscribed:  state/, move_base/feedback, move_base/status, move_base_simple/goal 
    Publishes:   returning_done/, exploring_finished/



"""