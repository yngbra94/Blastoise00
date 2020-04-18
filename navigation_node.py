#!/usr/bin/env python

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
