#!/usr/bin/env python

import rospy
import tf
import math

from enum import Enum
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from std_srvs.srv import SetBool, SetBoolResponse
from movement_starter.srv import SetPoint, SetPointResponse
from geometry_msgs.msg import PoseStamped

YAW_PERCISTION = (math.pi / 90) # +/- 2 degrees  in  radians
DIST_PRECISION = 0.1          # metre
MAX_SIDE_LIMIT = 0.5            # This  furthest  distance  we 'see' the  wall to the  side
MAX_APPROACH_DIST = 0.3       # The  closest  we want to get to a wall  from  the  front
STATE_COUNTER_LIMIT = 50        # Iterations 


"""
---------------------------------------------------------------------------------
This node is a modified bug2 algorithm. It has three states which is GO_TO_POINT, 
WALL_FOLLOW and DONE. The goal of this node is to receve a target point and move there.
Then when the target point is reached it changes state to DONE and publishes that it is done. 
The original bug2 algorithm tries to move straight to the target point (GO_TO_POINT). 
If it meets an obstacle, it changes to WALL_FOLLOWER. Then it will follow the wall 
until it is at the line between the point where it started following the wall and 
the target point before it tries to GO_TO_POINT again. However, some issues were found 
with the original algorithm and was tried to be solved with these modifications. 

Issue 1:    It was found that if the robot is in WALL_FOLLOWER mode when it reaches the 
            target point line and at the same time has a wall in front, if will change to GO_TO_POINT
            and then automatically back to WALL_FOLLOWER. This is because the algorithm is made such
            that if there is a wall in front of the robot, if should change to WALL_FOLLOWER. 
Solution 1: There is added a restriction saying that if the robot is in GO_TO_POINT, 
            it needs to be facing towards the point with a 0.3 radian accuracy. This gives the 
            robot time to turn around before it can go back in to WALL_FOLLOWING. 
            
Issue 2:    The original algorithm has no way of selecting which wall to follow. 
Solution 2: To decide which wall to follow the front left and front right laser scan is used.
            If the robot has a wall closer to the front left if will use wall follower on the left side. 
            The same works for the right side. 

Issue 3:    Sometimes the robot starts to follow the wall in the wrong direction and brings the robot 
            further away from the target goal. Also, if the robot then finds the line it will go back 
            where it started following the wall and do the same mistake once more.            
Solution 3: Now, if the point where the robot finds the line is further away from the target point
            than where it started wall follower, it will change it's wall follower direction and 
            try one more time. 

Issue 4:    The robot will crash in to the wall if it is close to the wall and is set to GO_TO_POINT. 
Solution 4: The robot is now not allowed to go to point before it knows that there is nothing to crash into
            in front of it. 

Issue 5:    The robot needs to point towards the point it is moving to before it can starts wall following. 
            This makes unnecessary rotations. 
Solution 5: To solve this issue it was tried to use a dynamic laser scan which always points towards the target point. 
            The idea was that if there is a wall between the target point and the robot, there is no 
            need to rotate the robot before it changes to WALL_FOLLOWER. 
            However, this was found to lower the performance of the system. This is most likely due to 
            the noise generated when the robot is always able to find a wall for a new point. 
            Hence, it is not in use. 

Issue 6:    When the robot gets stuck in a corner or in a square shape, it will hit the line multiple times
            and return to wall follower. This makes the robot go in circles. Solution 3 should help the robot 
            to get out of this loop. However, Solution 2 made the robot always predicting the best wall to follow. 
            Hence, it never changed the wall following direction. 
Solution 6: A counter was added to count if the line was hit more than twice. If this happens, it means that the robot is stuck. 
            If the robot is stuck, it will not try to predict the best wall to follow but try the other one. 
            Unfortunately this only works some of the time.

----------------------------------------------------------------------------------

"""

class Bug2State(Enum):
    GO_TO_POINT = 1
    WALL_FOLLOW = 2
    DONE = 3
class bug2_node: 
    def __init__(self):
        # Node State
        self.active = False
        self.target_point = Point()
        self.state = Bug2State.GO_TO_POINT # Placeholder for initial state
        self.state_counter = 0
        self.robot_was_stuck_cnt = 0

        # Wall Follow init state 
        self.wall_follow_left_dir    = False
        self.wall_follow_start_point   = None
        self.wall_follow_closest_point = None         

        # Robot state
        self.position = Point()
        self.yaw      = 0
        self.regions  = None
        self.dynamic_region = None

        # Services this node offers. 
        self.service_start_stop = rospy.Service('~start_stop', SetBool, self.callback_state)
        self.service_set_point = rospy.Service('~set_point', SetPoint, self.callback_set_point)

        # Service calls
        self.start_go_to_point = rospy.ServiceProxy('/go_to_point_node/start_stop', SetBool)
        self.start_wall_follower = rospy.ServiceProxy('/wall_follower_node/start_stop', SetBool)
        self.left_wall_follower = rospy.ServiceProxy('/wall_follower_node/follow_left', SetBool)
        self.go_to_point_set_point = rospy.ServiceProxy('/go_to_point_node/set_point', SetPoint)
        

        # Wait for service to be available. 
        rospy.loginfo("Waiting for services... ")
        rospy.wait_for_service('/go_to_point_node/start_stop')
        rospy.wait_for_service('/wall_follower_node/start_stop')
        rospy.wait_for_service('/wall_follower_node/follow_left')
        rospy.wait_for_service('/go_to_point_node/set_point')
        rospy.loginfo("Services available")
        
        # Set Wall follower direction. 
        self.left_wall_follower(self.wall_follow_left_dir)

        # Subscribers
        self.subscriber_laser_scan = rospy.Subscriber('scan/', LaserScan , self.callback_laser_scan)
        self.subscriber_odometry = rospy.Subscriber('odom/', Odometry , self.callback_odometry)

        # Debug publishers
        self.pub_debug = self.goal_pub = rospy.Publisher('~debug', Point, queue_size=10)


        # Loop 
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    # Node loop. 
    def loop(self):
        """
        Node loop. 
        Controls the Bug2 node
        It is able to check if a goal is reached,
        And if the state need to be changed.
        """

        # If we are no longer navigating then don't proceed 
        if not self.active: 
            return

        # Check if done
        if self.distance_points(self.target_point, self.position) <= DIST_PRECISION and self.state != Bug2State.DONE: 
            self.change_state(Bug2State.DONE)
        
        # Check if state needs changing 
        if self.state == Bug2State.GO_TO_POINT: 
            # If the robot is pointing toward the target point and there ia a wall to follow. 
            if self.yaw_error_to_point(self.position, self.target_point) <= 0.3 and self.yaw_error_to_point(self.position, self.target_point) >= -0.3: 
                if self.regions['front'] < MAX_APPROACH_DIST:
                    # If the robot has not been stuck, follow the closes wall. 
                    if self.robot_was_stuck_cnt == 0 :
                        if self.regions['fleft'] <= self.regions['fright'] or self.regions['left'] <= self.regions['right']: # If wall is closer to the left, follow left
                            print ("wall follow left")
                            self.set_wall_follower_dir(True)
                        elif self.regions['fleft'] > self.regions['fright'] or self.regions['left'] > self.regions['right']: # If wall is closer to the right, follow right
                            rospy.loginfo("wall follow right")
                            self.set_wall_follower_dir(False)
                    # If the robot has been stuck continue with the same wall follower to get out of a loop. 
                    self.wall_follow_start_point = self.position
                    self.wall_follow_closest_point = self.position
                    self.state_counter = 0
                     
                    self.change_state(Bug2State.WALL_FOLLOW)    # Robot enters wall follow, default = left direction
                    # Debug
                    rospy.loginfo('State is set to : {}'.format(self.state))
                
        # Change to Go To Point. 
        elif self.state == Bug2State.WALL_FOLLOW:  
            self.state_counter += 1 
            # Give the robot time to move out from the dist precision area. 
            # Check if the current position is close to the line. 

            if self.state_counter > STATE_COUNTER_LIMIT and self.distance_to_line(self.wall_follow_start_point, self.target_point, self.position) < DIST_PRECISION:
               
                # Check if there is nothing to crach in to in front of the robot. 
                if self.regions['fleft'] > 0.2 and self.regions['fright'] > 0.2 and self.regions['front'] > 0.2:
                    # If it has, change wall follower direction and change state to GO TO POINT  
                    currentPos_to_target = self.distance_points(self.position, self.target_point)
                    wallfollowStart_to_target = self.distance_points(self.wall_follow_start_point, self.target_point)
                    currentPos_to_wallfollowStart = self.distance_points(self.position, self.wall_follow_start_point)

                    # If the robot is further away from it's goal, it is likely to go the wrong way. 
                    if currentPos_to_target > wallfollowStart_to_target :
                        self.change_wall_follower_dir()
                        self.change_state(Bug2State.GO_TO_POINT)
                        self.robot_was_stuck_cnt = 1
                        # Debug
                        rospy.loginfo('State is set to : {}'.format(self.state))
                    # If the robot found the line closer to the target point. 
                    else: 
                        self.change_state(Bug2State.GO_TO_POINT)
                        self.robot_was_stuck_cnt = 0
                        # Debug
                        rospy.loginfo('State is set to : {}'.format(self.state))
                
            
        # If the state required does not exist send error message. 
        else: 
            rospy.logerr('[Bug 2] Invalid state.')

    # Change the bug2 state. 
    def change_state(self, state):
        """
        Change the bug2 state 
        :param state: desired Bug1State. 
        """

        if self.state == state: 
            return
        # Go to point 
        if state == Bug2State.GO_TO_POINT: 
            rospy.logdebug('[Bug 2] Swapping to Go To Point')
            self.state = state
            resp = self.start_go_to_point(True)
            resp = self.start_wall_follower(False) 

        # Start the Wall follower. 
        elif state == Bug2State.WALL_FOLLOW:
            rospy.logdebug('[Bug 2] Starting Wall Follow')
            self.state = state
            resp = self.start_go_to_point(False)
            resp = self.start_wall_follower(True)

        # Change state to Done
        elif state == Bug2State.DONE:
            rospy.logdebug('[Bug 2] Done')
            self.state = state
            resp = self.start_go_to_point(False)
            resp = self.start_wall_follower(False)
            self.pub_done_move_base_fake(True) # Tell move_base_fake that bug is done

    #### Callbacks. 

    # State callback used to start and stop the robot.  
    def callback_state(self, start_stop):
        """
        State callback used to start and stop the robot.
        :param start_stop: start stop command. 
        :type start_stop: SetBool
        """
        # If the robot should start and in not active
        if start_stop.data == True and not self.active: 
            self.active = True
            rospy.logdebug('[Bug 2]  Starting')
            if self.state == Bug2State.GO_TO_POINT:
                rospy.logdebug('[Bug 2]  Starting  Go To  Point')
                resp = self.start_go_to_point(True)
                resp = self.start_wall_follower(False)
        
        # If the robot should stop and is active. 
        elif start_stop.data == False and self.active: 
            self.active = False
            rospy.logdebug('[Bug 2]  Stopping')
            resp = self.start_go_to_point(False)
            resp = self.start_wall_follower(False)
            return SetBoolResponse(True, 'Stopping  Bug 2')

        else: 
            if start_stop.data==True: 
                return SetBoolResponse(False, 'Already  Doing  Bug 2')
            else: 
                return SetBoolResponse(False ,'Already  Stopped')

    # Set wall follower direction
    def set_wall_follower_dir(self, left_dir):
        if left_dir  == True:
            self.wall_follow_left_dir = True
            self.left_wall_follower(self.wall_follow_left_dir)
        else:
            self.wall_follow_left_dir = False
            self.left_wall_follower(self.wall_follow_left_dir)
    # Toggle wall follower direction
    def change_wall_follower_dir(self):
        if self.wall_follow_left_dir == True: 
            self.wall_follow_left_dir = False
            self.left_wall_follower(self.wall_follow_left_dir)
        else:
            self.wall_follow_left_dir = True
            self.left_wall_follower(self.wall_follow_left_dir)


    # Callback for the robot setpoint. 
    def callback_set_point(self, point_message): 
        """
        Callback for the robot setpoint.
        :param point_message: The point to move to. 
        :type point_message: SetPoint
        """
        self.target_point = point_message.point
         # Debug
        self.pub_debug.publish(self.target_point)
        resp = self.go_to_point_set_point(point_message.point)
        self.state = Bug2State.GO_TO_POINT
        rospy.logdebug('[Bug 2] Swapping to Go To Point')
        return  SetPointResponse(True ,'Target  Point  Set')

    # Callback for Odometry information 
    def callback_odometry(self, odom):
        """
        Callback for Odometry robot information 
        :param odom: Odometry information 
        :type odom: Odometry
        """
        # Update current position
        self.position = odom.pose.pose.position
        # Update current Yaw
        quaternion = (
            odom.pose.pose.orientation.x, 
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    
    # Callback information from the laster scan.
    # 
    def callback_laser_scan(self, scan):
        """
        Callback information from the laster scan.
        :param scan: Laser scan information. 
        :type scan: LaserScan
        """
        self.regions = {
        'right':   min(min(scan.ranges [270:305]) , 3.5),
        'fright':  min(min(scan.ranges [306:341]) , 3.5),
        'front':   min(min(min(scan.ranges [342:359]) , min(scan.ranges [0:17])), 3.5),
        'fleft':   min(min(scan.ranges [18:53]) , 3.5),
        'left':    min(min(scan.ranges [54:90]) , 3.5),
        }   


        # Dynamic laserscanner. Will always read the laser scan pointing toward the target point. 
        dynamic_range = 25
        dynamic_angle_deg = int(-self.yaw_error_to_point(self.position, self.target_point) * (180/math.pi))
        dynamic_max = dynamic_angle_deg + dynamic_range
        dynamic_min = dynamic_angle_deg - dynamic_range
        if dynamic_max < 0: 
            dynamic_max = dynamic_max + 360
        if dynamic_min < 0: 
            dynamic_min = dynamic_min + 360
        if dynamic_angle_deg < 0: 
            dynamic_angle_deg = dynamic_angle_deg + 360

        # 
        if dynamic_max > dynamic_min: 
            self.dynamic_region = min(min(scan.ranges [dynamic_min:dynamic_max]) , 3.5)
 
        # If the region is around 0/360
        else: 
            self.dynamic_region = min(min(min(scan.ranges [dynamic_max:dynamic_angle_deg+(360-dynamic_angle_deg)]) , min(scan.ranges [0:dynamic_min])), 3.5)

         
    
    ### Publishes 
    # Tell move_base_fake that bug is done 
    def pub_done_move_base_fake(self,is_done):
        move_base_fake_service = '/move_base_fake/is_bug_done/'
        rospy.wait_for_service(move_base_fake_service)
        try:
            # Create a service to tell move_base_fake that bug is done
            pub_bug_is_done = rospy.ServiceProxy(move_base_fake_service,SetBool)
            pub_bug_is_done(is_done)
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e


    ### Help functions. 

    # Normalizes the angle 
    # @param angle
    def normalise_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2* math.pi * angle) / (math.fabs(angle))
        return angle

    
    # Calculate the distance between two points. 
    # @param point1, first point 
    # @param point2, second point. 

    def distance_points(self, point1, point2): 
        dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
        return dist

    # Calculate distance to line. 
    def distance_to_line(self, start_point, end_point, current_point): 
        up_eq = math.fabs((end_point.y - start_point.y) * current_point.x - (end_point.x - start_point.x) * current_point.y + (end_point.x * start_point.y) - (end_point.y * start_point.x))
        lo_eq = math.sqrt(pow(end_point.y - start_point.y, 2) + pow(end_point.x - start_point.x, 2))
        distance = up_eq/lo_eq
        return distance

    # Calculate the angle between the robot pos and target point pos.
    def calc_angle_two_points(self, point1, point2):
        """
        Calculate the angle between two points around the Z axis. 
        :param point1: first point. 
        :param point2: Second point. 
        :return angle: the angle between the two points around the z axis. 
        """
        angle = 0
        delta_x = point2.x - point1.x 
        delta_y = point2.y - point1.y
        if delta_x != 0: 
            angle = math.atan(delta_y / delta_x)
           
        return angle
        
    # Calculate the orientation of the robot in relations to the target point. 
    #         
    def yaw_error_to_point(self, point1, point2):
        # calculate the angle between two points. 
        points_angle = self.calc_angle_two_points(point1, point2)
        error_angle = 0
        # Calculate the yaw error based on the quadrant. 
        quadrant = self.calc_quadrant(point1, point2)
        if quadrant == 1: 
            error_angle = self.yaw - points_angle
        elif quadrant == 2 or quadrant == 3: 
            error_angle = self.yaw - points_angle + math.pi
        elif quadrant == 4:
            error_angle = self.yaw - points_angle + 2 * math.pi
        return self.normalise_angle(error_angle)

        
    # Find quadrant a point is in in relations to another. 
    def calc_quadrant(self, init_point, target_point):
        """
        Calculates the quadrant the target point is in in relations to the init point.
              x
          Q1  |  Q4
        y ----+----
          Q2  |  Q3
        
        :param init_point: Initial point. Eg; robot pos
        :param target_point: Another point. Eg; the target of the robot. 
        """
        quadrant = 0
        d_x = target_point.x - init_point.x
        d_y = target_point.y - init_point.y
        # 1. Check the position quadrant. 
        if d_x >= 0 and d_y >= 0: 
            quadrant = 1
        elif d_x < 0 and d_y >= 0: 
            quadrant = 2
        elif d_x < 0 and d_y < 0: 
            quadrant = 3
        elif d_x >= 0 and d_y < 0: 
            quadrant = 4
        return quadrant



if __name__ == '__main__':
    print "Starting ROS Bug2 module"
    rospy.init_node('bug2_node', anonymous=True , log_level=rospy.DEBUG)
    bug2 = bug2_node()

    try: 
        rospy.spin()
    except KeyboardInterrupt: 
        print "Shutting down ROS Bug1 module"

