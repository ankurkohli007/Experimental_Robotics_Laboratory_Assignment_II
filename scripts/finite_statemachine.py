#!/usr/bin/env python3
"""
.. module:: finite_statemachine
   :platform: Unix
   :synopsis: A finite state machine is a computational model that represents an entity's behaviour through a finite number of states 
              and transitions between them. Also, Python module for implementing the Finite State Machine.

.. moduleauthor:: Ankur Kohli

In ROS (Robot Operating System), a finite state machine (FSM) is a programming paradigm that helps control a robot's behaviour by dividing its operation into distinct states and managing transitions between these states, allowing for organized and responsive robot control. Moreover, ROS node for implementing the Finite State Machine.


This the core of all ROS nodes, the finite state machine, plays a pivotal role in orchestrating state changes, enabling the robot to achieve desired conditions by coordinating actions across two action servers.

Service: 
    /server_name: To visualize the finite state machine, initiate and activate the introspection server using sm_introspection.
 
"""

#Import ROS libraries; imports various modules, including SMACH, ROS packages, and custom helpers, to create a state machine for a robot control application.

import smach
import rospy
import random
import smach_ros
import time
import assignment_1
from smach import StateMachine,State
from helper import InterfaceHelper
from helper import KnowledgeGraphHelper
from load_environment import LoadKnowledgeGraph
from assignment_1.msg import Point, ControlGoal, PlanGoal
from armor_api.armor_client import ArmorClient
from assignment_1 import architecture_name_mapper as anm
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import *
from geometry_msgs.msg import *
import math
from std_msgs.msg import *
from nav_msgs.msg import *
from move_base_msgs.msg import *
from tf import transformations
import actionlib
from assignment2.srv import *

reached=False


#list of possible states in the machine
INITIALIZING = 'LOADING_MAP'
CHOOSING_PATH = 'DETERMINING_CORRIDOR_OR_ROOM'
URGENT_SELECTION = 'CHOOSING_URGENT_ROOM'
MOVING_TO_CORRIDOR = "ON_CORRIDOR_MOVE"
MOVING_TO_URGENT = "ON_URGENT_MOVE"
SURVEY_IN_PROGRESS = 'EVALUATING_CORRIDOR_ROOM'
RECHARGING_BATTERY = 'RECHARGING_STATION'
START_URGENT_OP='INITIATE_URGENT_OPERATION'
START_CORRIDOR_OP='INITIATE_CORRIDOR_OPERATION'
IN_ROOM='WITHIN_ROOM'


# list of transition states
INITIALIZATION_COMPLETE = 'MAP_LOADED'
CORRIDOR_CHOICE_MADE = 'CORRIDOR_SELECTED'
MOVING_IN_CORRIDOR = 'CORRIDOR_TRAVERSAL'
URGENT_MOVE = 'URGENT_ROOM_TRAVERSAL'
URGENT_CHOICE_MADE = 'URGENT_ROOM_SELECTED'
LOW_BATTERY_WARNING = 'BATTERY_DEPLETION_WARNING'
RECHARGING_IN_PROGRESS = 'BATTERY_RECHARGING'
RECHARGING_COMPLETE = 'BATTERY_RECHARGED'

# Sleeping time (in seconds) of the waiting thread to allow the computations
# for getting stimulus from the other components of the architecture.
LOOP_SLEEP_TIME = 0.3


# Define constant for state for scanning environment
SCAN_ENVIRONMENT='SCAN'

# Define constant for transition state for scanning environment
UPDATED_TRANSITION_STATE = 'SCANNED'
    
def rotate(w):
    """
    Function responsible for the robot to rotate.
    
    Args:
        w: Angular velocity

    """
    cmd = Twist()
    cmd.angular.z = w
    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel_publisher.publish(cmd)

def rotate_arm_1(w):
    """
    Function responsible for rotating arm_1 to scan the markers
    """
    rot_publisher = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=1)
    position = w
    rot_publisher.publish(position)

def rotate_arm_2(w):
    """
    Function responsible for rotating arm_2 to scan the markers
    """
    rot_publisher1 = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=1)
    position1 = w
    rot_publisher1.publish(position1)

def rotate_base(w):
    """
    Function responsible for rotating arm_base to scan the markers
    """
    rot_publisher2 = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1)
    position2 = w
    rot_publisher2.publish(position2)

    
def marker_CallBack(data):
    """
    Function responsible for executing the CallBack when the /scan_marktopic is published.

    Args:
        data: Marker data

    """
    global get_id
    get_id = rospy.ServiceProxy("/room_info", RoomInformation)
    req = data.data
    res = get_id(req)
    con = res.connections
    print("ROOM ID:", req)
    print("ROOM NUMBER:", res.room)
    print("X:", res.x)
    print("Connections:", con)

class Environment_Scanning(smach.State):
    """
    A class to load the ontology map to the robot.
    """

    def __init__(self):
        State.__init__(self, outcomes=[UPDATED_TRANSITION_STATE])
            
    def execute(self, userdata):
        """
        Function responsible for the robot to scan the markers and get information about the specific room and their connections.

        Args:
            userdata: Not used

        Returns:
               UPDATED_TRANSITION_STATE (str): Transits to INITIALIZING where the map will be loaded
        """

        while not rospy.is_shutdown():
            rotations = [
                [0, 0, 0],
                [-1.00, -1.09, 0],
                [1.00, -0.12, 0],
                [1.00, -0.12, 1.00],
                [-0.60, -1.34, 0.57],
                [-0.60, -1.34, 0.50],
                [0.90, 0.50, 1.56],
                [0, 0, 0]
            ]

            for rotation in rotations:
                rotate_arm_1(rotation[0])
                rospy.sleep(2)
                rotate_arm_2(rotation[1])
                rospy.sleep(2)
                rotate_base(rotation[2])
                rospy.sleep(2)

            return UPDATED_TRANSITION_STATE


class EnvironmentInitializer(smach.State):
    """
    A class to initialize the environment simply we can say upload the ontology map to the robot's system.
    """
    def __init__(self):

        
        State.__init__(self, outcomes = [INITIALIZATION_COMPLETE])

    def execute(self, userdata):
        """
        The loading of the environment is managed by a function that invokes the `LoadKnowledgeGraph()` function, responsible for creating the environment, 
        with no need for input data from other states as the `userdata` parameter is unused.

        Args:
            userdata: not used

        Returns:
            INITIALIZATION_COMPLETE(str): transits to NORMAL_STATE which is a nested state_machine

        """
        
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        my_goal=MoveBaseActionGoal()
        print("\nInitiating map loading sequence: Robot is moving to Corridor E to load the ontology map.")
        my_goal.goal.target_pose.header.frame_id = "odom";
        my_goal.goal.target_pose.pose.orientation.w = 1;
        my_goal.goal.target_pose.pose.position.x = 1.5
        my_goal.goal.target_pose.pose.position.y = 8.0
        client.wait_for_server()
        client.send_goal(my_goal.goal)
        client.wait_for_result()
        if  client.get_state()==GoalStatus.SUCCESSFUL:
            LoadKnowledgeGraph()
            print("ONTOLOGY MAP LOADED")
            return INITIALIZATION_COMPLETE
           
class CorridorRoomDecisionMaker(smach.State):
      """
      This class refers to a component or module which is responsible for making decisions related to corridors and rooms robot should reach, often used in navigation or robotics contexts.
      """

      def __init__(self, interface_helper, knowledge_helper):
        
        # Obtain a connection to the interfaces linking to other nodes within the architecture.
        self._helper = interface_helper
        self._knowledge = knowledge_helper
        # Retrieve the environment size from ROS parameters.
        self.environment_size = rospy.get_param('config/environment_size')
        State.__init__(self, outcomes = [RECHARGING_IN_PROGRESS, CORRIDOR_CHOICE_MADE, URGENT_CHOICE_MADE], output_keys = ['robot_position', 'target','random_plan'])

      def execute(self, userdata):
        """
        This function manages transitions between the `CHOOSING_PATH` state and either `RECHARGING_BATTERY`, `MOVING_TO_CORRIDOR`, or `URGENT_SELECTION` states, based on conditions: low battery triggers
        `RECHARGING_BATTERY`, corridor target leads to `MOVING_TO_CORRIDOR`, and an urgent room prompts `URGENT_SELECTION`, utilizing a planner for path planning.

        Args:
            userdata: utilized in output_keys to convey information to subsequent states in the process.

        Returns:
            RECHARGING_IN_PROGRESS (str): transition to RECHARGING_BATTERY.
        
        Returns:
            URGENT_CHOICE_MADE (str): transition to the URGENT_SELECTION.
            
        .Returns:
            CORRIDOR_CHOICE_MADE (str): transition to the CHOOSING_PATH.

        """

        # Specify a target destination as a random point, and plan a path to reach it by navigating through intermediate via-points.
        goal = PlanGoal()
        goal.target = Point(x = random.uniform(0, self.environment_size[0]),
                            y = random.uniform(0, self.environment_size[1]))
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        my_goal=MoveBaseActionGoal()
        robot_position, target = self._knowledge.choose_destination()
        userdata.robot_position = robot_position
        userdata.target = target
        # Initiate the planner action server.
        self._helper.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            # Obtain the mutex to ensure data consistency when dealing with ROS subscription threads overseen by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery reaches a low level, terminate the control action server and trigger the transition labeled `BATTERY_DEPLETION_WARNING`.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.planner_client.cancel_goals()
                    print("\nLow battery alert: Initiating recharge sequence. Heading to the recharge room.")
                    my_goal.goal.target_pose.header.frame_id = "odom";
                    my_goal.goal.target_pose.pose.orientation.w = 1;
                    my_goal.goal.target_pose.pose.position.x = 1.5
                    my_goal.goal.target_pose.pose.position.y = 8.0
                    client.wait_for_server()
                    client.send_goal(my_goal.goal)
                    client.wait_for_result()
                if  client.get_state()==GoalStatus.SUCCEESSFUL:
                    self._knowledge.headToRechargeStation(robot_position)
                    return RECHARGING_IN_PROGRESS
                # When the target is identified as a room, the planner cancels any existing goals and initiates the transition labeled `URGENT_CHOICE_MADE`.

	
                 # Upon completion of the planner's computation, initiate the transition labeled `CORRIDOR_CHOICE_MADE`.
                if self._helper.planner_client.is_done():
                    if target=='R1' or target=='R2' or target=='R3' or target=='R4':
                          userdata.random_plan = self._helper.planner_client.get_results().via_points
                          return URGENT_CHOICE_MADE
                    else:
                          userdata.random_plan = self._helper.planner_client.get_results().via_points
                          return CORRIDOR_CHOICE_MADE

            finally:
                # Free the mutex to unblock any `self._helper` subscription threads that might be in a waiting state.
                self._helper.mutex.release()
            # Pause briefly, allowing `self._helper` to process stimuli, if any, within a reasonably short timeframe.
            rospy.sleep(LOOP_SLEEP_TIME)


class CorridorPathFollower(smach.State):
    """
    A class designed to facilitate the robot's movement to a previously obtained corridor.
    """

    def __init__(self, interface_helper, knowledge_helper):
        # Establish connections with interfaces linking to other nodes within the architecture.
        self._helper = interface_helper
        self._knowledge = knowledge_helper

        State.__init__(self, outcomes = [RECHARGING_IN_PROGRESS, MOVING_IN_CORRIDOR], input_keys = [ "random_plan",'robot_position', 'target'], output_keys = ['robot_position'])

    def execute(self, userdata):
        """
        This function manages transitions between the `MOVING_TO_CORRIDOR` state and either `RECHARGING_BATTERY` or `CHOOSING_PATH`, where it guides the robot to a designated corridor, ensures it remains 
        there for a specified duration, and employs a controller for robot movement.

        Args:
            userdata: utilize input_keys to retrieve data and output_keys to transmit data between components or states in the system.
        Returns:
            RECHARGING_IN_PROGRESS(str): transition to the RECHARGING_BATTERY.

        Returns:
            MOVING_IN_CORRIDOR(str): transition to STATE_DECISION_CORRIDOR.

        """
        plan = userdata.random_plan
        # Commence the action server responsible for guiding the robot along the planned via-points trajectory.
        goal = ControlGoal(via_points = plan)
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        my_goal=MoveBaseActionGoal()
        self._helper.controller_client.send_goal(goal)
        robot_position = userdata.robot_position
        target = userdata.target

        while not rospy.is_shutdown():
            # Obtain the mutex to ensure data consistency when dealing with ROS subscription threads overseen by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # In the event of low battery, terminate the control action server and proceed with the transition labeled `BATTERY_DEPLETION_WARNING`.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    print("\nLow battery alert: Initiating recharge sequence. Moving to the recharge room.")
                    my_goal.goal.target_pose.header.frame_id = "odom";
                    my_goal.goal.target_pose.pose.orientation.w = 1;
                    my_goal.goal.target_pose.pose.position.x = 1.5
                    my_goal.goal.target_pose.pose.position.y = 8.0
                    client.wait_for_server()
                    client.send_goal(my_goal.goal)
                    client.wait_for_result()
                
                if  client.get_state()==GoalStatus.SUCCEESSFUL:
                          self._ontology.go_to_recharge(robot_position)
                          return RECHARGING_IN_PROGRESS   
                
                # Once the controller has completed its computations, trigger the `went_random_pose` transition, which is linked to the `repeat` transition.
                if self._helper.controller_client.is_done():
                    if target=="C1":
                       my_goal.goal.target_pose.header.frame_id = "odom";
                       my_goal.goal.target_pose.pose.orientation.w = 1;
                       my_goal.goal.target_pose.pose.position.x = -1.5
                       my_goal.goal.target_pose.pose.position.y = 0.0
                       client.wait_for_server()
                       client.send_goal(my_goal.goal)
                       client.wait_for_result()
                    elif target=="C2":
                       my_goal.goal.target_pose.header.frame_id = "odom";
                       my_goal.goal.target_pose.pose.orientation.w = 1;
                       my_goal.goal.target_pose.pose.position.x = 3.5
                       my_goal.goal.target_pose.pose.position.y = 0.0
                       client.wait_for_server()
                       client.send_goal(my_goal.goal)
                       client.wait_for_result()
               
                    #Robot is staying in the corridor for {specific_time_period} seconds
                    
                if  client.get_state()==GoalStatus.SUCCEESSFUL:
                    self._knowledge.headToRechargeStation(robot_position)
                    rotate(3)
                    rospy.sleep(anm.MONITOR_TIME)
                    return MOVING_IN_CORRIDOR
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)






class UrgentPathfinder(smach.State):
      """
      A class to move the robot to a Urgent room which has been acquired
      """

      def __init__(self, interface_helper, knowledge_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._knowledge = knowledge_helper

        self.environment_size = rospy.get_param('config/environment_size')
        State.__init__(self, outcomes = [RECHARGING_IN_PROGRESS, URGENT_MOVE], input_keys = ["random_plan",'robot_position', 'target'], output_keys = ['robot_position'])

      def execute(self, userdata):
        """
        This function manages the transition from the `MOVING_TO_URGENT` state to either `RECHARGING_BATTERY` or `URGENT_SELECTION`. Its role is to guide the robot to a predetermined corridor and ensure 
        it remains   there for a specified duration, utilizing a controller for robot movement.

        Args:
            userdata: for input_keys and output_keys to get data and pass data.
        
        Returns:
            RECHARGING_IN_PROGRESS(str): transition to the RECHARGING_BATTERY.

        Returns:
            URGENT_MOVE(str): transition to STATE_DECISION_URGENT.

        """
        #plan = userdata.random_plan
        # Start the action server for moving the robot through the planned via-points.
        #goal = ControlGoal(via_points = plan)
        #self._helper.controller_client.send_goal(goal)
        #robot_position = userdata.robot_position
        #target = userdata.target
        
        # Specify a random destination point to be reached by planning a path through a series of via-points
        goal = PlanGoal()
        goal.target = Point(x = random.uniform(0, self.environment_size[0]),
                            y = random.uniform(0, self.environment_size[1]))
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        my_goal=MoveBaseActionGoal()
        robot_position, target = self._knowledge.choose_destination()
        userdata.robot_position = robot_position
        userdata.target = target
        # here we invoking the planner action server.
        self._helper.planner_client.send_goal(goal)

        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `BATTERY_DEPLETION_WARNING` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    print("\nThe robot is currently low on battery and is proceeding to the recharge room for recharging.")
                    my_goal.goal.target_pose.pose.orientation.w = 1;
                    my_goal.goal.target_pose.pose.position.x = 1.5
                    my_goal.goal.target_pose.pose.position.y = 8.0
                    client.wait_for_server()
                    client.send_goal(my_goal.goal)
                    client.wait_for_result()
                if  client.get_state()==GoalStatus.SUCCEESSFUL:
                          self._knowledge.headToRechargeStation(robot_position)
                          return RECHARGING_IN_PROGRESS
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if  target=='C1' or target=='C2' or target=='E':
                    self._helper.controller_client.cancel_goals()
                    return CORRIDOR_CHOICE_MADE
#If the controller completes its computation, transition to the went_random_pose state, which is associated with the repeat transition.     
                if self._helper.controller_client.is_done():
                   userdata.random_plan = self._helper.controller_client.get_results().via_points
                   return URGENT_CHOICE_MADE
                    
            finally:
                # Unlock the mutex to allow any waiting `self._helper` subscription threads to resume execution.
                self._helper.mutex.release()
            # Pause briefly to permit `self._helper` to process stimuli, which may occur at some point.
            rospy.sleep(LOOP_SLEEP_TIME)




class ChargeManagement(State):
    """
    This a class responsible for overseeing and managing the charging process for a robot, ensuring it effectively maintains its power source to continue its operations autonomously or as directed.
    """

    def __init__(self, interface_helper, knowledge_helper):
        # Obtain access points or connections to the communication interfaces with other nodes within the architecture.
        self._helper = interface_helper
        self._knowledge = knowledge_helper
        # Configure this state by defining the permissible transitions, which represent valid outcomes from the `execute` function.
        State.__init__(self, outcomes = [RECHARGING_COMPLETE])

    # Specify the action or task that occurs whenever a transition is taken to enter this state.
    # Please note that the `userdata` input parameter remains unused, as it doesn't require data from other states.

    def execute(self, userdata):
        """
        This function manages the transition from the `RECHARGING_BATTERY` state to the `NORMAL_STATE` state, 
        with the robot remaining in this state until the battery reaches an adequate level.
 
        Args:
            userdata: not used

        Returns:
            RECHARGING_COMPLETE(str): transition to NORMAL_STATE

        """
        while not rospy.is_shutdown():  # Pause and await input or triggering signals from other nodes within the architecture.
            # Obtain the mutex to ensure data consistency when dealing with the ROS subscription threads under the supervision of `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is no longer in a low state, proceed with the transition labeled `charged`.
                if not self._helper.is_battery_low():
                    self._helper.reset_states()  # Reset the state variable related to the stimulus.
                    return RECHARGING_COMPLETE
            finally:
                # Release the mutex to allow any waiting `self._helper` subscription threads to resume their operations.
                self._helper.mutex.release()
            # Pause briefly to provide time for `self._helper` to process any stimuli that may arrive at some point.
            rospy.sleep(LOOP_SLEEP_TIME)

def main():
    """
    This function constructs the state machine and specifies all the transitions, including the creation of two nested state machines.
    """
    rospy.init_node('Finite_state_machine', log_level = rospy.INFO)
    rospy.Subscriber("/scan_marker", Int64, marker_CallBack)
    rospy.wait_for_service("/room_info")
    get_id = rospy.ServiceProxy("/room_info",RoomInformation)
    # Initialize classes responsible for managing communication interfaces with other nodes within the architecture.
    helper = InterfaceHelper()
    knowledge = KnowledgeGraphHelper()
    sm_main = StateMachine([])
    
    # Create and run the state machine
   # sm = smach.StateMachine(outcomes=['outcome'])
   # with sm:
    #    StateMachine.add('SCAN_ENVIRONMENT', Environment_Scanning(), transitions={UPDATED_TRANSITION_STATE: 'INITIALIZING'})

    # outcome = sm.execute()

    with sm_main:

        StateMachine.add(SCAN_ENVIRONMENT, Environment_Scanning(), 
                         transitions = {UPDATED_TRANSITION_STATE: INITIALIZING})

        StateMachine.add(INITIALIZING, EnvironmentInitializer(),
                         transitions = {INITIALIZATION_COMPLETE: CHOOSING_PATH})
        
        sm_corridor_room = StateMachine(outcomes=[LOW_BATTERY_WARNING])

        with sm_corridor_room:
            StateMachine.add(CHOOSING_PATH, CorridorRoomDecisionMaker(helper, knowledge),
                            transitions = {RECHARGING_IN_PROGRESS : LOW_BATTERY_WARNING,
                                            CORRIDOR_CHOICE_MADE: MOVING_TO_CORRIDOR,URGENT_CHOICE_MADE:MOVING_TO_URGENT})
            StateMachine.add(MOVING_TO_CORRIDOR,CorridorPathFollower(helper, knowledge),
                            transitions = {RECHARGING_IN_PROGRESS : LOW_BATTERY_WARNING,
                                            MOVING_IN_CORRIDOR: CHOOSING_PATH})
            StateMachine.add(MOVING_TO_URGENT, UrgentPathfinder(helper, knowledge),
                            transitions = {RECHARGING_IN_PROGRESS : LOW_BATTERY_WARNING,
                                            URGENT_MOVE: CHOOSING_PATH})                                            
                                            
       
           

           
        StateMachine.add(CHOOSING_PATH, sm_corridor_room,
                         transitions={LOW_BATTERY_WARNING: RECHARGING_BATTERY})
            
        StateMachine.add(RECHARGING_BATTERY, ChargeManagement(helper, knowledge),
                            transitions = {RECHARGING_COMPLETE: CHOOSING_PATH})
    # Establish and activate the introspection server to enable visualization.
    sis = smach_ros.IntrospectionServer('sm_introspection', sm_main, '/SM_ROOT')
    sis.start()

    # Run or execute the state machine to initiate its operation.
    outcome = sm_main.execute()

    # Remain in a waiting state, expecting a "Ctrl+c" command to halt the application.
    rospy.spin()
    sis.stop()
   
if __name__ == "__main__":
    main()
