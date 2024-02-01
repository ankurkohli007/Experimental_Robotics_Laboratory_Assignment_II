#!/usr/bin/env python3

"""
.. module:: helper
   :platform: Unix
   :synopsis: A node equipped with Interface Helper and ActionClient Helper to facilitate the Finite State Machine by providing 
              information about the action client's status and executing battery-related stimuli.

.. moduleauthor:: Ankur Kohli & Prof. Luca Buoncompagni

Clients:
    :armor_client- A client designed for communication with the aRMOR server.
    :motion/planner- A client responsible for communicating with the planner server, enabling the planning of a random path with via points.
    :motion/controller- A client designed for communication with the controller server, responsible for following the path provided by the planner server.
Subscribers:
    :state/battery_low- The battery state (high or low) is broadcast or published.
Servers:
    :state/set_pose- A server responsible for updating or setting the current robot pose within the robot-states node.
    
"""

# Import ROS libraries.
import rospy
import random
import time
from actionlib import SimpleActionClient
import assignment_1

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock
from armor_api.armor_client import ArmorClient


from assignment_1 import architecture_name_mapper as anm

# Import ROS-based messages.
from std_msgs.msg import Bool
from assignment_1.msg import PlanAction, ControlAction
from assignment_1.srv import SetPose

client = ArmorClient("armor_client", "my_ontology")




class ActionClientHelper:
    """
    `ActionClientHelper` is a class or module likely designed to assist in the usage and management of action clients, typically in the context of a ROS (Robot Operating System) application. It may provide 
     functionality to interact with action servers and handle actions efficiently.
    """

    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        # Initialize the client's state, setting `_is_running`, `_is_done`, and `_results` to their initial values.
        self.reset_client_states()
        # Assign the name of the server that is intended to be called or invoked.
        self._service_name = service_name
        # Retrieve an existing mutex or create a new one if none exists.

        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        # Create an instance of a straightforward ROS-based action client.
        self._client = SimpleActionClient(service_name, action_type)
        # Assign the done and feedback callbacks specified by the class that uses this client.
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        # Pause execution and wait for the action server to become active or available.
        self._client.wait_for_server()

    def send_goal(self, goal):
        """
         In the ROS architecture, only one client can send a new goal to the action server at a given time, as the server must not be running when a new goal is provided, simplifying the handling of concurrent 
         requests.

        Args:
            goal(PlanGoal): The goal to be transmitted consists of two Points, one representing the start point and the other indicating the target point, both specified in (x, y) coordinates.

        Returns:
            None
        """

        if not self._is_running:
            # Initiate the action server
            self._client.send_goal(goal,
                                   done_cb = self.done_callback_,
                                   feedback_cb = self.feedback_callback_)
            # Set the client's states
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            print("Warning send a new goal, cancel the current request first!")

    def cancel_goals(self):
        """
        Function to halt the computation of the action server.
        
        Args:
            None
            
        Returns:
            None
        """
        
        if self._is_running:
            # Halt the computation
            self._client.cancel_all_goals()
            # Reset the client's state
            self.reset_client_states()
        else:
            print("Warning cannot cancel a not running service!")

    def reset_client_states(self):
        """
        A function that resets the client's stored state variables within this class.
        
        Args:
            None
            
        Returns:
            None
        """

        self._is_running = False
        self._is_done = False
        self._results = None

    def feedback_callback_(self, feedback):
        """
        A function invoked when the action server needs to provide feedback to the client.
        
        Args:
            feedback: feedback message to be sent to the client
            
        Returns:
            None
        """

        self._mutex.acquire()
        try:
            # At some point, invoke the method offered by the node that employs this action client to handle feedback.
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
        finally:
            # Release the mutex to enable ROS-based threads waiting on the same mutex to proceed.
            self._mutex.release()

    def done_callback_(self, status, results):
        """
       A function that is triggered when the action server completes its computation.
        
        Args:
            status: status of the action server
            results: results from the action server
            
        Returns:
            None
        """
        
        self._mutex.acquire()
        try:
            # Set the client's states
            self._is_running = False
            self._is_done = True
            self._results = results
            # Invoke the method supplied by the node employing this action client to handle a result.
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
        finally:
            self._mutex.release()

    def is_done(self):
        """
        A function that returns `True` if the action server has completed its computation and `False` otherwise.
        
        Args:
            None
            
        Returns:
            Bool: Returns `True` if the action server has completed its computation and `False` otherwise.
        """
        
        return self._is_done

    def is_running(self):
        """
        A function that returns `True` if the action server is currently running and `False` otherwise.
        
        Args:
            None
            
        Returns:
            Bool: Indicates `True` if the action server is in an active state and `False` if it's not running.
        """

        return self._is_running

    def get_results(self):
        """
        A function that retrieves the result generated by the action server.
        
        Args:
            None
            
        Returns:
            Result of the action server, if any, 'None' otherwise
        """

        if self._is_done:
            return self._results
        else:
            print("Error: cannot get result")
            return None

class InterfaceHelper:
    """
    This class serves to separate the implementation of the Finite State Machine from the stimuli that trigger state transitions. It handles synchronization with subscribers and action servers, enhancing modularity 
    and organization in the system architecture.
    """

    def __init__(self):
        # Establish a shared mutex to facilitate synchronization between action clients and subscribers, ensuring coordinated access to shared resources.
        self.mutex = Lock()
        # Define or specify the initial state for a process or system.
        self.reset_states()
        # Specify the callback function that is linked to the ROS subscribers monitoring low battery conditions.
        rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self.battery_callback_)
        # Create instances of clients that will interact with both the plan and control action servers.
        self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)

    def reset_states(self):
        """
        A function designed to reset the stimulus related to the battery, stored as a state variable, under the assumption that no states of the Finite State Machine run concurrently.
        
        Args: 
            None
        
        Returns:
            None
        """

        self._battery_low = False

    def battery_callback_(self, msg):
        """
        A subscriber function responsible for receiving messages published by the `robot_state` node on the `/state/battery_low/` topic.
        
        Args:
            msg(Bool): battery status
            
        Returns:
            None
        """

        # Secure the mutex to ensure synchronization with other subscribers and action clients.
        self.mutex.acquire()
        try:
            # Retrieve the battery level and update the relevant state variable stored within this class.
            self._battery_low = msg.data
        finally:
            # Release the mutex to potentially unblock other subscribers or action servers that may be in a waiting state.
            self.mutex.release()

    def is_battery_low(self):
        """
        A function designed to retrieve the state variable related to the battery level encoded within this class.
        
        Args:
            None
        
        Returns:
            Bool: Returns `True` if the battery is in a low state, and `False` otherwise.
        """

        return self._battery_low

    @staticmethod
    def init_robot_pose(point):
        """
        A function responsible for updating the robot's current position, stored within the `robot_state` node.
        
        Args:
            point(Point): A point that represents the robot's pose using (x, y) coordinates.
            
        Returns:
            None
        """

        # Pause and wait for the server to complete its initialization process.
        rospy.wait_for_service(anm.SERVER_SET_POSE)
        try:
            # Invoke the service to establish and update the robot's current position.
            service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
            service(point)
            print("Setting initial robot position")
        except rospy.ServiceException as e:
            print("Cannot set current robot position")
            
class KnowledgeGraphHelper:
    def __init__(self):
        self.survillancerobot = "Robot1"
        self.survillancerobot_position = 'robot_coordinates'
        self.survillancerobot_reachable_rooms = 'reachable_rooms'
        
    def list_section_formatter(self, initiallocation, initial, boundary):
        currentlocation = []
        for location in initiallocation:
            start_index = location.find(initial)
            end_index = location.find(boundary, start_index + len(initial))
            if start_index != -1 and end_index != -1:
                extracted_substring = location[start_index + len(initial):end_index]
                currentlocation.append(extracted_substring)
        return currentlocation
            
    def obtainLocation(self, location):
        if location == 'corridor':
            return self.list_section_formatter(client.query.ind_b2_class('CORRIDOR'), '#', '>')
        if location == 'urgent':
            return self.list_section_formatter(client.query.ind_b2_class('URGENT'), '#', '>')
        if location == self.survillancerobot_position:
            return self.list_section_formatter(client.query.objectprop_b2_ind('isIn', self.survillancerobot), '#', '>')[0]
        if location == self.survillancerobot_reachable_rooms:
            return self.list_section_formatter(client.query.objectprop_b2_ind('canReach', self.survillancerobot), '#', '>')

    def choose_destination(self):
        corridors_list = self.obtainLocation('corridor')
        current_position = self.obtainLocation(self.survillancerobot_position)
        print("\nCurrent position of robot is:", current_position)
        
        possible_destinations = self.obtainLocation(self.survillancerobot_reachable_rooms)
        print("Reachable Positions:", possible_destinations)
        urgent_rooms = self.obtainLocation('urgent')
        print("URGENCY ROOMS:", urgent_rooms)
        
        ReachableUrgent_room = set(possible_destinations) & set(urgent_rooms)
        
        if not ReachableUrgent_room:
            ReachableCorridor_room = set(possible_destinations) & set(corridors_list)
            if not ReachableCorridor_room:
                target = random.choice(possible_destinations)
            else:
                target = random.choice(list(ReachableCorridor_room))
        else:
            oldest = None
            oldest_time = float('inf')
            for room in ReachableUrgent_room:
                last_visit = client.query.dataprop_b2_ind('visitedAt', room)
                last_visit = int(self.list_section_formatter(last_visit, '"', '"')[0])
                if last_visit < oldest_time:
                    oldest_time = last_visit
                    oldest = room
            target = oldest

        print("Target to be Reached:", target)
        return current_position, target

    def relocate(self, target, current_position):
        client.manipulation.replace_objectprop_b2_ind('isIn', self.survillancerobot, target, current_position)
        print(f"Robot moved to {target} and monitoring")

        current_time = str(int(time.time()))

        last_change_prop = 'now'
        last_change_values = client.query.dataprop_b2_ind(last_change_prop, self.survillancerobot)
        last_change = str(self.list_section_formatter(last_change_values, '"', '"')[0])

        client.manipulation.replace_dataprop_b2_ind(last_change_prop, self.survillancerobot, 'Long', current_time, last_change)

        corridors = set(self.obtainLocation('corridor'))
        if target not in corridors:
           visit_time_prop = 'visitedAt'
           last_visit_values = client.query.dataprop_b2_ind(visit_time_prop, target)
           last_visit = str(self.list_section_formatter(last_visit_values, '"', '"')[0])
           client.manipulation.replace_dataprop_b2_ind(visit_time_prop, target, 'Long', current_time, last_visit)

        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()
    def headToRechargeStation(self, robot_location):
        """
        A function responsible for guiding the robot to its designated charging room location.

        Args:
            robot_location(str): The current robot position obtained E

        """
        client.manipulation.replace_objectprop_b2_ind('isIn', self.survillancerobot, "E", robot_location)
        client.utils.sync_buffered_reasoner()
        print("Robot Reached Charging Room and Changing")


