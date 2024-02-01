#!/usr/bin/env python3

"""
.. module:: robot_states
   :platform: Unix
   :synopsis: robot_states typically refers to a component or module in a robotic system responsible for managing and updating 
              various states and information related to the robot, such as its pose, battery level, and operational status. Also, A  
              Python module responsible for publishing the state of the battery, typically utilizing a ROS publisher to broadcast 
              this information.

.. moduleauthor:: Ankur Kohli & Prof. Luca Buoncompagni


Publisher:
    /state/battery_low to obtain the battery status.

Servers: 
    /state/set_pose: A server responsible for updating or configuring the current pose of the robot.
    /state/get_pose: A server designed to retrieve and provide the current pose of the robot.
    
"""

import threading
import rospy
import assignment_1
from helper import InterfaceHelper
from assignment_1 import architecture_name_mapper as anm
from std_msgs.msg import Bool
from assignment_1.msg import Point
from assignment_1.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse
import random

# A tag used for identifying the source or producer of logs, often utilized for categorization or tracking purposes in logging systems.
LOG_TAG = anm.NODE_ROBOT_STATE

# A constant value that represents the amount of time the battery can be used before requiring a recharge or replacement.


class RobotState:
    """
    The Node Manager class encapsulates functionality including two services for obtaining and configuring the robot's current pose, along with a publisher to announce low battery conditions.
    """

    def __init__(self):
        # Initialize or set up this node, typically involving the configuration and setup of its services and publishers.
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        # Initialize or set the robot's current position, often carried out at the beginning of a robotic task or operation.
        self._pose = None
        # Set the initial value or state for the battery level, usually performed at the start of a system or application.
        self._battery_low = False
        self._randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
        if self._randomness:
        	self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [15.0, 60.0])
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose)
        # Initiate the publisher's operation on a distinct or separate thread, allowing it to run concurrently with other parts of the program.
        th = threading.Thread(target=self.is_battery_low_)
        th.start()
        # Record or log informational data, often for the purpose of monitoring, troubleshooting, or analysis in a software system or application.
        log_msg = (f'Initialise node `{anm.NODE_ROBOT_STATE}` with services `{anm.SERVER_GET_POSE}` and '
                   f'`{anm.SERVER_SET_POSE}`, and topic {anm.TOPIC_BATTERY_LOW}.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def set_pose(self, request):
        """
        The implementation of the `robot/set_pose` service, which typically handles requests to set or update the robot's current pose.

        Args:
            request(Point): input parameter is the current robot pose to be set,

        Returns:
            SetPoseResponse(): This server returns an empty `response`
        """

        if request.pose is not None:
            # Save or store the newly provided current robot position, typically within the system or application for future reference and use.
            self._pose = request.pose
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        return SetPoseResponse()

    def get_pose(self, request):
        """
        A function that provides the implementation for the 'state/get_pose' service, typically responsible for responding to requests for retrieving the current robot pose.
        
        Args:
            request: given by the client as empty, it is not used
            
        Returns:
            response(Point): current robot position
        """

        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        # Generate a response containing the current robot pose and return it to fulfill the `state/get_pose` service request.
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    def is_battery_low_(self):
        """
        A function that publishes changes in the battery level, typically generating random battery level fluctuations and broadcasting them to indicate varying battery conditions.
        Args:
            None
            
        Returns:
            None
        """

        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        
        if self._randomness:
        	self.battery_notifier_(publisher)

    def battery_notifier_(self, publisher):
        """
        Publish battery level changes using a separate thread, introducing delays within the specified interval [`self._random_battery_time[0`], `self._random_battery_time[1]`) between updates.
        
        Args:
            publisher(Publisher): The publisher responsible for transmitting messages, often used to disseminate data or information to other parts of a system or external components.
            
        Returns:
            None
        """
        delay=0
        while not rospy.is_shutdown():
            # Publish the battery level status, indicating `True` if the battery is low and `False` if it is not.
            publisher.publish(Bool(self._battery_low))
            # Simulate the consumption or usage of the battery, typically for testing or demonstration purposes in a software or hardware system.
            if self._battery_low:
            
               print("Robot battery is low after",delay,"seconds")
               battery_percentage = 0

               
               while battery_percentage < 100:
                   battery_percentage += 10 

                   print(f"Robot Charging - Battery Percentage: {battery_percentage}%")
               
               print("Robot Charging Complete")
               
            else:
            	print("Robot battery is full",delay,"seconds")
            if self._randomness:
            	delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
            rospy.sleep(delay)
            # Alter the state or status of the battery, often indicating shifts between a low and normal state or other relevant conditions.
            self._battery_low = not self._battery_low

if __name__ == "__main__":
    # Create an instance of the Node Manager class, allowing for the management of services and publishers related to robot pose and battery status.
    RobotState()

    # Specify or define the helper, which could refer to a utility or support module assisting in various tasks within a system or application.
    helper = InterfaceHelper()

    # Set up or initialize the initial position of the robot, often performed at the start of a robotic task or application.
    robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
    helper.init_robot_pose(Point(x=robot_pose_param[0], y=robot_pose_param[1]))

    rospy.spin()
