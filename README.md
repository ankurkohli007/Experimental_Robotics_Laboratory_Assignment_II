[Experimental Robotics Laboratory](https://corsi.unige.it/en/off.f/2022/ins/60244)<br>
**Programmer:** [Ankur Kohli](https://github.com/ankurkohli007)<br>
[M.Sc Robotics Engineering](https://corsi.unige.it/corsi/10635)<br>
[University of Genoa (UniGe), Italy](https://unige.it/en)<br>
**Supervisor:** [Prof. Carmine Tommaso Recchiuto](https://rubrica.unige.it/personale/UkNDWV1r) 

# Assignment 2: Integration of Autonomous Surveillance Robot Architecture with Robotic Simulation for Indoor Environment Mapping and Patrolling

## Abstract

The second assignment in the Experimental Robotics Laboratory course focuses on the seamless integration of an adaptive autonomous surveillance robot architecture into a robotic simulation environment designed for indoor monitoring. The assignment involves the incorporation of a robot into the simulation, positioned at specific coordinates. The robot is tasked with autonomously building a semantic map of its surroundings by detecting markers using a provided service node. This initial mapping phase is crucial for subsequent patrolling activities. The robot then engages in a patrolling algorithm, leveraging autonomous navigation strategies and the information stored in the ontology during the mapping phase. Upon reaching a room, the robot performs a comprehensive scan, either by rotating its base or camera, enhancing the surveillance capabilities of the adaptive system. This assignment aims to demonstrate the effective integration and functionality of the developed architecture in a simulated environment for efficient indoor surveillance.

## Introduction

In the second assignment of the Experimental Robotics Laboratory course, the focus shifts toward the integration of an adaptive autonomous surveillance robot architecture within a simulated indoor environment designed for monitoring purposes. The assignment builds upon the foundation laid in the initial phase, where the robot's behavior and decision-making processes were conceptualized for efficient surveillance in a controlled environment. This second phase introduces a simulated environment comprising a custom message, service definitions, and a house model to be monitored. Additionally, a service node is provided, requiring the robot to detect markers and respond with information about the corresponding room. The overarching objective is to seamlessly incorporate the previously developed architecture into this simulation scenario, enhancing the robot's capabilities to navigate, map, and patrol the indoor space effectively. This integration aims to showcase the adaptability and functionality of the autonomous surveillance system in a more dynamic and realistic environment.

The assignment entails specific tasks, including the addition of a robot to the simulation environment, strategic placement of the robot at predefined coordinates, and the initiation of a comprehensive mapping process using markers and the provided service node. The robot's ability to autonomously navigate, detect markers, and build a semantic map becomes pivotal for the subsequent patrolling phase. Once the mapping is completed, the robot engages in a patrolling algorithm, utilizing autonomous navigation strategies and leveraging the information stored in the ontology during the mapping phase. Moreover, the robot is required to conduct thorough scans of each room it enters, employing rotations of its base or camera to enhance surveillance capabilities. This assignment aims to demonstrate the successful integration of the autonomous surveillance architecture within a simulated environment, emphasizing the practical application of the developed system in indoor surveillance scenarios.

Moreover, the [ROS package](https://github.com/CarmineD8/assignment2) serves as an experimental implementation utilizing a topological map ontology to control a mobile robot. The ontology describes an indoor environment with multiple rooms and a mobile robot. Comprehensive details about the source code can be found in the accompanying [documentation](https://ankurkohli007.github.io/Experimental_Robotics_Laboratory_Assignment_I/). The robot initiates its operation in room E and, through scanning designated markers, acquires information to construct a semantic map. This map includes the name and center position of each room, along with the connections between them. Below is the figure shows the scenario how the robot works in the 2D environment:

<p align="center">
  <img width="600" height="600" src="https://github.com/ankurkohli007/Experimental_Robotics_Laboratory_Assignment_I/blob/0a58201b93abaf209ae51218e42a83f1819e6c0d/environment.png">
</p>

<p align="center">
    <em>2D Environment</em>
</p> 


Once the semantic map is established, the robot follows a policy to navigate among rooms, prioritizing those that have not been visited for an extended period. Upon reaching a target room, the robot conducts a thorough environment scan, akin to the initial marker-based mapping process. When the robot's battery level is low, it autonomously navigates to the charger located in room E, where it waits for a specific duration before resuming its routine. This behavior loop ensures efficient room exploration, periodic scanning, and battery management in alignment with the specified ontology and control strategy.

The above scenario highlights that, we were provided with [package]((https://github.com/CarmineD8/assignment2)) and our requirements were:

- Integrate a robot into the environment, spawning it initially at coordinates x = -6.0, y = 11.0.
- Utilize a service node to detect all seven [AruCo markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) around the robot without moving its base, building the semantic map.
- Implement a patrolling algorithm based on autonomous navigation strategies and the information stored in the ontology.
- Conduct a comprehensive room scan, achieved by rotating either the robot's base or camera, upon reaching a designated room.

The figure below shows the gazebo environment for the simulation of the task: 

<p align="center">
  <img width="800" height="500" src="https://github.com/ankurkohli007/Experimental_Robotics_Laboratory_Assignment_II/blob/b753b4bcef2aa500ded1b3783e4657a5cf80945e/gazebo_environment.png">
</p>

<p align="center">
    <em>Gazebo Environment</em>
</p> 

## Robot Model

A comprehensive robot model with articulated features, sensors, and a differential drive mechanism. The robot's structural representation is segmented into distinct links, each serving a specific purpose in defining the physical components of the robot.

The primary link, named "link_chassis," represents the chassis of the robot, characterized by a collision model in the shape of a box. This link serves as the foundational structure for attaching other components, including the wheels and sensors. The wheels, namely "link_left_wheel" and "link_right_wheel," are modeled as cylinders and are connected to the chassis through continuous joints, allowing for rotational motion.

The robot features an articulated arm composed of two segments, "arm_base" and "arm_1," connected by a revolute joint named "arm_1_to_arm_base." This joint facilitates rotational movement, enabling the arm to pivot. Additionally, the model includes a further link, "arm_2," connected to "arm_1" through the "arm_2_to_arm_1" revolute joint. This design provides a multi-jointed arm capable of intricate movements.

The inclusion of sensors enhances the robot's perception capabilities. The "camera_link" link incorporates a camera sensor, and the "hokuyo_link" integrates a lidar sensor. Both sensors are equipped with collision and visual models, allowing for realistic simulation of their physical presence in the robot model.

Furthermore, the URDF specifies joint properties, such as joint types, limits, and damping, to govern the dynamic behavior of the robot. The differential drive mechanism is implemented through continuous joints, "joint_left_wheel" and "joint_right_wheel," connecting the wheels to the chassis. Additionally, joints "base_to_arm_base," "arm_1_to_arm_base," and "arm_2_to_arm_1" facilitate the movement of the articulated arm. The figure below highligts the robot model in Gazebo.

<p align="center">
  <img width="700" height="700" src="https://github.com/ankurkohli007/Experimental_Robotics_Laboratory_Assignment_II/blob/99ab74aa75483415ecf2fb5205ef3230ab1be8dc/gazebo_robot_model.png">
</p>

<p align="center">
    <em>Robot Model in Gazebo</em>
</p> 

The overall structure and properties defined in the Gazebo and Xacro files create a versatile and detailed robot model suitable for simulation environments like Gazebo. The inclusion of materials and colors enhances the visual representation of different components, aiding in the visualization and analysis of the robot's behavior and interactions within a simulated environment. Also, the figure below describes the robot model in RViz.

<p align="center">
  <img width="700" height="700" src="https://github.com/ankurkohli007/Experimental_Robotics_Laboratory_Assignment_II/blob/b4a5ee513ed0fcc11036a4ee60ac395a3e1f20b4/robot_model_rviz.png">
</p>

<p align="center">
    <em>Robot Model in RViz</em>
</p> 

After designing the urdf file, there some additional tools to check the syntax, below is the command:

```bashscript
check_urdf robot.urdf
```

* **Note:** In the above command, robot.urdf is your filename which you have saved under urdf folder.


To view the Graphical scheme of the file: 
```bashscript
urdf_to_graphiz robot.urdf
```
Here, also robot.urdf is your filename. This will going to generate the `.gv` file and to view the output of `.gv` file run the command `dot -Tpng -O filename.gv`. 

<p align="center">
  <img width="1000" height="800" src="https://github.com/ankurkohli007/Experimental_Robotics_Laboratory_Assignment_II/blob/1fa3b6834abf2e3a3becab0202cf797f86fa56e3/robot_link.png">
</p>

<p align="center">
    <em>Graphical Scheme of URDF File</em>
</p> 

The above figure depicts the output generated by the command `urdf_to_graphiz robot.urdf`, which explains the graphical view of the robot model.

## Description of the Architecture

The user can find the detailed description of the architecture from the Assignmnet 1 from [here](https://github.com/ankurkohli007/Experimental_Robotics_Laboratory_Assignment_I#software-architecture). For, this assignment the same set of nodes has been utilized, with the addition of the `marker_publish` and `move_base` nodes for the practical simulation of this assignment as shown below:

<p align="center">
  <img width="1000" height="1000" src="https://github.com/ankurkohli007/Experimental_Robotics_Laboratory_Assignment_II/blob/c00818403322af9c202793debccb9e5203e1f6f1/uml_diag.png">
</p>

<p align="center">
    <em>Software Archutecture Component Diagram</em>
</p> 

In the above diagram, there were addition nodes such as `marker_publisher` & `move_base` nodes and below is the brief description about the same:

* **marker_publisher:** Image processing is accomplished using [OpenCV](https://opencv.org/) through a method outlined in [aruco_ros](https://github.com/CarmineD8/aruco_ros). The conversion between ROS Image messages and OpenCV images is facilitated by the [cv_bridge](http://wiki.ros.org/cv_bridge) package.
* **move_base:** For navigation purposes, the node responsible is utilizing the [move_base](http://wiki.ros.org/move_base) node. This particular node is instrumental in determining the path from the current position of the robot to the target room position. Additionally, it manages the movement of the robot's base link along the identified path.

#### Mapping Algorithm

For mapping I used [slam_gmapping](https://github.com/CarmineD8/slam_gmapping) package to achieve the goal. The widely used ROS package, slam_gmapping, is employed for map creation. Leveraging data from a robot laser scanner and the position of the robot's base frame, slam_gmapping accurately determines the robot's location within the map. Notably, slam_gmapping stands out from other mapping packages by generating maps in real-time, eliminating the need for a pre-existing map file.

## Installation & Running

### Installation

To utilize this software effectively, the user should adhere to the following steps for installing the necessary packages and repositories.

* As the author opted to utilize the files [``planner.py``](https://github.com/buoncubi/arch_skeleton/blob/main/scripts/planner.py) and [``controller.py``](https://github.com/buoncubi/arch_skeleton/blob/main/scripts/controller.py) from the [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository, along with the [topological_map.owl](https://github.com/buoncubi/topological_map/blob/main/topological_map.owl) file from the [topological_map](https://github.com/buoncubi/topological_map) repository— both authored by [Prof. Luca Buoncompagni](https://rubrica.unige.it/personale/VkRGWFJq) and the user is required to clone the aforementioned repositories along with the current one into the ROS workspace.
* This package, being dependent on [aRMOR](https://github.com/EmaroLab/armor), must be installed according to the instructions outlined in the provided link as a prerequisite for running this package.
* Additionally, it has a dependency on [SMACH](https://wiki.ros.org/smach), which can be installed by using the following commands:

```bashscript
sudo apt-get install ros-<distro>-executive-smach*
```
```bashscript
sudo apt-get install ros-<distro>-smach-viewer
```

Here, ``ros-<distro>-`` depends to denote a package or component associated with a specific ROS distribution. For instance, **"ros-noetic"**, **"ros-melodic"**, and so on.

* Also, clone the [armor_py_api](https://github.com/EmaroLab/armor_py_api) repository in your ROS workspace.
* Once the dependencies are met, the package can be installed as it follows:

```bashscript
mkdir -p ros_ws/src
```
```bashscript
cd ros_ws/src
```
```bashscript
git clone https://github.com/ankurkohli007/Experimental_Robotics_Laboratory_Assignment_II.git
```
```bashscript
cd ..
```
```bashscript
source /opt/ros/<distro>/setup.bash
```

Here, again the ``<distro>`` denotes a package or component associated with a specific ROS distribution.

* Execute ``chmod +x <file_name>`` for each file inside the folder ``scripts`` of the assignment package which was cloned in the last step.
* Execute ``catkin_make`` from the root of your ROS workspace.
* For the installation of aRMOS package, you can also the provide [link](https://github.com/ankurkohli007/Experimental_Robotics_Laboratory_Assignment_I?tab=readme-ov-file#installation).

**Note:** For debugging during installation refer Assignment 1 installation steps [from here](https://github.com/ankurkohli007/Experimental_Robotics_Laboratory_Assignment_I?tab=readme-ov-file#note).

There are some useful package can be install for the navigation task by the command below:

```bashscript
sudo apt-get install ros-<ros_distro>-openslam-gmapping
```
```bashscript
sudo apt-get install ros-<ros_distro>-navigation
```

For ``ros-<distro>-``, again depends on a component associated with a specific ROS distribution.


### aruco_ros

Also, clone [aruco_ros](https://github.com/CarmineD8/aruco_ros) package from the repository link Provided by [Prof. Carmine Tommaso Recchiuto](https://rubrica.unige.it/personale/UkNDWV1r).

```bashscript
git clone https://github.com/CarmineD8/aruco_ros.git
```
For detection of marker, first we need to create the marker textute. For this, let’s copy the folder [models](https://github.com/CarmineD8/aruco_ros/tree/main/aruco_ros/models) of the aruco_ros package in `/root/.gazebo/models` (all new models should be put here, to let the camera work properly). To copy all the models use the command `cp -r source_path destination_path` (if recursively) `cp -r source_path* destination_path` (if not recursively). Here the source path is where you clone the above `aruco_ros` package and destination path is `/root/.gazebo/models`. To copy the multiple models at once use the command `cp -r source{file1,file2,}* destination` (for multiple files).

* **Note:** In the `marker_publish.cpp`, under this aruco_ros package [path](https://github.com/CarmineD8/aruco_ros/blob/main/aruco_ros/src/marker_publish.cpp), some minor changes have been made as show below:

```cpp
public:
  ArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  {
    image_sub_ = it_.subscribe("/image", 1, &ArucoMarkerPublisher::image_callback, this);
    image_pub_ = it_.advertise("result", 1);
    debug_pub_ = it_.advertise("debug", 1);
    scan_pub = nh_.advertise<std_msgs::Int64>("/scan_marker", 1000); # publisher defined
    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();
  }
```
* **Note:** For more details about the assignment follow the repository [link](https://github.com/CarmineD8/assignment2) provided by [Prof. Carmine Tommaso Recchiuto](https://rubrica.unige.it/personale/UkNDWV1r)
  
### Running 

To initialize the software architecture in conjunction with the finite state machine representation, first install ``xterm`` if it is not installed by the command ``sudo apt-get install -y xterm`` and then execute the following commands:

```bashscript
source devel/setup.bash
```
```bashscript
roslaunch assignment2 main.launch
```

### RQT Graph

To view the `rqt_graph` execute the command given below:

```bashscript
rosrun rqt_graph rqt_graph
```

<p align="center">
  <img width="1000" height="800" src="https://github.com/ankurkohli007/Experimental_Robotics_Laboratory_Assignment_II/blob/16b9333feadb5807eb8990bed5772e79feca807c/rosgraph.png">
</p>

<p align="center">
    <em>RQT Graph</em>
</p> 

The figure above shows the `rqt_graph`. 

## Result Discussion

The integration of the adaptive autonomous surveillance robot architecture into the robotic simulation environment yielded promising results in terms of indoor monitoring and surveillance. Through the seamless integration process outlined in the assignment, the robot successfully navigated the simulated indoor environment, autonomously building a semantic map and conducting comprehensive room scans.

The initial mapping phase, crucial for subsequent patrolling activities, was effectively executed by the robot. Leveraging the provided service node and markers, the robot autonomously detected and mapped its surroundings. This process demonstrated the robot's capability to interpret environmental cues and construct a semantic map, laying the foundation for efficient navigation and surveillance.

Upon completion of the mapping phase, the robot seamlessly transitioned into the patrolling algorithm. Utilizing autonomous navigation strategies and leveraging the information stored in the ontology from the mapping phase, the robot efficiently patrolled the indoor space. By prioritizing unvisited rooms and conducting thorough scans upon entry, the robot demonstrated its ability to adapt to dynamic environments and enhance surveillance capabilities.

The inclusion of additional nodes such as marker_publisher and move_base further enriched the simulation environment, enabling enhanced functionality and practical application of the developed architecture. The marker_publisher node facilitated image processing through OpenCV, enabling marker detection, while the move_base node enabled navigation and path planning, contributing to the overall autonomy of the surveillance system.

Moreover, the utilization of the slam_gmapping package for map creation proved to be effective, allowing real-time generation of maps using data from the robot's laser scanner and base frame position. This approach eliminated the need for pre-existing map files and enhanced the adaptability of the surveillance system to dynamic environments.

Overall, the successful integration and functionality of the developed architecture in the simulated environment showcase its potential for efficient indoor surveillance applications. The results highlight the adaptability, autonomy, and effectiveness of the autonomous surveillance robot architecture in monitoring and patrolling indoor environments, laying the groundwork for future advancements in robotic surveillance systems.


