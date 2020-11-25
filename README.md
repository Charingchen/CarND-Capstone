This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the original repo https://github.com/udacity/CarND-Capstone 

# Getting Started
I used the VM provide from Udacity to run ROS. You can download [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/Udacity_VM_Base_V1.0.0.zip)

Download [VirtualBox](https://www.virtualbox.org/wiki/Downloads) to load the image

1. Import your VM image to VirtualBox
2. Download and install VirtualBox.
3. Download the image from supplied link.
4. Unzip the image.
5. Open VirtualBox Application.
6. Click File > Import Appliance..
7. Click the folder icon on the right and navigate to your unzipped image (the .ovf file).
8. Follow the prompts to import the image.
9. From the VirtualBox Manager, select your VM and press start.

 The password for the VM is `udacity-nd`

## Running launch files
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

# Final Result Video
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/S9gTOHb5b2A/0.jpg)](https://www.youtube.com/watch?v=S9gTOHb5b2A)
### Note
This project does not include a neural network traffic light detection. The traffic light detection is using simulator ground true detected. 

## Code Structure
![stru](./images/final-project-ros-graph-v2.png)
Below is a brief overview of the repo structure, along with descriptions of the ROS nodes. The code that you will need to modify for the project will be contained entirely within the (path_to_project_repo)/ros/src/ directory. Within this directory, you will find the following ROS packages:

### (path_to_project_repo)/ros/src/tl_detector/
This package contains the traffic light detection node: tl_detector.py. This node takes in data from the `/image_color`, `/current_pose`, and `/base_waypoints` topics and publishes the locations to stop for red traffic lights to the `/traffic_waypoint` topic.

The `/current_pose topic` provides the vehicle's current position, and `/base_waypoints` provides a complete list of waypoints the car will be following.

You will build both a traffic light detection node and a traffic light classification node. Traffic light detection should take place within tl_detector.py, whereas traffic light classification should take place within `../tl_detector/light_classification_model/tl_classfier.py.`


### (path_to_project_repo)/ros/src/waypoint_updater/
This package contains the waypoint updater node: `waypoint_updater.py`. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the `/base_waypoints`,` /current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint topics`, and publish a list of waypoints ahead of the car with target velocities to the `/final_waypoints` topic.


### (path_to_project_repo)/ros/src/twist_controller/
Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node dbw_node.py and the file twist_controller.py, along with a pid and lowpass filter that you can use in your implementation. The dbw_node subscribes to the `/current_velocity` topic along with the `/twist_cmd topic` to receive target linear and angular velocities. Additionally, this node will subscribe to `/vehicle/dbw_enabled`, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd topics`.


In addition to these packages you will find the following, which are not necessary to change for the project. The styx and styx_msgs packages are used to provide a link between the simulator and ROS, and to provide custom ROS message types:

* (path_to_project_repo)/ros/src/styx/
  A package that contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.
* (path_to_project_repo)/ros/src/styx_msgs/
  A package which includes definitions of the custom ROS message types used in the project.
* (path_to_project_repo)/ros/src/waypoint_loader/
  A package which loads the static waypoint data and publishes to /base_waypoints.
* (path_to_project_repo)/ros/src/waypoint_follower/
  A package containing code from Autoware which subscribes to `/final_waypoints` and publishes target vehicle linear and angular velocities in the form of twist commands to the `/twist_cmd topic`.

