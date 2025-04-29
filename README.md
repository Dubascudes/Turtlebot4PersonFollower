# Person Follower Robot, EEL 5934 Autonomous Robotics, University of Florida Spring 2025

This project implements a person-following robot using ROS 2. We us YOLOv8n for person detection and target heading estimation and lidar data to estimate distance. These measurements are smoothed, filtered, and used in a P-controller designed to stay 1 meter behind the target.

We have defined some commands for the robot that determine its current action.
Undock: The robot starts on a power dock for charging. This command has the robot
back off of the dock and turn 180 degrees. Dock: There is a dock pose topic than we subscribe to, which allows us to get the position of the dock relative to the robot. We can then move the robot to that position, having it re-dock.
Idle: This kills the robot's current action. 
Follow: This causes the robot to follow a person.

The "Following" process has two main components: the "see" node and "think" node. 

See Node:
The see node drives person detection for the robot. We use YOLOv8 to detect people. We gather all detected person bounding boxes
and pass them to the Simple, Online, and Real-time Tracking (SORT) algorithm ([link](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt)). 
SORT tracks all of the detected person objects and assigns unique IDs to each object. Then, a homography is performed on each 
bounding box, transforming them from the camera point-of-view to the lidar point-of-view. This allows us to get the 
heading of each of the tracked bounding boxes. We search a +/-10 degree angle for each bounding box heading. Within that 
cone, we take the average of the closest 10% of points and consider that to be the location of the person indicated by the bounding box.
If no person has previously been detected, the closest person will be the new person to follow. Otherwise, the robot
follows the person that it was previously following, indicated by a matching SORT ID. The distance and heading to the designated person
are then filtered and smoothed using a 1D Kalman Filter. The heading and distance are then fed to the think node. 

Think Node: 
We directly control the linear and angular velocity and publish them to /cmd_vel in a TwistStamped message. The robot moves directly toward the person
until it reaches the minimum following distance, at which point it stops and waits for the person to move farther away. Once the person has moved farther away,
the robot begins moving towards the person again. While moving, we adjust the heading of the robot so that the person being followed is directly 
centered on the camera. This is achieved by comparing the camera center to the bounding box center and adjusting the heading accordingly. When the 
person moves out of camera view, the robot spins in the direction that the person moved out of camera view until the person is found again.

## Prerequisites
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Python 3.10+
- Virtual environment (recommended)

## Setup Instructions

1. Clone this repository and cd into it:
```bash
git clone <repository-url>
cd ~/path/to/repo
```

2. Create and activate a virtual environment:
```bash
python3 -m venv .venv
source .venv/bin/activate
```

3. Install Python dependencies:
```bash
pip install -r requirements.txt
```

4. Install ROS 2 dependencies and source ROS:
```bash
sudo apt-get install ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-sensor-msgs ros-jazzy-std-msgs ros-jazzy-tf-transformations ros-jazzy-irobot-create-msgs
source /opt/ros/jazzy/install/setup.bash
```

5. Build the ROS 2 workspace:
```bash
colcon build
source install/setup.bash
```

## Project Structure

- `final_project/`: Main ROS 2 package
  - `perception_subsystem.py`: Handles person detection + heading and distance estimation
  - `planning_subsystem.py`: Implements state machine and P-controller
  - `person_follower_cli.py`: Command-line interface for human control

## Usage
Start 4 terminal sessions (make sure each of them are sourced in your venv except the one used for rqt_image_view):

1. In terminal 1, run the CLI:
```bash
ros2 run final_project cli
```

2. Start the planning subsystem:
```bash
ros2 run final_project think
```

3. Start the perception subsystem:
```bash
ros2 run final_project see
```

## YOLO Model Files

The project uses the YOLOv8n model (`yolov8n.pt`) for person detection. [Download it](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt) and put it in this repository.


## Maintainers

- William English (will.english@ufl.edu)
- Dominic Simon (dominic.simon@ufl.edu)
- Chase Walker (chase.walker@ufl.edu)
- Mustafa Mohammad Shaky (mustafa.shaky@ufl.edu)
