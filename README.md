# Person Follower Robot, EEL 5934 Autonomous Robotics, University of Florida Spring 2025

This project implements a person-following robot using ROS 2. We us YOLOv8n for person detection and target heading estimation and lidar data to estimate distance. These measurements are smoothed, filtered, and used in a P-controller designed to stay 1 meter behind the target.

We have defined some commands for the robot that determine its current action.
Undock: The robot starts on a power dock for charging. This command has the robot
back off of the dock and turn 180 degrees. Dock: There is a dock pose topic than we subscribe to, which allows us to get the position of the dock relative to the robot. We can then move the robot to that position, having it re-dock.

Idle: This kills the robot's current action. 
Follow: This causes the robot to follow a person.

The "Following" process has two main components: Person detection and P-control. 

Person Detection:
We use YOLOv8 to detect people. We use the lidar to find the closest object and draw bounding
boxes around the closest detected object. We then draw a circle at the center of that bounding box.
The robot tries to center its camera on the bounding box center. We also use an algorithm called
SORT to keep track of all the different objects that have been detected and only follow the object
that was initially detected.

P-Control: We directly control the linear and angular velocity through the Twist node. We get the
heading from the robot to the person by comparing the camera center to the bounding box center and adjust accordingly. 
We get the distance from the robot to the person using lidar information. When the robot reaches the minimum following
distance, it stops and waits for the person to move farther away. When the person moves out of camera view, the robot
spins in the direction that the person moved out of camera view until the person is found again.

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
