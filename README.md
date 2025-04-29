# Person Follower Robot

This project implements a person-following robot using ROS 2. We us YOLOv8n for person detection and target heading estimation and lidar data to estimate distance. These measurements are smoothed, filtered, and used in a P-controller designed to stay 1 meter behind the target.

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

4. Install ROS 2 dependencies:
```bash
sudo apt-get install ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-sensor-msgs ros-jazzy-std-msgs
```

5. Build the ROS 2 workspace:
```bash
cd final_project
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

## Model Files

The project uses the YOLOv8n model (`yolov8n.pt`) for person detection. [Download it](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt) and put it in this repository.
## License

TODO: Add license information

## Maintainers

- William English (will.english@ufl.edu)
- Dominic Simon (dominic.simon@ufl.edu)
- Chase Walker (chase.walker@ufl.edu)
- Mustafa Mohammad Shaky (mustafa.shaky@ufl.edu)
