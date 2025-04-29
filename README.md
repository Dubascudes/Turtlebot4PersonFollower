# Person Follower Robot

This project implements a person-following robot using ROS 2, YOLOv8 for person detection, and various sensors for localization and control.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10+
- Virtual environment (recommended)

## Setup Instructions

1. Clone this repository:
```bash
git clone <repository-url>
cd robotics_project
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
sudo apt-get install ros-humble-cv-bridge ros-humble-image-transport ros-humble-sensor-msgs ros-humble-std-msgs
```

5. Build the ROS 2 workspace:
```bash
cd final_project
colcon build
source install/setup.bash
```

## Project Structure

- `final_project/`: Main ROS 2 package
  - `perception_subsystem.py`: Handles person detection and localization
  - `planning_subsystem.py`: Implements state machine and decision making
  - `control_subsystem.py`: Controls robot movement
  - `person_follower_cli.py`: Command-line interface for human control

## Usage

1. Start the perception subsystem:
```bash
ros2 run final_project see
```

2. Start the planning subsystem:
```bash
ros2 run final_project think
```

3. Start the control subsystem:
```bash
ros2 run final_project act
```

4. Use the CLI for manual control:
```bash
ros2 run final_project cli
```

## Model Files

The project uses the YOLOv8n model (`yolov8n.pt`) for person detection. This model is included in the repository.

## License

TODO: Add license information

## Maintainers

- William English (will.english@ufl.edu)
