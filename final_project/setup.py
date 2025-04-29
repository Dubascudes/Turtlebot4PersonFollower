from setuptools import find_packages, setup
import os
from glob import glob
import sys

package_name = 'final_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge', 'ultralytics', 'filterpy'],
    zip_safe=True,
    maintainer='William English',
    maintainer_email='will.english@ufl.edu',
    description='Final Project - Person Follower Robot for EEL 5934: Autonomous Robotics, Spring 2025',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Perception and localization subsytem 
            # subscribes to the oakd camera and lidar to estimate
            # the distance and heading of the target relative to the robot
            'see = final_project.perception_subsystem:main',

            # Planning subsystem records information given by the sensors and uses it
            # to determine the state we are in and our Robustness Score
            # (Distance from target? Perception error? idk this isn't that important)
            'think = final_project.planning_subsystem:main',

            # Control subsystem that interfaces between the planner and
            #  nav2/create3
            'act = final_project.control_subsystem:main',

            # Human controller input
            'cli = final_project.person_follower_cli:main',
        ],
    },
)