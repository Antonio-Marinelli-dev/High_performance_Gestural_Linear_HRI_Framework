Gesture-Based Human–Robot Interaction for the OpenManipulator-X

## Demo (watch the full demo video)
▶️ [https://www.youtube.com/watch?v=aQiPd1rBwCM]

A ROS Noetic / Gazebo framework that lets an operator command a 4-DOF OpenManipulator-X pick-and-place task barehanded: the user points at an object with the index finger and confirms by extending the thumb. Hand tracking comes from a standard monocular webcam, with no gloves, controllers or depth sensors.

Developed for the Robotics course, MSc Mechatronics Engineering, Politecnico di Torino. The whole system runs in simulation inside a Docker container.

<p align="center"> <img src="docs/architecture.png" alt="System architecture and information flow" width="100%"> </p>
Why this project

The method is based on the Linear HRI paradigm from Cornak, Tolgyessy and Hubinsky, "Innovative Collaborative Method for Interaction between a Human Operator and Robotic Manipulator Using Pointing Gestures". That paper points out two weaknesses: the Leap Motion sensor is sensitive to occlusions and lighting, and raising the thumb to confirm a command shifts the operator's aim by roughly 15–50 mm. This project shows that both can be compensated for in software.

Key features
Markerless hand tracking with MediaPipe on a 2D RGB camera. The missing depth is estimated geometrically from the proportions of the hand landmarks, plus a virtual sensor at a fixed altitude that stabilises the projection onto the workbench.
Tremor suppression: a moving-average (low-pass) filter on the index-finger vector.
Assisted Aiming: a dynamic tolerance zone (5.4 cm radius) around the pointer intersection. If the command is issued with an object inside it, the system snaps to that object's centre. With several candidates, the one closest to the pointer wins.
Analytical inverse kinematics for the 4-DOF arm, written from scratch, with decoupled trajectories: horizontal approach at a safe height, then a vertical descent.
Reachability checks and feedback: the HUD reports NO TARGET DETECTED and TARGET UNREACHABLE instead of executing invalid commands.
Gestural emergency stop: a single open palm freezes the arm at its current position while keeping motor torque, so gravity cannot make it drift. The stop is latched in software and cleared only by a manual reset typed in the terminal.
Virtual grasp through the gazebo_ros_link_attacher plugin, avoiding the numerical instabilities of contact physics.
Control loop at 50 Hz and camera stream at 15 FPS.
Software architecture
Component	Role
apri_ambiente.launch, ambiente_simulazione.world	Environment initialisation: Gazebo world and OpenManipulator-X model
sensor_3D.py	Sensory module: reads the webcam, extracts the hand skeleton, filters the index-finger vector, detects thumb extension and the open-palm stop gesture
pointer_gazebo.py	Geometric engine: intersects the pointing vector with the workbench plane, places the marker in Gazebo, checks that the point is inside the workspace
camera_sensor.py	Spatial supervisor: analyses the scene from above, finds object contours, applies the tolerance radius and validates the target
controller_2.py	Control and execution: reachability check, inverse kinematics and trajectory execution
Repository structure
UR5_project_10/               # ROS package (name kept for historical reasons)
├── CMakeLists.txt
├── package.xml
├── requirements.txt
├── docs/
│   └── architecture.png
└── scripts/
    ├── ambiente_simulazione.world
    ├── apri_ambiente.launch
    ├── sensor_3D.py
    ├── pointer_gazebo.py
    ├── camera_sensor.py
    └── controller_2.py
Requirements
Docker, with the image osrf/ros:noetic-desktop-full (ROS Noetic, Gazebo)
Python 3 packages (see requirements.txt): mediapipe 0.10.5, opencv-python 4.13, numpy, and the system package python3-tk
A webcam
ROS packages used by the project, cloned into the same src folder:
Package	Repository	Branch	Tested commit
open_manipulator	ROBOTIS-GIT/open_manipulator	noetic	8cb9c5e
open_manipulator_dependencies	ROBOTIS-GIT/open_manipulator_dependencies	noetic	ed3a162
open_manipulator_simulations	ROBOTIS-GIT/open_manipulator_simulations	noetic-devel	088db0c
robotis_manipulator	ROBOTIS-GIT/robotis_manipulator	noetic	7023c35
gazebo_ros_link_attacher	pal-robotics/gazebo_ros_link_attacher	melodic-devel	1d170c4
Installation
bash
# inside the container
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

# this project
git clone <URL-of-this-repository> UR5_project_10

# external ROS packages
git clone -b noetic https://github.com/ROBOTIS-GIT/open_manipulator.git
git clone -b noetic https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
git clone -b noetic https://github.com/ROBOTIS-GIT/robotis_manipulator.git
git clone -b melodic-devel https://github.com/pal-robotics/gazebo_ros_link_attacher.git

# system and Python dependencies
apt update && apt install -y python3-tk
pip3 install -r UR5_project_10/requirements.txt

# build
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
Usage

Open one terminal per node in the container. In each of them load the workspace first:

bash
source ~/catkin_ws/devel/setup.bash

0. Webcam (Windows + WSL2 only). The webcam must be shared with WSL using usbipd-win. In PowerShell:

powershell
usbipd list                          # find the BUSID of the webcam
usbipd bind --busid <BUSID>          # first time only, as administrator
usbipd attach --wsl --busid <BUSID>

1. Start the ROS master:

bash
roscore

2. Launch the simulation environment (Gazebo, world and robot):

bash
roslaunch UR5_project_10 apri_ambiente.launch

3. Start the nodes, one per terminal:

bash
rosrun UR5_project_10 pointer_gazebo.py
rosrun UR5_project_10 sensor_3D.py
rosrun UR5_project_10 camera_sensor.py
rosrun UR5_project_10 controller_2.py

Point at an object with your index finger and extend your thumb to send the command. Show an open palm to trigger the emergency stop.

Resetting after an emergency stop

Once the stop has been triggered, the robot ignores every positioning command and the HUD keeps showing the alert until the operator resets it with a manual command in the terminal (/riprendi_robot).

<!-- TODO: write the exact reset command and which terminal it goes in -->
Troubleshooting
The webcam is not detected: check that it is attached with usbipd, then try index 0 or 1 in the cv2.VideoCapture call. A quick test: python3 -c "import cv2; print(cv2.VideoCapture(0).isOpened(), cv2.VideoCapture(1).isOpened())"
ModuleNotFoundError: tkinter: install it with apt install -y python3-tk.
rosrun cannot find the package: the workspace is not loaded in that terminal, run source ~/catkin_ws/devel/setup.bash.
Simplifying assumptions

The framework was validated in simulation, with these simplifications:

the Leap Motion is replaced by a 2D camera, with depth estimated in software;
grasp physics is replaced by a virtual coupling between links;
the workcell has constant illumination and no disturbances, so the contour segmentation uses fixed thresholds.
Authors
Marinelli Antonio, Barile Mauro, Leopizzi Francesco, Tarantini Antonio

Reference

M. Cornak, M. Tolgyessy, P. Hubinsky, Innovative Collaborative Method for Interaction between a Human Operator and Robotic Manipulator Using Pointing Gestures.
