# Landing_on_Moving_Rover
This project intends to land a quad-copter on a moving rover using Visual Servoing.

# Demonstration


# Installation

Register on https://cps-vo.org/group/CPSchallenge to get access to pre-configured docker container with 3D graphics, ROS, Gazebo, and PX4 flight stack.
Or you can clone and build using https://github.com/Open-UAV/cps_challenge_2020.git


Apriltag & Apriltag_ros (v2 or v3) Install any dependencies for apriltag from https://github.com/AprilRobotics/apriltag_ros
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/AprilRobotics/apriltag_ros
git clone --recursive https://github.com/AprilRobotics/apriltag
```

Follow below instruction to clone and build the code![Studio_Project (1)](https://user-images.githubusercontent.com/97504177/236708959-100a0d4e-8fb0-4113-8710-c076d8596115.gif)

```
git clone https://github.com/skp-1997/landing_on_moving_rover.git
catkin build
```

# Running the code

Run the phase-1 world of gazebo using the command
```
roslaunch cps_challenge_2020 phase-2.launch
```
Run the scripts in ./scripts folder
```
python autonomous_landing.py
```

```
python move_rover.py
```

