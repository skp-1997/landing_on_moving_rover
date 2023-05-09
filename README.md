# Landing_on_Moving_Rover
This project intends to land a quad-copter on a moving rover using Visual Servoing.


# Demonstration
![Studio_Project (2)](https://user-images.githubusercontent.com/97504177/236709060-351487da-2213-4cca-b190-518aad6e828a.gif)


# Installation

Register on https://cps-vo.org/group/CPSchallenge to get access to pre-configured docker container with 3D graphics, ROS, Gazebo, and PX4 flight stack.
Or you can clone and build using https://github.com/Open-UAV/cps_challenge_2020.git


Apriltag & Apriltag_ros (v2 or v3) Install any dependencies for apriltag from https://github.com/AprilRobotics/apriltag_ros
[ Make sure you have made all necessary changes in the apriltag_ros to link it with the gazebo! ]
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/AprilRobotics/apriltag_ros
git clone --recursive https://github.com/AprilRobotics/apriltag
```

Follow below instruction to clone and build the code





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

# Setup of apriltag_ros folder

1. Provide correct camera path of the quadcopter into the continuous_detection.launch file in ./apriltag_ros/launch/
2. Edit the settings.yaml and tag.yaml in ./apriltag_ros/config as per your apriltag
3. Add the continuous_detection.launch file in /cps_challenge/launch/phase-2.launch file, which will run the april_tag ros when you run the phase-2.launch


# Pre-requisites

1. Python 2.7
2. Ubuntu 18.04
3. ROS1 MELODIC
4. Gazebo 9.6


# Additional Content (Landing the drone on a rover moving in a straight line)

https://youtu.be/1oGakOXok2U
