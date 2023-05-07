# Landing_on_Moving_Rover
This project intends to land a quad-copter on a moving rover using Visual Servoing.

# Demonstration


# Installation

Register on https://cps-vo.org/group/CPSchallenge to get access to pre-configured docker container with 3D graphics, ROS, Gazebo, and PX4 flight stack.
Or you can clone and build using https://github.com/Open-UAV/cps_challenge_2020.git


Follow below instruction to clone the repo in the cps_challenge_2020 folder
```
cd cps_challenge_2020
```
```
git clone https://github.com/skp-1997/landing_on_moving_rover.git
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

