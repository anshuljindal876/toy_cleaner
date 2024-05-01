The packages for the vision demo can be found in vision/src on this branch.
Those packages are the only ones on this branch that are needed for the vision demo.
To run the vision demo as seen in the presentation slides, use the following:

First,
ros2 launch capstone_vision_env simulation.launch.py 

Then,
ros2 launch capstone_vision camera_reading_demo.launch.py 

This was made with ROS 2 Humble on Ubuntu 22.04.4 LTS.
OpenCV will need to be installed in order to run the demo.
