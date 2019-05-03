# Leg detection package through LaserScan 2D points.

 - This package listens to the `/scan` laser scan message.
 - Tested on *ROS Kinetic* and *Python2.7*
 - Legs markers publishes on `/LegMarker` topic.

## Usage:

 - Clone this repo.
 - `catkin_make`
 - `rosrun agn_leg_detection agn_leg_detection.py `

### TODO:
 - Refactoring (I wrote this node, when I was a newbie in Python.)
 - Make it as parametric-able by ros-param.
