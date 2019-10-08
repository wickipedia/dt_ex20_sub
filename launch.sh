#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roscore &
sleep 5
roslaunch process_image multiple_nodes.launch
#rosrun read_bag read.py
