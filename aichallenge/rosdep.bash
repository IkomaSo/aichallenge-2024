#!/bin/bash
rosdep update
rosdep install -y -r -i --from-paths /aichallenge/workspace/src/ --ignore-src --rosdistro $ROS_DISTRO