#!/bin/sh

git submodule update --init src/image_common
chmod +x src/missionpkg/src/el_det_fi.py
catkin_make
source devel/setup.bash
roslaunch missionpkg missionpkg.launch
