# simulation_ws
ros simulation scripts for iarc mission7 using iris with optical flow, lpe and px4 flight stack.

Moved from simulation branch under odroid-ws, for previous commits refer https://github.com/TEAMIFOR/odroid_ws/commits/simulation

## Setup Intructions 
PART A: Setup px4 flight stack and sim env
```
git clone https://github.com/px4/Firmware.git
cd Firmware
make posix_sitl_lpe gazebo_iris_opt_flow 
```
Editing necessary files
1. Replace Firmware/launch/mavros_posix_sitl.launch (spawing iris_opt_flow)
2. Replace Firmware/Tools/sitl_gazebo/models/iris_opt_flow/iris_opt_flow.sdf (attach camera and define rostpoics usbcam substitute)
3. Replace Firmware/Tools/sitl_gazebo/worlds/empty.world
4. Replace Firmware/posix-confings/SITL/init/lpe/iris_opt_fow (custom tweaked params, reduce some errors)
5. Replace ~/.gazebo/models/hokuyo/model.sdf (GPU laser with ros publisher /scan)
```
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_lpe
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch
```
PART B: Setup your workspace
```
git clone https://github.com/TEAMIFOR/odroid_ws.git
cd odroid_ws
chmod 755 launcher.sh
. ./launcher.sh
```

## Changelog
```
v1 ellipse detection with single onboard cam
v2 ellipse detection with two onboard cam
v3 ellipse detection and offboard surveillance script fireup from single launch
v4 object detection and following scripts added (feedback loop based on quadrants)
v5 object detection and following scripts added (feedback loop based on gradient map)
```

## Notes
1. make posix_sitl_lpe gazebo_iris_opt_flow is CPU intensive so apply brain
2. The gazebo camera plugin which is edited into iris_opt_flow.sdf publishes a ros node, check topic names here ( replaces usb_cam node during simulation runs)
3. we launch px4 flight stack and sim env with mavros as in mavros_posix_sitl.launch which has been edited according to our requirements 
4. for ellipse detection specialised values have been used for thresh, lower major, lower minor axis and etc which suit best in sim env

