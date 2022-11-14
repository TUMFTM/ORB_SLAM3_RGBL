# ORB-SLAM3 ROS2 Utils

### Contents
This directory contains the scripts required to use ORBSLAM3 in a ros environment.

### Prerequisites
ROS2 Foxy
ORBSLAM3 build including all it's dependencies

Tested on Ubuntu 20.04.

# 1. ROS 2 Interface for ORBSLAM3
ROS2 node wrapping the ORB_SLAM3 library

## Build:
if you built ORB_SLAM3 following the instructions provided in its repository, you will have to tell CMake where to find it by exporting an environment variable that points to the cloned repository (as the library and include files will be in there).
```
$ export ORB_SLAM3_ROOT_DIR=/path/to/ORB_SLAM3
```
Also copy the files in ORB_SLAM3/include/CameraModels into the ORB_SLAM3/include 

Then you can build this package

```
cd /mod_map_loc/ORB_SLAM3/ros2/ros2_orb_slam3
colcon build
```
## Usage:
First source the workspace
```
$ source ws/install/setup.sh
```
Then add to the LD_LIBRARY_PATH the location of ORB_SLAM2 library and its dependencies (the following paths may be different on your machine)
```
$ export LD_LIBRARY_PATH=~/Pangolin/build/src/:~/ORB_SLAM3/Thirdparty/DBoW2/lib:~/ORB_SLAM3/Thirdparty/g2o/lib:~/ORB_SLAM3/lib:$LD_LIBRARY_PATH
```
Run the monocular SLAM node
```
$ ros2 run ros2_orb_slam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
you can find the vocabulary file in the ORB_SLAM3 repository (e.g. ORB_SLAM3/Vocabulary/ORBvoc.txt), while the config file can be found within this repo (e.g. ros2-ORB_SLAM3/src/monocular/TUM1.yaml for monocular SLAM).

This node subscribes to the ROS2 topic camera and waits for Image messages. For example you can stream frames from your laptop webcam using:
```
$ ros2 run image_tools cam2image --ros-args -r image:=camera
```
You can run the rgbd node by using (not working yet)
```
$ ros2 run ros2_orb_slam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
You can run the stereo node by using (not working yet)
```
$ ros2 run ros2_orb_slam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
```

# 2. KITTI Odometry Publishers
## Contains:
image_publisher	-	Publishes the images of a sequence
pcd_publisher		-	Publishes the pointclouds of a sequence
pcd_overlay_publisher	-	Recieves img and pcd from the publishers above and performs an overlay

## Note:
When running both, the image and pcd publisher, the image publisher can be seen as the "master". It sends the currently published image id to the pcd publisher which resets its counter in case of a deviation, to ensure both sources are publishing the same frame.

## Known Issues:
In the calib files from KITTI, some matrices may be missing. In case you get an error from the overlay nodes read_calib function check the respecive calibration file and add the missing matrices:

```
P0: 7.215377000000e+02 0.000000000000e+00 6.095593000000e+02 0.000000000000e+00 0.000000000000e+00 7.215377000000e+02 1.728540000000e+02 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0.000000000000e+00
P1: 7.215377000000e+02 0.000000000000e+00 6.095593000000e+02 -3.875744000000e+02 0.000000000000e+00 7.215377000000e+02 1.728540000000e+02 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0.000000000000e+00
P2: 7.215377000000e+02 0.000000000000e+00 6.095593000000e+02 4.485728000000e+01 0.000000000000e+00 7.215377000000e+02 1.728540000000e+02 2.163791000000e-01 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 2.745884000000e-03
P3: 7.215377000000e+02 0.000000000000e+00 6.095593000000e+02 -3.395242000000e+02 0.000000000000e+00 7.215377000000e+02 1.728540000000e+02 2.199936000000e+00 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 2.729905000000e-03
R0_rect: 9.999239000000e-01 9.837760000000e-03 -7.445048000000e-03 -9.869795000000e-03 9.999421000000e-01 -4.278459000000e-03 7.402527000000e-03 4.351614000000e-03 9.999631000000e-01
Tr_velo_to_cam: 7.533745000000e-03 -9.999714000000e-01 -6.166020000000e-04 -4.069766000000e-03 1.480249000000e-02 7.280733000000e-04 -9.998902000000e-01 -7.631618000000e-02 9.998621000000e-01 7.523790000000e-03 1.480755000000e-02 -2.717806000000e-01
Tr_imu_to_velo: 9.999976000000e-01 7.553071000000e-04 -2.035826000000e-03 -8.086759000000e-01 -7.854027000000e-04 9.998898000000e-01 -1.482298000000e-02 3.195559000000e-01 2.024406000000e-03 1.482454000000e-02 9.998881000000e-01 -7.997231000000e-01
```



## Running:
Navigate to the ros2 directory (where this file is located)

```shell
    colcon build --packages-select kitti_odometry_publisher

    #You can also use symlink-install if required
    colcon build --packages-select kitti_odometry_publisher --symlink-install 
```

You can now either run the nodes independently or from the launch file.

### Run manually from terminal

To start them from the terminal, open three terminals and in each of them paste one line from the following: 

Replace "PATH_TO_THE_SEQUENCE_YOU_WANT_TO_RUN" with the path to the sequence to be processed. Make sure it follows the KITTI convention, e.g. for sequence 00

```
00 \
    - image_0\
        -000000.png
        -000001.png
        - ...
    - image_1\
        -000000.png
        -000001.png
        - ...
    - velodyne\
        -000000.bin
        -000001.bin
        - ...
    - calib.txt
```

```shell
    # Pointcloud Publisher
    ros2 run kitti_odometry_publisher pcd_publisher --ros-args -p PathToSequence:=PATH_TO_THE_SEQUENCE_YOU_WANT_TO_RUN
    # You may another argument FramesPerSecond:=NUMBER_OF_FRAMES_PER_SECOND
    # Note: The default is set to 10 which is the default for Kitti Odometry Dataset

    # Image Publisher
    ros2 run kitti_odometry_publisher image_publisher --ros-args -p PathToSequence:=PATH_TO_THE_SEQUENCE_YOU_WANT_TO_RUN
    # You may another argument FramesPerSecond:=NUMBER_OF_FRAMES_PER_SECOND
    # Note: The default is set to 10 which is the default for Kitti Odometry Dataset

    # Overlay Publisher
    ros2 run kitti_odometry_publisher pcd_overlay_publisher --ros-args -p CalibPath:=PATH_TO_THE_SEQUENCE_YOU_WANT_TO_RUN
```

### Run from launchfile

To run from the launchfile open the launch file in ros2/kitti_odometry_publisher/launch/kitti_odometry_publisher.launch.py and adjust the file path (and fps if desired).
You may need to rebuild:

```shell
    colcon build --packages-select kitti_odometry_publisher
```

Next you can run the launch file: 
```shell
    ros2 launch kitti_odometry_publisher kitti_odometry_publisher.launch.py
```

To see the overlay images open rviz from a terminal 
```shell
    rviz2
```
In the left bottom corner click add. In the new window select the tab "By topic" and add image from /overlay_left.
