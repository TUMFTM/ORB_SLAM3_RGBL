#!/bin/bash
echo "Starting processing of KITTI Odometry Sequences"

# Executable="ros2 run depth_map_creator kittipublisher --ros-args -p PublishFrequency:=20.0 -p SaveDenseDepth:=true -p CreateOverlay:=true -p optDataset:=Odometry -p SequencePath:="

Executable="ros2 run depth_map_creator kittipublisher --ros-args -p PublishFrequency:=30.0 -p SaveDenseDepth:=true -p optDataset:=Odometry -p SequencePath:="

##Read and verify the desired SLAM Mode
#read -p "Enter Results Folder Name: " FolderName
#mkdir "$FolderName"

## Sequence Paths
declare -a sequences=( 	
						# "/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/00/"	
 						# "/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/01/"	
 						# "/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/02/"	
 						# "/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/03/"	
 						"/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/04/"	
 						# "/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/05/"	
 						# "/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/06/"	
 						# "/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/07/"	
 						# "/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/08/"	
 						# "/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/09/"	
 						# "/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/10/"
			)
	
for SequencePath in "${sequences[@]}"
do
	mkdir "$SequencePath""/image_2_Depth_DDIL"
	mkdir "$SequencePath""/image_2_Depth_Overlay"
   	$Executable"$SequencePath"
done

