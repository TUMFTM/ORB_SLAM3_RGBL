#!/bin/bash
echo "Starting processing of KITTI Depth Completion Sequences"

Executable="ros2 run depth_map_creator kittipublisher --ros-args -p PublishFrequency:=200.0 -p SaveDenseDepth:=true -p CreateOverlay:=true -p optDataset:=Completion -p SequencePath:="

##Read and verify the desired SLAM Mode
#read -p "Enter Results Folder Name: " FolderName
#mkdir "$FolderName"

## Sequence Paths
declare -a sequences=( "/home/martin/data/datasets/03_KITTI/01_DepthCompletion/selection/Drive0004/"	
			"/home/martin/data/datasets/03_KITTI/01_DepthCompletion/selection/Drive0009/"
			"/home/martin/data/datasets/03_KITTI/01_DepthCompletion/selection/Drive0028/"
			"/home/martin/data/datasets/03_KITTI/01_DepthCompletion/selection/Drive0032/")
	
for SequencePath in "${sequences[@]}"
do
   	$Executable"$SequencePath"
done

