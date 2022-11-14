#!/bin/bash
pathDataset='/kitti/sequences/00/' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching Kitti RGB-D Mode"
./Examples/RGB-D/rgbd_kitti Vocabulary/ORBvoc.txt Examples/RGB-D/KITTI00-02.yaml "$pathDataset"

