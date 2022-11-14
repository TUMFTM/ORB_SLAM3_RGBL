#!/bin/bash
pathDataset='/kitti/sequences/00/' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching Kitti Stereo Mode"
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI00-02.yaml "$pathDataset"

