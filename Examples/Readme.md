**Examples**

In this part of the repo, several example scripts for ORB-SLAM are given. They are mostly created by the original authors and were, were required, adapted for the needs of this project.
Throughout the thesis, mostly KITTI data was used, such that the respective examples are most used and hence also stable.

The folder contains folders for the following setups:
- Stereo
- RGB-D (for the use of previously generated depth maps)
- RGB-L (loads image and pointcloud and does upsampling online)


**Instrucions to use the examples for KITTI**
0. Download KITTI odometry dataset and unpack it as intended by the KITTI organizers (see bottom)
1. Navigate to the ORBSLAM repository
2. Run the code for the respective example
    - Stereo (requires image_0 and image_1)

    `./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI00-02.yaml <<Path to sequence>>`
    - RGB-D (requires image_x and image_x_Depth with x = 0..3)

    `./Examples/RGB-D/rgbd_kitti Vocabulary/ORBvoc.txt Examples/RGB-D/KITTI00-02.yaml <<Path to sequence>>`
    - RGB-L (requires image_x and velodyne with x = 0..3)

    `./Examples/RGBL/rgbl_kitti Vocabulary/ORBvoc.txt Examples/RGBL/KITTI00-02.yaml <<Path to sequence>>`

    Make sure to use the correct *.yaml for the respective sequences (e.g. KITTI00-02.yaml for sequences 00, 01, and 02)
3. The run should start and after sucessful completion save the trajectory to "CameraTrajectory.txt"
4. You may now evaluate the trajectory with the KITTI tool or the [simplified Python version](https://github.com/Huangying-Zhan/kitti-odom-eval)

_Exemplary KITTI folder structure_

- odometry dataset/ _(may be called as desired)_
    - devkit/
    - dataset/
        - poses/
        - sequences/
            - 00/       _(give path to this level, to run the example scripts)_
                - calib.txt
                - times.txt
                - image_0/
                - image_1/
                - image_2/
                - image_3/
                - velodyne/
            - ...
            - 21

In case you want to use the RGB-D example, the depth maps must be stored in the folder as called from the respective example script.
For example, the standard path implemented here is image_0_Depth. The depthmaps must have the same names, as the corresponding RGB/grayscale images.
