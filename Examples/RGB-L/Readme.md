**RGBL Readme**

These example scripts are intended to be used for the KITTI odometry dataset.
In order to use the scripts, please make sure:

- that you have correctly setup and built your ORBSLAM version
- you have downloaded the [KITTI odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php), especially the camera images you want to use and the LiDAR section
- the structure of your downloaded dataset is as intended by KITTI (for an example see the readme in the overall examples folder)

To run the example, navigate to the ORBSLAM folder and run:

`./Examples/RGB-L/rgbl_kitti Vocabulary/ORBvoc.txt Examples/RGB-L/KITTI00-02.yaml <<Path to sequence>>`

Make sure to use the correct *.yaml file for the respective sequence.

The upsampling method to be used is selected in the *.yaml files. Each method requires different parameters.
It is best practice, to leave the unused parameters unchanged, such that the example script may be easily used for all methods.
