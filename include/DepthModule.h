/*
Header:

Author: Martin Rudolph
Contact: martin.rudolph@tum.de

The depthmodule acts as an interface between the LiDAR data given from outside the SLAM algorithm and the internal representation as depthmaps.

It contains the functionality for pointcloud projection and upsampling.

*/

#ifndef DEPTHMODULE_H
#define DEPTHMODULE_H


#include <opencv2/core/core.hpp>
#include "opencv2/imgproc.hpp"


#include<fstream>
#include<iomanip>
#include <chrono>
#include <iostream>

namespace ORB_SLAM3
{

class System;

class DepthModule{
    // Custom Definitions
    public:
        enum UpsamlingMethod{
            None                    = 0,
            NearestNeighborPixel    = 1,
            AverageFiltering        = 2,
            InverseDilation         = 3,
            IPBasic                 = 5
        };

    //Constructors
    public:
        // Constructor
        DepthModule(const std::string& strSettingPath, const int sensor);

        // Destructor
        ~DepthModule();

    // Parameter Parsing
    private:
        // Parsing Parameter File
        bool ParseRGBLParameters(const std::string& strSettingPath);

        // Parsing Upsampling Method Options  Files
        bool ParseUpsamplingParameters(const std::string& strSettingPath);


    // Depth Value Handler, manages depth calculation for each frame
    public:
        // Depth Value Handler, manages depth calculation for each frame
        void CalculateDepthFromPcd(std::vector<cv::KeyPoint> mvKeys, std::vector<cv::KeyPoint> mvKeysUn, const cv::Mat& PointCloud, const int imwidth, const int imheight);

    // Upsampling Functions
    private:
        //Test from Frame for RGBD
        void GetFeatureDepthFromDepthMap(std::vector<cv::KeyPoint> mvKeys, std::vector<cv::KeyPoint> mvKeysUn);

        // Upsampling Based on Nearest Neighbor interpolation for desired pixels only
        void Upsample_NearestNeighbor_Pixel(std::vector<cv::KeyPoint> mvKeys, std::vector<cv::KeyPoint> mvKeysUn);

        // Upsampling Based on Mean Filter 
        void Upsample_AverageFiltering();

        // Upsampling Based on Inverse Dilation
        void Upsample_InverseDilation();

        // Upsampling Based on IP Basic from Jason Ku: https://github.com/kujason/ip_basic
        void Upsample_IPBasic();

    // Functions
    protected:
        // Projects a cv::Mat Pointcloud (each column represents a point) to an image frame
        void ProjectPointcloudToImage (const cv::Mat& PointCloud, const int imwidht, const int imheight,  const float min_dist, const float max_dist);

    //Variables
    public:
        // Projection Matrix from LiDAR to camera frame of reference
        cv::Mat LidarProjectionMatrix;

        // Depthmap as obtained from LiDAR pcd projection (raw, i.e. no further processing)
        cv::Mat RawDepthMap;

        // Depthmap after Upsampling, not necessarily dense depending on the method that was used
        cv::Mat ProcessedDepthMap;

        // Test: RGBD Function from Frame
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;

    //Variables
    protected:
        // Indicates whether parameters were sucessfully read
        bool b_parse_LiDAR, b_parse_LiDARUpsampling;

        // 3D Baseline, only required to store depth values, has no physical meaning
        float mbf;

        // Defines which method is used for upsamling
        UpsamlingMethod SelectedUpsamlingMethod;

        // Minimum and Maximum Distance to be considered during pointcloud projection
        float opt_min_dist, opt_max_dist;

        // Parameters Required for Upsampling Method: Nearest Neighbor Pixel Based
        float ParamUpsampling_NearestNeighborPixel_SearchRadius;
        int ParamUpsampling_NearestNeighborPixel_MaskSize = cv::DIST_MASK_5;
        int ParamUpsampling_NearestNeighborPixel_DistType = cv::DIST_L2;

        // Parameters Required for Upsampling Method: Average Filtering
        cv::Point ParamUpsampling_AverageFilter_Anchor = cv::Point(-1, -1);
        int ParamUpsampling_AverageFilter_delta = 0, ParamUpsampling_AverageFilter_ddepth = -1;
        int ParamUpsampling_AverageFilter_KernelSize = 0;
        bool ParamUpsampling_AverageFilter_bDoDilationPreprocessing = false;
        std::string ParamUpsampling_AverageFilter_DilationPreprocessing_KernelType = "";
        int ParamUpsampling_AverageFilter_DilationPreprocessing_KernelSize = 0;

        // Parameters Required for Upsampling Method: InverseDilation
        std::string ParamUpsampling_InverseDilation_KernelType = "";
        float ParamUpsampling_InverseDilation_ScaleFactor = 1.0;    // Might be required, if  a scaling is performed in the projection step. Currently unused.
        int ParamUpsampling_InverseDilation_KernelSize_u = 0, ParamUpsampling_InverseDilation_KernelSize_v = 0;

        // Counter for number of frames processed in the session
        int FrameCounter = 0;

    private:
        // Diamond Kernel Data
        uchar DiamondKernelData_3[9] = {    0, 1, 0,
                                            1, 1, 1,
                                            0, 1, 0};
        uchar DiamondKernelData_5[25] = {   0, 0, 1, 0, 0,
                                            0, 1, 1, 1, 0,
                                            1, 1, 1, 1, 1,
                                            0, 1, 1, 1, 0,
                                            0, 0, 1, 0, 0};
        uchar DiamondKernelData_7[49] = {   0, 0, 0, 1, 0, 0, 0,
                                            0, 0, 1, 1, 1, 0, 0,
                                            0, 1, 1, 1, 1, 1, 0,
                                            1, 1, 1, 1, 1, 1, 1,
                                            0, 1, 1, 1, 1, 1, 0,
                                            0, 0, 1, 1, 1, 0, 0,
                                            0, 0, 0, 1, 0, 0, 0};
        uchar DiamondKernelData_9[81] = {   0, 0, 0, 0, 1, 0, 0, 0, 0,
                                            0, 0, 0, 1, 1, 1, 0, 0, 0,
                                            0, 0, 1, 1, 1, 1, 1, 0, 0,
                                            0, 1, 1, 1, 1, 1, 1, 1, 0,
                                            1, 1, 1, 1, 1, 1, 1, 1, 1,
                                            0, 1, 1, 1, 1, 1, 1, 1, 0,
                                            0, 0, 1, 1, 1, 1, 1, 0, 0,
                                            0, 0, 0, 1, 1, 1, 0, 0, 0,
                                            0, 0, 0, 0, 1, 0, 0, 0, 0};

};
} // Namespace ORB_SLAM3
#endif // DEPTHMODULE_H