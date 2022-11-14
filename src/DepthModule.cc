/*
Header:

Authors: Martin Rudolph & Florian Sauerbeck
Contact: martin.rudolph@tum.de, florian.sauerbeck@tum.de


The depthmodule acts as an interface between the LiDAR data given from outside the SLAM algorithm and the internal representation as depthmaps.

It contains the functionality for pointcloud projection and upsampling.

*/

#include "DepthModule.h"

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>

#include<fstream>
#include<iomanip>
#include <chrono>
#include <iostream>
#include <math.h>

namespace ORB_SLAM3
{

// Constructor
DepthModule::DepthModule(const std::string &strSettingPath, const int sensor){
    // Parse Parameters
    b_parse_LiDAR = false;
    b_parse_LiDAR = ParseRGBLParameters(strSettingPath);
    if(!b_parse_LiDAR){
        std::cout << "*Error in the LiDAR parameters in the config file*" << std::endl;
    }

    b_parse_LiDARUpsampling = false;
    b_parse_LiDARUpsampling = ParseUpsamplingParameters(strSettingPath);
    if(!b_parse_LiDARUpsampling){
        std::cout << "*Error in the LiDAR upsampling parameters in the config file*" << std::endl;
    }
}

// Destructor
DepthModule::~DepthModule(){
}

// Depth Module Master Function
void DepthModule::CalculateDepthFromPcd(std::vector<cv::KeyPoint> mvKeys, std::vector<cv::KeyPoint> mvKeysUn, const cv::Mat& PointCloud, const int imwidth, const int imheight){
    // Check if all required Parameters are available
    if(!b_parse_LiDARUpsampling || !b_parse_LiDAR){
        std::cout << "*Cannot perform LiDAR Upsampling since parameters were missing in the config file.*" << std::endl;
        return;
    }

    // Depth Map Projection
    ProjectPointcloudToImage (PointCloud, imwidth, imheight, opt_min_dist, opt_max_dist);

    // Upsampling
    if (SelectedUpsamlingMethod != DepthModule::None){
        switch (SelectedUpsamlingMethod){
            case DepthModule::NearestNeighborPixel:
                Upsample_NearestNeighbor_Pixel(mvKeys, mvKeysUn);
                break;
            case DepthModule::AverageFiltering:
                Upsample_AverageFiltering();
                DepthModule::GetFeatureDepthFromDepthMap(mvKeys, mvKeysUn);
                break;
            case DepthModule::InverseDilation:
                Upsample_InverseDilation();
                DepthModule::GetFeatureDepthFromDepthMap(mvKeys, mvKeysUn);
                break;
            default:
                std::cout << "*Desired Upsampling Method was not yet implemented.*";
                break;
        }
    }
}

// Test: Function from Frame
void DepthModule::GetFeatureDepthFromDepthMap(std::vector<cv::KeyPoint> mvKeys, std::vector<cv::KeyPoint> mvKeysUn){
    int N = mvKeys.size();

    mvuRight = std::vector<float>(N,-1);
    mvDepth = std::vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = ProcessedDepthMap.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

void DepthModule::ProjectPointcloudToImage (const cv::Mat& PointCloud, const int imwidth, const int imheight,  const float min_dist = 5, const float max_dist = 200){
    // Projects a Lidar Pointcloud given as a vector of Points (x,y,z,r) into a image, given the Projection Matrix  

    // Clear Depth Image
    RawDepthMap = cv::Mat::zeros(cv::Size(imwidth, imheight), CV_32F);

    // Perform Projection as Matrix multiplication of Projection Matrix e [3 x 4] and Points e [4 x n]
    // to obtain a matrix of projected points e [3 x n]
    cv::Mat ProjectedPointcloud;
    ProjectedPointcloud = LidarProjectionMatrix * PointCloud;

    // Normalization to pixel coordinates
    ProjectedPointcloud.row(0) = ProjectedPointcloud.row(0).mul((1 / ProjectedPointcloud.row(2)));
    ProjectedPointcloud.row(1) = ProjectedPointcloud.row(1).mul((1 / ProjectedPointcloud.row(2)));

    // Fill depth image
    float *u,*v,*d;
    for (int32_t i = 0; i < ProjectedPointcloud.cols; i++){
        // Retrieve Pixel Coordinate as pointer
        u = &ProjectedPointcloud.at<float>(0, i);
        v = &ProjectedPointcloud.at<float>(1, i);
        d = &ProjectedPointcloud.at<float>(2, i);

        // Check if the point lies in the image
        if (*u > 0 && *v > 0 && *u < imwidth && *v < imheight){
            // Check if point lies in the specified boundaries for the distance
            if (*d > min_dist && *d < max_dist){
                // Set the pixel value at the given position
                RawDepthMap.at<float>((int)*v, (int)*u) = (float)(*d);
            }
        }
    }
    // std::cout << RawDepthMap << std::endl;
}

// --------------------------------------------
// UPSAMPLING FUNCTIONS
// --------------------------------------------

void DepthModule::Upsample_NearestNeighbor_Pixel(std::vector<cv::KeyPoint> mvKeys, std::vector<cv::KeyPoint> mvKeysUn){
    // Upsamples a sparse image with nearest neighbor within a nxn patch
    // Detailled Explanation given in chapter 3.3.2 of Rudolph, Implementation of Depth Maps Generated from LiDAR Data in a Visual-SLAM
    // DenseDepthMap = Mat::zeros(DepthImage.rows, DepthImage.cols, CV_16U);

    // Add padding of size "max_radius"
    cv::Mat PaddedDepthMap;
    copyMakeBorder(RawDepthMap, PaddedDepthMap, ParamUpsampling_NearestNeighborPixel_SearchRadius, ParamUpsampling_NearestNeighborPixel_SearchRadius, ParamUpsampling_NearestNeighborPixel_SearchRadius, ParamUpsampling_NearestNeighborPixel_SearchRadius, cv::BORDER_CONSTANT, 0);
    // std::cout << "y: " << PaddedDepthMap.rows << " | x: " << PaddedDepthMap.cols << std::endl;

    // Do distance transform to get the correct search radius for each pixel
    cv::Mat Labels;
    cv::Mat DistMap;
    RawDepthMap.convertTo(DistMap, CV_8U);
    cv::threshold(DistMap, DistMap, 0, 1, 1); // Inverted Thresh (0 if > 1); threshold(src, dst, threshold_value, max_binary_value, type)
    cv::distanceTransform(DistMap, DistMap, Labels, ParamUpsampling_NearestNeighborPixel_DistType, ParamUpsampling_NearestNeighborPixel_MaskSize);

    // Help Variables
    int searchradius = 0;
    double min, max;
    float d;
    cv::Mat SearchBox;

    int N = mvKeys.size();

    mvuRight = std::vector<float>(N,-1);
    mvDepth = std::vector<float>(N,-1);

    // Iterate through keypoints and find depth value
    for(int i=0; i<N; i++){
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        // Get Search Radius for the current pixel
        searchradius = (int)DistMap.at<float>(v, u);

        // Get Depth Value for current pixel
        d = 0; // reset d
        if (searchradius >= 0 && searchradius < ParamUpsampling_NearestNeighborPixel_SearchRadius){
            searchradius++;
            SearchBox = PaddedDepthMap(cv::Rect(u+ParamUpsampling_NearestNeighborPixel_SearchRadius-searchradius, v+ParamUpsampling_NearestNeighborPixel_SearchRadius-searchradius, 2*searchradius, 2*searchradius)); //2*(searchradius+1), 2*(searchradius+1)));
            cv::minMaxLoc(SearchBox, &min, &max);
            d = (float)max;
        }

        if(d>0){
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

void DepthModule::Upsample_AverageFiltering(){
    // Takes sparse depth image and fills empty pixels by calculating the avg of nxn neighboring patch
    // Convolution with unity kernel (1 e [nxn]) and divide by number of non zero pixels in that patch
    // Detailled Explanation given in chapter 3.3.3 of Rudolph, Implementation of Depth Maps Generated from LiDAR Data in a Visual-SLAM

    int SquaredKernelSize = ParamUpsampling_AverageFilter_KernelSize * ParamUpsampling_AverageFilter_KernelSize;

    // Check that Raw Depthmap is initially type CV_32F
    // You may convert images by: SourceImage.convertTo(TargetImage, CV_32F, Factor);
    if (RawDepthMap.type() != 5){
        std::cerr << "Could not run FillPixels_nxn as input 'DepthMap' was not given as CV_32F." << std::endl;
        return;
    }


    // Convolve image with nxn kernel, normalized by the size of the kernel
    cv::Mat FilteredImage;
    cv::Mat kernel = cv::Mat::ones(ParamUpsampling_AverageFilter_KernelSize, ParamUpsampling_AverageFilter_KernelSize, CV_32F) / (SquaredKernelSize);
    cv::filter2D(RawDepthMap, FilteredImage, ParamUpsampling_AverageFilter_ddepth , kernel, ParamUpsampling_AverageFilter_Anchor, ParamUpsampling_AverageFilter_delta, cv::BORDER_DEFAULT);

    // Count number of Pixels in each patch
    cv::Mat PixelsPerPatch;
    cv::Mat UnitKernel = cv::Mat::ones(ParamUpsampling_AverageFilter_KernelSize, ParamUpsampling_AverageFilter_KernelSize, CV_16U);
    cv::threshold(RawDepthMap, PixelsPerPatch, 0, 1, 0); // Convert everything above 0 to 1
    cv::filter2D(PixelsPerPatch, PixelsPerPatch, ParamUpsampling_AverageFilter_ddepth, UnitKernel, ParamUpsampling_AverageFilter_Anchor, ParamUpsampling_AverageFilter_delta, cv::BORDER_DEFAULT); // holds number of non zero pixels in patch

    // Normalize the Filtered image by number of valid pixels per patch
    ProcessedDepthMap = FilteredImage.mul((SquaredKernelSize) / PixelsPerPatch);
}

void DepthModule::Upsample_InverseDilation(){
    cv::Mat kernel;
    // Initiate Kernel
    // Kernels: https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#gac2db39b56866583a95a5680313c314ad
    if (ParamUpsampling_InverseDilation_KernelType == "Rectangle"){
        kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(ParamUpsampling_InverseDilation_KernelSize_u,ParamUpsampling_InverseDilation_KernelSize_v));
    }else if (ParamUpsampling_InverseDilation_KernelType == "Cross"){
        kernel = getStructuringElement(cv::MORPH_CROSS, cv::Size(ParamUpsampling_InverseDilation_KernelSize_u,ParamUpsampling_InverseDilation_KernelSize_v));
    }else if (ParamUpsampling_InverseDilation_KernelType == "Ellipse"){
        kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ParamUpsampling_InverseDilation_KernelSize_u,ParamUpsampling_InverseDilation_KernelSize_v));
    }else if (ParamUpsampling_InverseDilation_KernelType == "Diamond"){
        // Holds data to initialize kernel for Diamon dilation
        if (ParamUpsampling_InverseDilation_KernelSize_u == 3){
            kernel = cv::Mat(cv::Size(ParamUpsampling_InverseDilation_KernelSize_u,ParamUpsampling_InverseDilation_KernelSize_u), CV_8U, DiamondKernelData_3);
        }else if(ParamUpsampling_InverseDilation_KernelSize_u == 5){
            kernel = cv::Mat(cv::Size(ParamUpsampling_InverseDilation_KernelSize_u,ParamUpsampling_InverseDilation_KernelSize_u), CV_8U, DiamondKernelData_5);
        }else if (ParamUpsampling_InverseDilation_KernelSize_u == 7){
            kernel = cv::Mat(cv::Size(ParamUpsampling_InverseDilation_KernelSize_u,ParamUpsampling_InverseDilation_KernelSize_u), CV_8U, DiamondKernelData_7);
        }else if (ParamUpsampling_InverseDilation_KernelSize_u == 9){
            kernel = cv::Mat(cv::Size(ParamUpsampling_InverseDilation_KernelSize_u,ParamUpsampling_InverseDilation_KernelSize_u), CV_8U, DiamondKernelData_9);
        }else{
            std::cerr << "Could not perform dilation, invalid kernel size for diamond kernel." << std::endl;
            return;
        }

        // if (ParamUpsampling_InverseDilation_KernelSize_u != ParamUpsampling_InverseDilation_KernelSize_v){
        //     std::cout << "Warning: KernelSize_u != KernelSize_v for diamond dilation. Only u is considered." << std::endl;
        // }
    }else{
        std::cerr << "Could not perform dilation, invalid kernel type: " << ParamUpsampling_InverseDilation_KernelType << std::endl;
        std::cout << "Valid and implemented kernel types are: Rectangle, Cross, Ellipse and Diamond" << std::endl;
        return;
    }

    // Invert Depth Data
    ProcessedDepthMap = (opt_max_dist * ParamUpsampling_InverseDilation_ScaleFactor) - RawDepthMap;
    cv::threshold(ProcessedDepthMap, ProcessedDepthMap, (opt_max_dist * ParamUpsampling_InverseDilation_ScaleFactor)-1, 0, cv::THRESH_TOZERO_INV);

    //Dialation
    cv::dilate(ProcessedDepthMap, ProcessedDepthMap, kernel, cv::Point(-1,-1), 1);

    // Reverse Inversion
    ProcessedDepthMap = (opt_max_dist * ParamUpsampling_InverseDilation_ScaleFactor) - ProcessedDepthMap;
    cv::threshold(ProcessedDepthMap, ProcessedDepthMap, (opt_max_dist * ParamUpsampling_InverseDilation_ScaleFactor)-1, 0, cv::THRESH_TOZERO_INV);
}


// --------------------------------------------
// PARAMETER PARSING FUNCTIONS
// --------------------------------------------

bool DepthModule::ParseRGBLParameters(const std::string &strSettingPath){
    // Load parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    // Initialize Target Matrix
    LidarProjectionMatrix = cv::Mat::zeros(cv::Size(4, 4), CV_32F);

    // Help Matrices
    cv::Mat CameraMatrix = cv::Mat::zeros(cv::Size(4, 3), CV_32F);
    cv::Mat RotationMatrix = cv::Mat::zeros(cv::Size(4, 4), CV_32F);

    // Fill Camera Matrix
    cv::FileNode node = fSettings["Camera.fx"];
    if(!node.empty() && node.isReal()){
        CameraMatrix.at<float>(0,0) = node.real();
    }
    else{
        std::cout << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }

    node = fSettings["Camera.fy"];
    if(!node.empty() && node.isReal()){
        CameraMatrix.at<float>(1,1) = node.real();
    }
    else{
        std::cout << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }

    node = fSettings["Camera.cx"];
    if(!node.empty() && node.isReal()){
        CameraMatrix.at<float>(0,2) = node.real();
    }
    else{
        std::cout << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }

    node = fSettings["Camera.cy"];
    if(!node.empty() && node.isReal()){
        CameraMatrix.at<float>(1,2) = node.real();
    }
    else{
        std::cout << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }

    CameraMatrix.at<float>(2,2) = 1;

    // Fill Rotation Matrix
    node = fSettings["LiDAR.Tr11"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(0,0) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr11 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }
    node = fSettings["LiDAR.Tr12"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(0,1) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr12 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }
    node = fSettings["LiDAR.Tr13"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(0,2) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr13 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }
    node = fSettings["LiDAR.Tr14"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(0,3) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr14 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }

    node = fSettings["LiDAR.Tr21"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(1,0) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr21 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }
    node = fSettings["LiDAR.Tr22"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(1,1) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr22 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }
    node = fSettings["LiDAR.Tr23"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(1,2) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr23 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }
    node = fSettings["LiDAR.Tr24"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(1,3) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr24 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }

    node = fSettings["LiDAR.Tr31"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(2,0) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr31 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }
    node = fSettings["LiDAR.Tr32"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(2,1) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr32 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }
    node = fSettings["LiDAR.Tr33"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(2,2) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr33 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }
    node = fSettings["LiDAR.Tr34"];
    if(!node.empty() && node.isReal()){
        RotationMatrix.at<float>(2,3) = node.real();
    }
    else{
        std::cout << "*LiDAR.Tr34 parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }

    RotationMatrix.at<float>(3,3) = 1;

    // Calculate Projection Matrix
    LidarProjectionMatrix = CameraMatrix * RotationMatrix;

    std::cout << CameraMatrix << std::endl;
    std::cout << RotationMatrix << std::endl;

    std::cout << std::endl << "LiDAR to Camera Projection Matrix: " << std::endl;
    std::cout << LidarProjectionMatrix << std::endl;

    // Read Other Parameters
    node = fSettings["LiDAR.min_dist"];
    if(!node.empty() && node.isReal()){
        opt_min_dist = node.real();
    }
    else{
        std::cout << "*LiDAR.min_dist parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }
    node = fSettings["LiDAR.max_dist"];
    if(!node.empty() && node.isReal()){
        opt_max_dist = node.real();
    }
    else{
        std::cout << "*LiDAR.max_dist parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }
    node = fSettings["Camera.bf"];
    if(!node.empty() && node.isReal()){
        mbf = node.real();
    }
    else{
        std::cout << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
        return false;
    }

    std::string MethodName = fSettings["LiDAR.Method"];
    std::cout << "Lidar Method: " << MethodName << std::endl;
    if(MethodName.empty()){
        std::cout << "*LiDAR.Method parameter doesn't exist*" << std::endl;
        return false;
    }
    else if (MethodName == "None"){
        SelectedUpsamlingMethod = UpsamlingMethod::None;
    }
    else if (MethodName == "NearestNeighborPixel"){
        SelectedUpsamlingMethod = UpsamlingMethod::NearestNeighborPixel;
    }
    else if (MethodName == "AverageFiltering"){
        SelectedUpsamlingMethod = UpsamlingMethod::AverageFiltering;
    }
    else if (MethodName == "InverseDilation"){
        SelectedUpsamlingMethod = UpsamlingMethod::InverseDilation;
    }
    else if (MethodName == "IPBasic"){
        SelectedUpsamlingMethod = UpsamlingMethod::IPBasic;
    }
    else{
        std::cout << "*LiDAR.Method parameter indicated an unknown option.*" << std::endl;
        return false;
    }
    

    // Return true if arrived here = sucessfully read all parameters
    return true;
}

bool DepthModule::ParseUpsamplingParameters(const std::string& strSettingPath){
    // Load parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    cv::FileNode node;

    // Read Parameters, depending on which Method is selected
    switch (SelectedUpsamlingMethod){
        case UpsamlingMethod::None:
            return true;
            break;

        // Parameters for Upsampling Method: NearestNeighborPixelLevel
        case UpsamlingMethod::NearestNeighborPixel:
            node = fSettings["LiDAR.MethodNearestNeighborPixel.SearchDistance"];
            if(!node.empty() && node.isReal()){
                ParamUpsampling_NearestNeighborPixel_SearchRadius = node.real();
            }
            else{
                std::cout << "*LiDAR.MethodNearestNeighborPixel.SearchDistance parameter doesn't exist or is not a real number*" << std::endl;
                return false;
            }
            return true;
            break;

        // Parameters for Upsampling Method: AverageFiltering
        case UpsamlingMethod::AverageFiltering:
            node = fSettings["LiDAR.MethodAverageFiltering.KernelSize"];
            if(!node.empty() && node.isReal()){
                ParamUpsampling_AverageFilter_KernelSize = node.real();
            }
            else{
                std::cout << "*LiDAR.MethodAverageFiltering.KernelSize parameter doesn't exist or is not a real number*" << std::endl;
                return false;
            }
            node = fSettings["LiDAR.MethodAverageFiltering.bDoDilationPreprocessing"];
            if(!node.empty() && node.real() == 0){
                ParamUpsampling_AverageFilter_bDoDilationPreprocessing = false; // Do no dilation as preprocessing
            }else if(!node.empty() && node.real() == 1){
                ParamUpsampling_AverageFilter_bDoDilationPreprocessing = true;  // Do dilation as preprocessing
            }else{
                std::cout << "*LiDAR.MethodAverageFiltering.bDoDilationPreprocessing parameter doesn't exist or is not 0 (false) or 1 (true)*" << std::endl;
                return false;
            }

            if (ParamUpsampling_AverageFilter_bDoDilationPreprocessing){
                // If Dilation as preprocessing is desired, parse also the other parameters
                node = fSettings["LiDAR.MethodAverageFiltering.DilationPreprocessing_KernelSize"];
                if(!node.empty() && node.isReal()){
                    ParamUpsampling_AverageFilter_DilationPreprocessing_KernelSize = node.real();
                }
                else{
                    std::cout << "*LiDAR.MethodAverageFiltering.DilationPreprocessing_KernelSize parameter doesn't exist or is not a real number*" << std::endl;
                    return false;
                }

                try{
                    std::string KernelType = fSettings["LiDAR.MethodAverageFiltering.DilationPreprocessing_KernelType"];
                    ParamUpsampling_AverageFilter_DilationPreprocessing_KernelType = KernelType;
                }catch(const std::exception& e){                    
                    std::cout << "*LiDAR.MethodAverageFiltering.DilationPreprocessing_KernelType parameter doesn't exist or is not a string*" << std::endl;
                    std::cerr << e.what() << '\n';                    
                    return false;
                }
            }

        // Parameters for Upsampling Method: AverageFiltering
        case UpsamlingMethod::InverseDilation:
            node = fSettings["LiDAR.MethodInverseDilation.KernelSize_u"];
            if(!node.empty() && node.isReal()){
                ParamUpsampling_InverseDilation_KernelSize_u = node.real();
            }
            else{
                std::cout << "*LiDAR.MethodInverseDilation.KernelSize_u parameter doesn't exist or is not a real number*" << std::endl;
                return false;
            }

            node = fSettings["LiDAR.MethodInverseDilation.KernelSize_v"];
            if(!node.empty() && node.isReal()){
                ParamUpsampling_InverseDilation_KernelSize_v = node.real();
            }
            else{
                std::cout << "*LiDAR.MethodInverseDilation.KernelSize_v parameter doesn't exist or is not a real number*" << std::endl;
                return false;
            }

            try{
                std::string KernelTypeDilation = fSettings["LiDAR.MethodInverseDilation.KernelType"];
                ParamUpsampling_InverseDilation_KernelType = KernelTypeDilation;
            }catch(const std::exception& e){                    
                std::cout << "*LiDAR.MethodInverseDilation.KernelType parameter doesn't exist or is not a string*" << std::endl;
                std::cerr << e.what() << '\n';                    
                return false;
            }

            return true;
            break;
        
        default:
            // Not defined Option -> return false
            return false;
            break;
    }
}

} // Namespace ORB_SLAM3