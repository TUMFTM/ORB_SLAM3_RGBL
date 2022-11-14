//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include<fstream>
#include<iomanip>
#include <chrono>

// User Defined Types
struct CalibMatrices{cv::Mat P0, P1, P2, P3, Tr;};

class KittiPublisherNode : public rclcpp::Node{
    public:
        KittiPublisherNode() : Node("kitti_publisher"){
            // Parameters
            // To set parameters append "--ros-args -p ParamName:=Value" to the call
            // e.g. ros2 run depth_map_creator kittipublisher --ros-args -p PublishFrequency:=10.0 -p SaveDenseDepth:=true -p CreateOverlay:=true -p SequencePath:=/home/ << Path to Dataset >>/dataset/sequences/00/

            this->declare_parameter("SequencePath", "");
            this->declare_parameter("PublishFrequency", 10.0);
            this->declare_parameter("SaveSparseDepth", false);
            this->declare_parameter("SaveDenseDepth", false);
            this->declare_parameter("CreateOverlay", false);
            this->declare_parameter("optDataset", "Odometry");

            SequencePath = this->get_parameter("SequencePath").as_string();
            Frequency = this->get_parameter("PublishFrequency").as_double();
            SaveSparseDepth = this->get_parameter("SaveSparseDepth").as_bool();
            SaveDenseDepth = this->get_parameter("SaveDenseDepth").as_bool();
            CreateOverlay = this->get_parameter("CreateOverlay").as_bool();
            optDataset = this->get_parameter("optDataset").as_string();

            // Publishers
            pub_image_0 = this->create_publisher<sensor_msgs::msg::Image>("image_0", 10);
            pub_image_1 = this->create_publisher<sensor_msgs::msg::Image>("image_1", 10);
            pub_image_2 = this->create_publisher<sensor_msgs::msg::Image>("image_2", 10);
            pub_image_3 = this->create_publisher<sensor_msgs::msg::Image>("image_3", 10);

            // pub_lidar = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne", 10);

            pub_depthmap_0 = this->create_publisher<sensor_msgs::msg::Image>("depthmap/sparse/image_0", 10);
            pub_depthmap_1 = this->create_publisher<sensor_msgs::msg::Image>("depthmap/sparse/image_1", 10);
            pub_depthmap_2 = this->create_publisher<sensor_msgs::msg::Image>("depthmap/sparse/image_2", 10);
            pub_depthmap_3 = this->create_publisher<sensor_msgs::msg::Image>("depthmap/sparse/image_3", 10);

            // Timer
            timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000/Frequency)), std::bind(&KittiPublisherNode::publishFrame, this));

            // Terminal Info
            RCLCPP_INFO(this->get_logger(), "All publishers have been started.");
            // RCLCPP_INFO(this->get_logger(), "Processing Sequence: " + SequencePath);

            // Read Calib
            LoadTimestamps(SequencePath, vTimestamps);
            std::cout << "Done Loading Timestamps" << std::endl;

            // Check options
            if (optDataset != "Odometry" && optDataset != "Completion" && optDataset != "Virtual"){
                std::cerr << "Invalid Dataset Option! Please chose Odometry, Completion or Virtual." << std::endl;
            }

            // Calibration
            if (optDataset == "Odometry"){
                ReadKittiCalib (SequencePath + "calib.txt" , SequenceCalibration);
                ProjMat0 = CalcProjectionMatrix (0, SequenceCalibration);
                ProjMat1 = CalcProjectionMatrix (1, SequenceCalibration);
                ProjMat2 = CalcProjectionMatrix (2, SequenceCalibration);
                ProjMat3 = CalcProjectionMatrix (3, SequenceCalibration);
            }

            // Increment Frame counter, if working with completion dataset
            if (optDataset == "Completion"){
                FrameCounter = 5; // Depth only available from 5th frame
            }

            // Create Output Folders
            if (SaveDenseDepth){
                system(("mkdir " + SequencePath + "/image_0_Depth").c_str());
                // system(("mkdir " + SequencePath + "/image_1_Depth").c_str());
                // system(("mkdir " + SequencePath + "/image_2_Depth").c_str());
                // system(("mkdir " + SequencePath + "/image_3_Depth").c_str());
            }
            if (CreateOverlay){
                system(("mkdir " + SequencePath + "/image_0_Depth_Overlay").c_str());
                // system(("mkdir " + SequencePath + "/image_1_Depth_Overlay").c_str());
                // system(("mkdir " + SequencePath + "/image_2_Depth_Overlay").c_str());
                // system(("mkdir " + SequencePath + "/image_3_Depth_Overlay").c_str());
            }
        }

    private:
        // ===============================================
        // Subscriber and Publisher; Processing Call
        void publishFrame(){
            // Update Frame Name
            FrameName.str(std::string());   // Clear
            if (optDataset == "Odometry"){
                FrameName << std::setfill('0') << std::setw(6) << FrameCounter;  // Odometry Dataset

                // Load Images
                LoadImageGray (SequencePath + "/image_0/" + FrameName.str() + ".png", img_0);            // Odometry Dataset
                // LoadImageGray (SequencePath + "/image_1/" + FrameName.str() + ".png", img_1);            // Odometry Dataset
                // LoadImageGray (SequencePath + "/image_2/" + FrameName.str() + ".png", img_2);            // Odometry Dataset
                // LoadImageGray (SequencePath + "/image_3/" + FrameName.str() + ".png", img_3);            // Odometry Dataset
            }else if (optDataset == "Completion"){
                FrameName << std::setfill('0') << std::setw(10) << FrameCounter;    // Depth Completion Dataset

                // Load Images
                LoadImageGray (SequencePath + "/image_00/data/" + FrameName.str() + ".png", img_0);      // Depth Completion Dataset
                LoadImageGray (SequencePath + "/image_01/data/" + FrameName.str() + ".png", img_1);         // Depth Completion Dataset
                LoadImageGray (SequencePath + "/image_02/data/" + FrameName.str() + ".png", img_2);         // Depth Completion Dataset
                LoadImageGray (SequencePath + "/image_03/data/" + FrameName.str() + ".png", img_3);         // Depth Completion Dataset
            }else if (optDataset == "Virtual"){
                FrameName << std::setfill('0') << std::setw(5) << FrameCounter;  // Virtual Dataset

                // Load Images
                LoadImageGray (SequencePath + "/rgb/Camera_0/rgb_" + FrameName.str() + ".jpg", img_0);         // Virtual KITTI
            }

            // Load Pointcloud
            if  (optDataset == "Odometry"){
                LoadPointcloudBinaryMat(SequencePath + "/velodyne/" + FrameName.str() + ".bin", pcd);            // Odometry Dataset

                // Do Projection of PCD
                // Start Timer
                // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                //Projection
                ProjectPointcloudToImage (ProjMat0, img_0, pcd, depthmap_0);
                // ProjectPointcloudToImage (ProjMat1, img_1, pcd, depthmap_1);
                // ProjectPointcloudToImage (ProjMat2, img_2, pcd, depthmap_2);
                // ProjectPointcloudToImage (ProjMat3, img_3, pcd, depthmap_3);
                
                // End Timer, calc required time and output
                // std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();
                // std::cout << "Projection took:" << std::chrono::duration_cast<std::chrono::milliseconds>(end1 - begin).count() << " ms" << std::endl;
            }
            // LoadPointcloudBinaryMat(SequencePath + "/velodyne_points/data/" + FrameName.str() + ".bin", pcd);   // Depth Completion Dataset -> Default: Load Sparse Depthmap



            // Alternatively for Depth Completion Dataset: Load already projected sparse depth map
            // Caution: Only exists for image 2 and image 3
            if (optDataset == "Completion"){
                LoadImageGray (SequencePath + "/proj_depth/velodyne_raw/image_02/" + FrameName.str() + ".png", depthmap_2);  // Depth Completion Dataset only
                // LoadImageGray (SequencePath + "/proj_depth/velodyne_raw/image_02/" + FrameName.str() + ".png", depthmap_3);  // Depth Completion Dataset only

                depthmap_2.convertTo(depthmap_2, CV_16UC1, 65535/255);    // Format Change and scaling required to work within this environment
                // depthmap_3.convertTo(depthmap_2, CV_16UC1, 65535/255);    // Format Change and scaling required to work within this environment
            }else if (optDataset == "Virtual"){
                LoadDepthmap (SequencePath + "/Camera_0_Depth_Sparse/depth_" + FrameName.str() + ".png", depthmap_0);  // Virtual Dataset only
            }

            // Shutdown
            if ((depthmap_0.empty() && optDataset == "Odometry") || (depthmap_2.empty() && optDataset == "Completion") || (depthmap_0.empty() && optDataset == "Virtual")){ // TODO exit for odometry
                // End execution if end is reached
                if (FrameCounter > (int) vTimestamps.size()){
                    std::cout << "=============" << std::endl;
                    std::cout << "Avg Time: " << AvgTime << std::endl;
                    std::cout << "=============" << std::endl;
                    rclcpp::shutdown();
                }

                std::cout << "Could not load DepthMap " << FrameName.str() << std::endl;
                FrameCounter++;

                return;
            }


            // Upsampling Timer Start
            std::chrono::steady_clock::time_point TStartUpsampling = std::chrono::steady_clock::now();

            // Dilation
            // DialateDepthmapInverted(depthmap_0, "Diamond", 5,5);
            // DialateDepthmap(depthmap_1, "Rectangle", 5, 5);
            // DialateDepthmap(depthmap_2, "Rectangle",3,3);
            // DialateDepthmap(depthmap_3, "Rectangle", 5, 5);

            // DialateDepthmapInverted(depthmap_2, "Diamond", 5,5);
            // DialateDepthmap(depthmap_2, "Diamond",3,3);
            // FillPixels_nxn (depthmap_2, 3);

            // Inverted Dilation
            DialateDepthmapInverted(depthmap_0, "Diamond", 5,5);
            // DialateDepthmapInverted(depthmap_1, "Diamond", 7,7);
            // DialateDepthmapInverted(depthmap_2, "Diamond", 5,5);
            // DialateDepthmapInverted(depthmap_3, "Diamond", 7,7);

            // Upsampling - Nearest Neighbor
            // NearestNeighbor (depthmap_0, 7);
            // NearestNeighbor (depthmap_1, 7);
            // NearestNeighbor (depthmap_2, 3);
            // NearestNeighbor (depthmap_3, 7);

            // Mean Filtering
            // FillPixels_nxn (depthmap_0, 5);
            // FillPixels_nxn (depthmap_1, 10);
            // FillPixels_nxn (depthmap_2, 5);
            // FillPixels_nxn (depthmap_3, 10);

            // Edge Preserving Nearest Neighbor
            // EdgePreservingNearestNeighbor(depthmap_0, img_0, 5);
            // EdgePreservingNearestNeighbor(depthmap_1, img_1, 5);
            // EdgePreservingNearestNeighbor(depthmap_2, img_2, 7);
            // EdgePreservingNearestNeighbor(depthmap_3, img_3, 5);

            // Fill Upper Region
            // FillUpperRegion(depthmap_0);
            // FillUpperRegion(depthmap_1);
            // FillUpperRegion(depthmap_2);
            // FillUpperRegion(depthmap_3);

            // Column Interpolation
            // interpolation_cols(depthmap_0);
            // interpolation_cols(depthmap_1);
            // interpolation_cols(depthmap_2);
            // interpolation_cols(depthmap_3);

            // Row Interpolation
            // interpolation_rows(depthmap_0);
            // interpolation_rows(depthmap_1);
            // interpolation_rows(depthmap_2);
            // interpolation_rows(depthmap_3);

            // Column Extrapolation Upwards
            // ExtrapolateColsUpwards(depthmap_2);

            // Average Blur
            // AverageBlur(depthmap_0, 10);
            // AverageBlur(depthmap_1, 10);
            // AverageBlur(depthmap_2, 10);
            // AverageBlur(depthmap_3, 10);

            // Bilateral Filter
            // BilateralFilter(depthmap_0, 5, 75, 50);
            // BilateralFilter(depthmap_1, 5, 75, 50);
            // BilateralFilter(depthmap_2, 5, 75, 50);
            // BilateralFilter(depthmap_3, 5, 75, 50);

            // IPBasic
            // IPBasic(depthmap_0, 100, 256, "Diamond", 5, true, "Bilateral");
            // IPBasic(depthmap_1, 100, 256, "Diamond", 5, true, "Bilateral");
            // IPBasic(depthmap_2, 100, 256, "Diamond", 5, true, "Bilateral");
            // IPBasic(depthmap_3, 100, 256, "Diamond", 5, true, "Bilateral");

            // End Timer, calc required time and output
            std::chrono::steady_clock::time_point TEndUpsamling = std::chrono::steady_clock::now();
            // std::cout << "Upsampling took:" << std::chrono::duration_cast<std::chrono::milliseconds>(TEndUpsamling - TStartUpsampling).count() << " ms" << std::endl;

            // in depth dataset first frame is 5
            float TimeForFrame = std::chrono::duration_cast<std::chrono::milliseconds>(TEndUpsamling - TStartUpsampling).count();
            AvgTime = (AvgTime * (FrameCounterTime) + TimeForFrame) / (FrameCounterTime+1);
            FrameCounterTime ++;

            // Save to File
            if (SaveDenseDepth) {
                if (!depthmap_0.empty()){
                    SaveDepthmapToFile (depthmap_0, "image_0_Depth");
                }
                if (!depthmap_1.empty()){
                    SaveDepthmapToFile (depthmap_1, "image_1_Depth");
                }
                if (!depthmap_2.empty()){
                    SaveDepthmapToFile (depthmap_2, "image_2_Depth");
                }
                if (!depthmap_3.empty()){
                    SaveDepthmapToFile (depthmap_3, "image_3_Depth");
                }
            }

            // Create Overlay
            if (CreateOverlay){
                if (!depthmap_0.empty() && !img_0.empty()){
                    ImageDepthmapOverlay(img_0, depthmap_0, Overlay_0, OverlayGain, 0);
                }
                if (!depthmap_1.empty() && !img_1.empty()){
                    ImageDepthmapOverlay(img_1, depthmap_1, Overlay_1, OverlayGain, 1);
                }
                if (!depthmap_2.empty() && !img_2.empty()){
                    ImageDepthmapOverlay(img_2, depthmap_2, Overlay_2, OverlayGain, 2);
                }
                if (!depthmap_3.empty() && !img_3.empty()){
                    ImageDepthmapOverlay(img_3, depthmap_3, Overlay_3, OverlayGain, 3);
                }
            }

            // ROS2 Interface
            // Create Messages
            // convert_frame_to_message(img_0, msg_image_0);
            // convert_frame_to_message(img_1, msg_image_1);
            // convert_frame_to_message(img_2, msg_image_2);
            // convert_frame_to_message(img_3, msg_image_3);

            // convert_frame_to_message(depthmap_0, msg_depthmap_0);
            // convert_frame_to_message(depthmap_1, msg_depthmap_1);
            // convert_frame_to_message(depthmap_2, msg_depthmap_2);
            // convert_frame_to_message(depthmap_3, msg_depthmap_3);

            // convert_pcd_to_message(pcd, msg_pcd);

            // Publish
            // pub_image_0->publish(msg_image_0);
            // pub_image_1->publish(msg_image_1);
            // pub_image_2->publish(msg_image_2);
            // pub_image_3->publish(msg_image_3);

            // pub_depthmap_0->publish(msg_depthmap_0);
            // pub_depthmap_1->publish(msg_depthmap_1);
            // pub_depthmap_2->publish(msg_depthmap_2);
            // pub_depthmap_3->publish(msg_depthmap_3);

            // pub_lidar->publish(msg_pcd);

            // Feedback
            // RCLCPP_INFO(this->get_logger(), "Published Frame " + FrameName.str());

            // Increment Counter
            FrameCounter++;
        }

    // ===========================================================================================================================
    // ===========================================================================================================================
    // KITTI SPECIFIC FUNCTIONS
    // ===========================================================================================================================
    // ===========================================================================================================================
        bool LoadTimestamps (const std::string strPathToSequence, std::vector<double> &vTimestamps){
            using namespace std;

            cout << "Start Loading TimeStamps" << endl;
            ifstream fTimes;
            std::string strPathTimeFile;
            if (optDataset == "Odometry"){
                strPathTimeFile = strPathToSequence + "/times.txt"; // Odometry KITTI
            } else if (optDataset == "Completion"){
                strPathTimeFile = strPathToSequence + "/image_02/timestamps.txt"; // Depth Completion KITTI
            } else if (optDataset == "Virtual"){
                strPathTimeFile = strPathToSequence + "/timestamps.txt"; // Virtual
            }

            fTimes.open(strPathTimeFile);
            if (fTimes.is_open()){
                cout << strPathTimeFile << endl;
                while (!fTimes.eof()){
                    string s;
                    getline(fTimes, s);
                    if (!s.empty()){
                        stringstream ss;
                        ss << s;
                        double t;
                        ss >> t;
                        vTimestamps.push_back(t);
                    }
                }
                return true;
            }
            std::cout << "Could not open timestamps file: " << strPathTimeFile << std::endl;
            return false;
        }


        bool LoadImageGray (const std::string& FilePath, cv::Mat& img){
            //Reads an image to a cv::Mat object and returns true in case it was sucessful.

            // Load Image
            img = cv::imread(FilePath, cv::IMREAD_GRAYSCALE); // Load an image
            img.convertTo(img, CV_16SC1);

            // Check if loading was sucessful
            if(img.empty()){
                std::cerr << "Could not open or find the image: " << FilePath << std::endl;
                return false;
            }

            // cout << "Read Grayscale Image with resolution " << img.cols << " x " << img.rows << endl;
            return true;
        }

        bool LoadDepthmap (const std::string& FilePath, cv::Mat& img){
            //Reads an image to a cv::Mat object and returns true in case it was sucessful.

            // Load Image
            img = cv::imread(FilePath, cv::IMREAD_UNCHANGED); // Load an image

            // Check if loading was sucessful
            if(img.empty()){
                std::cerr << "Could not open or find the image: " << FilePath << std::endl;
                return false;
            }

            // cout << "Read Grayscale Image with resolution " << img.cols << " x " << img.rows << endl;
            return true;
        }

        bool LoadImageRGB (const std::string& FilePath, cv::Mat& img){
            //Reads an image to a cv::Mat object and returns true in case it was sucessful.

            // Load Image
            img = cv::imread(FilePath, cv::IMREAD_COLOR); // Load an image
            img.convertTo(img, CV_8UC3);

            // Check if loading was sucessful
            if(img.empty()){
                std::cerr << "Could not open or find the image: " << FilePath << std::endl;
                return false;
            }

            // std::cout << "Read RGB Image with resolution " << img.cols << " x " << img.rows << std::endl;
            return true;
        }

        std::string mat_type2encoding(int mat_type){
            switch (mat_type) {
                case CV_8UC1:
                return "mono8";
                case CV_8UC3:
                return "bgr8";
                case CV_16SC1:
                return "mono16";
                case CV_16U:
                return "mono16";
                case CV_8UC4:
                return "rgba8";
                default:
                throw std::runtime_error("Unsupported encoding type");
            }
        }

        void convert_frame_to_message(const cv::Mat & frame, sensor_msgs::msg::Image & msg){
            // copy cv information into ros message
            msg.height = frame.rows;
            msg.width = frame.cols;
            msg.encoding = mat_type2encoding(frame.type()); //"mono16"
            msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            size_t size = frame.step * frame.rows;
            msg.data.resize(size);
            memcpy(&msg.data[0], frame.data, size);
            msg.header.frame_id = "map";
            msg.header.stamp = this->now();
        }

        bool LoadPointcloudBinaryMat(const std::string& FilePath, cv::Mat& point_cloud){
            // Code is mainly copied from the Readme of Kitti Odometry Dataset
            // From there on I (Martin), adapted the code for my purposes (save as cv::Mat not as vector)

            // Initialization
            int32_t num = 1000000; // maximum Number of points to allocate
            float* data = (float*) malloc(num * sizeof(float));
            float* px = data + 0;
            float* py = data + 1;
            float* pz = data + 2;
            float* pr = data + 3;

            // load point cloud from file
            std::FILE* stream;
            stream = fopen(FilePath.c_str(), "rb");
            // stream = fopen("/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/02//velodyne/000043.png", "rb");
            // stream = fopen("/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/02//velodyne/000043.png", "rb");

            // Check if file was sucessfully opended
            if (!stream) {
                std::cerr << "Input error when loading Lidar scan: " << FilePath.c_str() << std::endl;
                point_cloud = cv::Mat::zeros(cv::Size(1, 4), CV_64F);
                return false;
            }

            // Save data to variable
            num = fread(data, sizeof(float), num, stream)/4;

            // Format data as desired
            // Mat point_cloud;
            point_cloud = cv::Mat::zeros(cv::Size(num, 4), CV_64F);
            for (int32_t i = 0; i < num; i++) {
                point_cloud.at<double>(0, i) = (double)*px;
                point_cloud.at<double>(1, i) = (double)*py;
                point_cloud.at<double>(2, i) = (double)*pz;
                point_cloud.at<double>(3, i) = (double)1;
                px+=4; py+=4; pz+=4; pr+=4;
            }

            // Close Stream and Free Memory
            fclose(stream);
            free(data);

            // Feedback and return
            // std::cout << "Read Pointcloud with number of points: " << num << std::endl;
            return true;
        }

        bool ReadKittiCalib (const std::string& FilePath, CalibMatrices& SequenceCalibration){
            // Reads a calib.txt file in the format of the Kitti Data and stores the projection matrices in a cv::Mat.
            std::ifstream CalibFile;

            // Open file
            CalibFile.open(FilePath);
            if (!CalibFile){
                throw std::runtime_error("Could not open Calibration file in path: " + FilePath);
            }

            // Go trough the file line by line
            int max_rows = 1000;
            int cnt = 0;
            while (!CalibFile.eof()){
                // Abort if more than max_rows were read
                if (cnt > max_rows){
                    break;
                }else{
                    cnt++;
                }

                // Get a line from calib file
                std::string s;
                getline(CalibFile, s);
                if (!s.empty()){
                    // Split Line into "words" split by a space
                    std::vector<std::string> line;
                    std::string word = "";
                    for (auto c : s){
                        if (c == ' '){
                            line.push_back(word);
                            word = "";
                        }else{
                            word = word + c;
                        }
                    }
                    line.push_back(word); // Last word in line

                    // Read which Matrix is defined in that line and obtain a pointer to that matrix
                    cv::Mat* CurrentMatrix;
                    int cols;
                    if (line.at(0).compare("P0:") == 0){
                        CurrentMatrix = &SequenceCalibration.P0;
                        *CurrentMatrix = cv::Mat::zeros(cv::Size(4, 3), CV_64F);
                        cols = 4;
                    }else if (line.at(0).compare("P1:") == 0){
                        CurrentMatrix = &SequenceCalibration.P1;
                        *CurrentMatrix = cv::Mat::zeros(cv::Size(4, 3), CV_64F);
                        cols = 4;
                    }else if (line.at(0).compare("P2:") == 0){
                        CurrentMatrix = &SequenceCalibration.P2;
                        *CurrentMatrix = cv::Mat::zeros(cv::Size(4, 3), CV_64F);
                        cols = 4;
                    }else if (line.at(0).compare("P3:") == 0){
                        CurrentMatrix = &SequenceCalibration.P3;
                        *CurrentMatrix = cv::Mat::zeros(cv::Size(4, 3), CV_64F);
                        cols = 4;
                    }else if (line.at(0).compare("Tr:") == 0){
                        CurrentMatrix = &SequenceCalibration.Tr;
                        *CurrentMatrix = cv::Mat::zeros(cv::Size(4, 4), CV_64F);
                        cols = 4;

                        //Tr e [3 x 4] -> add a "1" at (3,3) and fill rest of row with "0"
                        //to make it e [4 x 4] for multiplication with Pi Matrices
                        CurrentMatrix->at<double>(3, 0) = 0;
                        CurrentMatrix->at<double>(3, 1) = 0;
                        CurrentMatrix->at<double>(3, 2) = 0;
                        CurrentMatrix->at<double>(3, 3) = 1;
                    }else{
                        continue;
                    }

                    // Fill the Calibration Matrix referred to in that specific line
                    for (int i = 1; i < (int)line.size(); i++){
                        // Get indices
                        int col = (i-1) % cols;
                        int row = (i-1) / cols;
                        // Convert word to a double and save it in the correct location
                        CurrentMatrix->at<double>(row, col) = std::stod(line.at(i)); //stod converts string to double
                    }
                }
            }
            return true;
        }

        // ===========================================================================================================================
        // ===========================================================================================================================
        // PROJECTION
        // ===========================================================================================================================
        // ===========================================================================================================================

        cv::Mat CalcProjectionMatrix (const int& CameraId, const CalibMatrices& SequenceCalibration){
            cv::Mat ProjMat {};
            switch (CameraId){
                case 0:
                    ProjMat = SequenceCalibration.P0 * SequenceCalibration.Tr;
                    break;
                case 1:
                    ProjMat = SequenceCalibration.P1 * SequenceCalibration.Tr;
                    break;
                case 2:
                    ProjMat = SequenceCalibration.P2 * SequenceCalibration.Tr;
                    break;
                case 3:
                    ProjMat = SequenceCalibration.P3 * SequenceCalibration.Tr;
                    break;

                default:
                    std::cerr << "Invalid Camera Id for Projection Matrix calculation. Enter a value between 0 and 3." << std::endl;
                    break;
            }

            std::cout << ProjMat << std::endl;
            return ProjMat;
        }

        void ProjectPointcloudToImage (const cv::Mat& ProjMat, const cv::Mat& img, cv::Mat& PointCloud, cv::Mat& DepthImage,
            const float min_dist = 5, const float max_dist = 200, const int depthfactor = 256){
            // Projects a Lidar Pointcloud given as a vector of Points (x,y,z,r) into a image, given the Projection Matrix

            // Clear Depth Image
            DepthImage = cv::Mat::zeros(cv::Size(img.cols, img.rows), CV_16U);

            // Perform Projection as Matrix multiplication of Projection Matrix e [3 x 4] and Points e [4 x n]
            // to obtain a matrix of projected points e [3 x n]
            cv::Mat ProjectedPointcloud;
            ProjectedPointcloud = ProjMat * PointCloud;

            // Normalization to pixel coordinates
            ProjectedPointcloud.row(0) = ProjectedPointcloud.row(0).mul((1 / ProjectedPointcloud.row(2)));
            ProjectedPointcloud.row(1) = ProjectedPointcloud.row(1).mul((1 / ProjectedPointcloud.row(2)));

            // Fill depth image
            double *u,*v,*d;
            for (int32_t i = 0; i < ProjectedPointcloud.cols; i++){
                // Retrieve Pixel Coordinate
                u = &ProjectedPointcloud.at<double>(0, i);
                v = &ProjectedPointcloud.at<double>(1, i);
                d = &ProjectedPointcloud.at<double>(2, i);

                // Check if the point lies in the image
                if (*u > 0 && *v > 0 && *u < img.cols && *v < img.rows){
                    // Check if point lies in the specified boundaries for the distance
                    if (*d > min_dist && *d < max_dist){
                        // Set the pixel value at the given position
                        DepthImage.at<ushort>((int)*v, (int)*u) = (ushort)(*d * depthfactor);
                    }
                }
            }

            // Save Image to a file
            // cv::imwrite(SequencePath + "/image_" + std::to_string(camera_id) + "_Depth_Sparse/" + FrameName.str() + ".png", DepthImage);
        }

        // ===========================================================================================================================
        // ===========================================================================================================================
        // UPSAMPLING
        // ===========================================================================================================================
        // ===========================================================================================================================

        void NearestNeighbor (cv::Mat& DepthImage, const int max_radius){
            // // Upsamples a sparse image with neasrest neighbor within a nxn patch
            // DenseDepthMap = Mat::zeros(DepthImage.rows, DepthImage.cols, CV_16U);

            // Add padding of size "max_radius"
            cv::Mat PaddedImage;
            copyMakeBorder(DepthImage, PaddedImage, max_radius, max_radius, max_radius, max_radius, cv::BORDER_CONSTANT, 0);
            // std::cout << "y: " << PaddedImage.rows << " | x: " << PaddedImage.cols << std::endl;

            // Do distance transform to get the correct search radius for each pixel
            cv::Mat Labels;
            cv::Mat DistMap;
            DepthImage.convertTo(DistMap, CV_8U);
            cv::threshold(DistMap, DistMap, 0, 1, 1); // Inverted Thresh (0 if > 1); threshold(src, dst, threshold_value, max_binary_value, type)
            cv::distanceTransform(DistMap, DistMap, Labels, distType, maskSize);
            // DistMap.convertTo(DistMap, CV_16U);

            // Iterate through all pixels
            int searchradius = 0;
            double min, max;
            cv::Mat SearchBox;

            for (int u = 0; u < DepthImage.cols; u++){     //cols
                for (int v = 0; v < DepthImage.rows; v++){   //rows
                // Get Search Radius for the current pixel
                searchradius = (int)DistMap.at<float>(v, u);

                if (searchradius >= 0 && searchradius < max_radius){
                    searchradius++;
                    SearchBox = PaddedImage(cv::Rect(u+max_radius-searchradius, v+max_radius-searchradius, 2*searchradius, 2*searchradius)); //2*(searchradius+1), 2*(searchradius+1)));
                    cv::minMaxLoc(SearchBox, &min, &max);
                    DepthImage.at<ushort>(v, u) = (ushort)max;
                }
                }
            }
        }

        void FillPixels_nxn (cv::Mat& DepthMap, const int& PatchSize){
            // Takes sparse depth image and fills empty pixels by calculating the avg of nxn neighboring patch
            // Convolution with unity kernel (1 e [nxn]) and divide by number of non zero pixels in that patch

            // Check Datatype of incoming image
            if (DepthMap.type() != 2){
                std::cerr << "Could not run FillPixels_nxn as input 'DepthMap' was not given as CV_16U." << std::endl;
                return;
            }

            //Convolution Parameters
            cv::Point anchor = cv::Point(-1, -1);
            int delta = 0;
            int ddepth = -1;

            // Convolve scaled image with nxn kernel
            cv::Mat FilteredImage;
            DepthMap.convertTo(DepthMap, CV_32F);
            cv::Mat kernel = cv::Mat::ones(PatchSize, PatchSize, CV_32F) / (PatchSize*PatchSize);
            cv::filter2D(DepthMap, FilteredImage, ddepth , kernel, anchor, delta, cv::BORDER_DEFAULT);

            // Count number of Pixels in each patch
            cv::Mat PixelsPerPatch;
            cv::Mat UnitKernel = cv::Mat::ones(PatchSize, PatchSize, CV_16U);
            cv::threshold(DepthMap, PixelsPerPatch, 0, 1, 0); // Convert everything above 0 to 1
            cv::filter2D(PixelsPerPatch, PixelsPerPatch, ddepth, UnitKernel, anchor, delta, cv::BORDER_DEFAULT); // holds number of non zero pixels in patch
            // PixelsPerPatch.convertTo(PixelsPerPatch, CV_32F);

            // Normalize the Filtered image by number of valid pixels per patch
            DepthMap = FilteredImage.mul((PatchSize*PatchSize) / PixelsPerPatch);

            // Reset to CV_16U
            DepthMap.convertTo(DepthMap, CV_16U);
        }

        ushort GetNearestNeighborRespectingEdges(const cv::Mat &ROI, const cv::Mat &Edges, const int width){
            // Find non Zero pixels in ROI
            cv::Mat NonZero;
            std::vector<cv::Point> locations;
            ROI.convertTo(NonZero, CV_8U);
            cv::findNonZero(NonZero,  locations);

            int BestDistance = std::numeric_limits<int>::max();
            ushort BestDistanceValue = 0;

            for(auto &value: locations) {

                // std::cout << "=========" << std::endl;
                int dx = value.x - width;
                int dy = value.y - width;
                int sgnx = (dx>=0) ? 1 : -1;
                int sgny = (dy>=0) ? 1 : -1;
                bool validpath = true;
                uchar val;

                // Get Route to each of the points

                //First : go min(abs(dx), abs(dy)) steps diagonally in the respective directions
                int nstepsdiag;
                nstepsdiag = std::min(abs(dx), abs(dy));
                for (int i = 1; i <= nstepsdiag; i++){
                    val = Edges.at<uchar>(width+i*sgny, width+i*sgnx);
                    if (val > 0){   // Path crosses a edge
                        validpath = false;
                        break;
                    }
                }

                //Second, go residual steps in x/y direction
                int nstepshorvert;
                nstepshorvert = std::max(abs(dx), abs(dy)) - nstepsdiag;
                if ((abs(dx) > abs(dy)) & validpath){
                    // Go steps in x direction
                    for (int x = 1; x <= nstepshorvert; x++){
                        val = Edges.at<uchar>(width+(nstepsdiag)*sgny, width+(nstepsdiag+x)*sgnx);
                        if (val > 0){ // Path crosses a edge
                            validpath = false;
                            break;
                        }
                    }
                }else if (validpath){
                    // Go steps in y direction
                    for (int y = 1; y <= nstepshorvert; y++){
                        val = Edges.at<uchar>(width+(nstepsdiag+y)*sgny, width+(nstepsdiag)*sgnx);
                        if (val > 0){ // Path crosses a edge
                            validpath = false;
                            break;
                        }
                    }
                }

                // Evaluate the path
                int DistanceToTarget = (nstepsdiag + nstepshorvert);
                if (validpath & (DistanceToTarget < BestDistance)){ // TODO: or distance is equal but value is smaller
                    //If we arrive here, no edge was found on the way to the pixel and the current depth value is the closest to the pixel of interes
                    BestDistanceValue = ROI.at<ushort>(value.y, value.x);
                    BestDistance = DistanceToTarget;
                }
            }

            return BestDistanceValue;
        }

        void EdgePreservingNearestNeighbor(cv::Mat &DepthMap, cv::Mat &img, const int searchradius,
            const int Canny_BlurKernelSize = 8, const int Canny_LowThreshold = 10, const int Canny_LowToHighRatio = 3, const int Canny_CannyKernelSize = 3){
            // Ensure correct image formats
            // TODO: Handle Color image input
            if (DepthMap.type() != 2){
                DepthMap.convertTo(DepthMap, CV_16U);
            }

            // Run Canny Edge Detection
            cv::Mat CannyEdge;
            img.convertTo(CannyEdge, CV_8U);
            cv::blur(CannyEdge, CannyEdge, cv::Size(Canny_BlurKernelSize,Canny_BlurKernelSize));
            cv::Canny(CannyEdge, CannyEdge, Canny_LowThreshold, Canny_LowThreshold*Canny_LowToHighRatio, Canny_CannyKernelSize);

            // Add Padding to Depthmap and Canny Image
            cv::Mat DepthMap_Padded, CannyEdge_Padded;
            copyMakeBorder(DepthMap, DepthMap_Padded, searchradius, searchradius, searchradius, searchradius, cv::BORDER_CONSTANT, 0);
            copyMakeBorder(CannyEdge, CannyEdge_Padded, searchradius, searchradius, searchradius, searchradius, cv::BORDER_CONSTANT, 0);

            // Reserve Variables for Depth Upsamling
            cv::Mat ROI_Depth, ROI_Canny, DepthDense;
            ushort depthval;

            for (int u = 0; u < DepthMap.cols; u++){
                for (int v = 0; v < DepthMap.rows; v++){
                    // std::cout << v + width << " - " << u + width << std::endl;
                    depthval = DepthMap.at<ushort>(v, u);
                    if (depthval == 0){
                        //Define ROI
                        ROI_Depth = DepthMap_Padded(cv::Rect(u, v, 2*searchradius + 1, 2*searchradius + 1));   // Center of these Rect's are the point of interest in the original image
                        ROI_Canny = CannyEdge_Padded(cv::Rect(u, v, 2*searchradius + 1, 2*searchradius + 1));

                        //TODO: Check if ROI is empty or pixel is sampled
                        DepthMap.at<ushort>(v, u) = GetNearestNeighborRespectingEdges(ROI_Depth, ROI_Canny, searchradius);
                    }
                }
            }
        }

        void FillUpperRegion (cv::Mat& RawImage){
            // Get List of non zero Pixels
            std::vector<cv::Point> idx_n0;
            //cv::Mat idx_n0;
            cv::Mat Img_CV8UC1;
            RawImage.convertTo(Img_CV8UC1, CV_8U);      //Conversion required for findNonZero function
            cv::findNonZero(Img_CV8UC1,  idx_n0);

            // For each column (x) in Image find upper row (min y) with a value
            std::vector<int> UpperPixelYs(RawImage.cols, RawImage.rows-1);
            std::vector<int> UpperPixelValues(RawImage.cols, 0);
            //int PixelValue;
            cv::Point pnt;

            for (auto const& pixel: idx_n0){
                if (UpperPixelYs.at(pixel.x) > pixel.y){
                    UpperPixelYs.at(pixel.x) = pixel.y;
                    UpperPixelValues.at(pixel.x) = (int)RawImage.at<ushort>(pixel.y, pixel.x);
                    //UpperPixelValues.at(pixel.x) = RawImage.row(pixel.y).col(pixel.x);
                }
            }

            //Fill upper region of image
            cv::Mat roi;
            for (int x = 0; x < RawImage.cols; x++){
                if (UpperPixelValues[x] > 0){
                    roi = RawImage.col(x).rowRange(0, UpperPixelYs[x]+1);
                    roi.setTo(cv::Scalar(UpperPixelValues[x]));
                }
            }
        }

        void interpolation_cols (cv::Mat& DepthMapToProcess){
            // Variables

            // Col of interest
            cv::Mat Col;

            // First and last valid LiDAR points in a column
            ushort first_y, last_y;

            // Iterator
            int row_idx;

            // Vector holding the y/v coordinates of valid LiDAR points in a column
            std::vector<int> idx_n0_y;

            // Vector holding the Points (u,v) on non zero values
            std::vector<cv::Point> idx_n0;

            // Interpolation variables
            ushort y1, y2;
            // Interpolation variables
            int x1, x2;

            //Convert Image to CV8UC1, required for findNonZero
            cv::Mat DepthMap_CV8U;
            DepthMapToProcess.convertTo(DepthMap_CV8U, CV_8U);



            for (int x = 0; x<DepthMapToProcess.cols; x++){
                // Extract the current coloumn of interes
                Col = DepthMap_CV8U.col(x);

                // Get Non Zero Values
                idx_n0.clear();                             //Holds indices of all non zero elements of Row
                cv::findNonZero(Col,  idx_n0);

                // Extract the y coordinates of non zero pixels
                idx_n0_y.clear();
                for (auto const& value: idx_n0){
                    idx_n0_y.push_back(value.y);
                }

                row_idx = 0;
                if (!idx_n0_y.empty()){
                    // Extract first and last valid value in the column
                    first_y = DepthMapToProcess.at<ushort>(idx_n0_y.front(), x);
                    last_y = DepthMapToProcess.at<ushort>(idx_n0_y.back(), x);

                    // Iterate through all indices in the row to set the value
                    for (int idx = 0; idx < Col.rows; idx++){
                        if (idx < idx_n0_y.front()){
                            // If below lowest row with LiDAR point, take the value of the lowest row
                            DepthMapToProcess.at<ushort>(idx, x) = first_y;
                        }else if (idx > idx_n0_y.back()){
                            // If above highest row with LiDAR point, take the value of the highest row
                            DepthMapToProcess.at<ushort>(idx, x) = last_y;
                        }else if (idx == idx_n0_y[row_idx]){
                            // If it's a valid pixel, increase counter pointing to the currently valid values
                            row_idx++;
                        }else{
                            // Between to valid points --> Do linear interpolation
                            // Linear Interpolation
                            // y = y1 + (x - x1) ((y2 - y1) / (x2 - x1))
                            y1 = DepthMapToProcess.at<ushort>(idx_n0_y[row_idx - 1], x);
                            y2 = DepthMapToProcess.at<ushort>(idx_n0_y[row_idx], x);
                            x1 = idx_n0_y[row_idx - 1];
                            x2 = idx_n0_y[row_idx];
                            DepthMapToProcess.at<ushort>(idx, x) = (y1 + (idx - x1) * ((y2 - y1) / (x2 - x1)));
                        }
                    }
                }
            }
        }

        void interpolation_rows (cv::Mat& DepthMapToProcess){
            // Variables

            // Row of interest
            cv::Mat Row;

            // First and last valid LiDAR points in a column
            ushort first_x, last_x;

            // Iterator
            int col_idx;

            // Vector holding the x/u coordinates of valid LiDAR points in a row
            std::vector<int> idx_n0_x;

            // Vector holding the Points (u,v) on non zero values
            std::vector<cv::Point> idx_n0;

            // Interpolation variables
            ushort y1, y2;
            // Interpolation variables
            int x1, x2;

            //Convert Image to CV8UC1, required for findNonZero
            cv::Mat DepthMap_CV8U;
            DepthMapToProcess.convertTo(DepthMap_CV8U, CV_8U);

            for (int y = 0; y<DepthMapToProcess.rows; y++){
                // Extract the current coloumn of interes
                Row = DepthMap_CV8U.row(y);

                // Get Non Zero Values
                idx_n0.clear();                             //Holds indices of all non zero elements of Row
                cv::findNonZero(Row,  idx_n0);

                // Extract the y coordinates of non zero pixels
                idx_n0_x.clear();
                for (auto const& value: idx_n0){
                    idx_n0_x.push_back(value.x);
                }

                col_idx = 0;
                if (!idx_n0_x.empty()){ // at least one valid pixel in the row
                    // Extract first and last valid value in the row
                    first_x = DepthMapToProcess.at<ushort>(y, idx_n0_x.front());
                    last_x = DepthMapToProcess.at<ushort>(y, idx_n0_x.back());

                    // Iterate through all indices in the Column to set the value
                    for (int idx = 0; idx < Row.cols; idx++){
                        if (idx < idx_n0_x.front()){
                            // If below lowest row with LiDAR point, take the value of the lowest row
                            DepthMapToProcess.at<ushort>(y, idx) = first_x;
                        }else if (idx > idx_n0_x.back()){
                            // If above highest row with LiDAR point, take the value of the highest row
                            DepthMapToProcess.at<ushort>(y, idx) = last_x;
                        }else if (idx == idx_n0_x[col_idx]){
                            // If it's a valid pixel, increase counter pointing to the currently valid values
                            col_idx++;
                        }else{
                            // Between to valid points --> Do linear interpolation
                            // Linear Interpolation
                            // y = y1 + (x - x1) ((y2 - y1) / (x2 - x1))
                            // y = y1 + (x - x1) ((y2 - y1) / (x2 - x1))
                            y1 = DepthMapToProcess.at<ushort>(y, idx_n0_x[col_idx - 1]);
                            y2 = DepthMapToProcess.at<ushort>(y, idx_n0_x[col_idx]);
                            x1 = idx_n0_x[col_idx - 1];
                            x2 = idx_n0_x[col_idx];
                            DepthMapToProcess.at<ushort>(y, idx) = (y1 + (idx - x1) * ((y2 - y1) / (x2 - x1)));
                        }

                    // std::cout << idx << " of " << Row.cols << std::endl;
                    }
                }
            }
        }

        void DialateDepthmap(cv::Mat& DepthMapToProcess, const std::string KernelType, const int KernelSize_u, const int KernelSize_v){
            cv::Mat kernel;
            // Initiate Kernel
            // Kernels: https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#gac2db39b56866583a95a5680313c314ad
            if (KernelType == "Rectangle"){
                kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(KernelSize_u,KernelSize_v));
            }else if (KernelType == "Cross"){
                kernel = getStructuringElement(cv::MORPH_CROSS, cv::Size(KernelSize_u,KernelSize_v));
            }else if (KernelType == "Ellipse"){
                kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(KernelSize_u,KernelSize_v));
            }else if (KernelType == "Diamond"){
                // Holds data to initialize kernel for Diamon dilation
                if (KernelSize_u == 3){
                    kernel = cv::Mat(cv::Size(KernelSize_u,KernelSize_u), CV_8U, DiamondKernelData_3);
                }else if(KernelSize_u == 5){
                    kernel = cv::Mat(cv::Size(KernelSize_u,KernelSize_u), CV_8U, DiamondKernelData_5);
                }else if (KernelSize_u == 7){
                    kernel = cv::Mat(cv::Size(KernelSize_u,KernelSize_u), CV_8U, DiamondKernelData_7);
                }else if (KernelSize_u == 9){
                    kernel = cv::Mat(cv::Size(KernelSize_u,KernelSize_u), CV_8U, DiamondKernelData_9);
                }else{
                    std::cerr << "Could not perform dilation, invalid kernel size for diamond kernel." << std::endl;
                    return;
                }

                if (KernelSize_u != KernelSize_v){
                    std::cout << "Warning: KernelSize_u != KernelSize_v for diamond dilation. Only u is considered." << std::endl;
                }
            }else{
                std::cerr << "Could not perform dilation, invalid kernel Type." << std::endl;
                return;
            }

            //Dialation
            cv::dilate(DepthMapToProcess, DepthMapToProcess, kernel, cv::Point(-1,-1), 1);
        }

        void DialateDepthmapInverted(cv::Mat& DepthMapToProcess, const std::string KernelType, const int KernelSize_u, const int KernelSize_v, const float max_depth = 100, const float depth_factor = 256){
            cv::Mat kernel;
            // Initiate Kernel
            // Kernels: https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#gac2db39b56866583a95a5680313c314ad
            if (KernelType == "Rectangle"){
                kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(KernelSize_u,KernelSize_v));
            }else if (KernelType == "Cross"){
                kernel = getStructuringElement(cv::MORPH_CROSS, cv::Size(KernelSize_u,KernelSize_v));
            }else if (KernelType == "Ellipse"){
                kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(KernelSize_u,KernelSize_v));
            }else if (KernelType == "Diamond"){
                // Holds data to initialize kernel for Diamon dilation
                if (KernelSize_u == 3){
                    kernel = cv::Mat(cv::Size(KernelSize_u,KernelSize_u), CV_8U, DiamondKernelData_3);
                }else if(KernelSize_u == 5){
                    kernel = cv::Mat(cv::Size(KernelSize_u,KernelSize_u), CV_8U, DiamondKernelData_5);
                }else if (KernelSize_u == 7){
                    kernel = cv::Mat(cv::Size(KernelSize_u,KernelSize_u), CV_8U, DiamondKernelData_7);
                }else if (KernelSize_u == 9){
                    kernel = cv::Mat(cv::Size(KernelSize_u,KernelSize_u), CV_8U, DiamondKernelData_9);
                }else{
                    std::cerr << "Could not perform dilation, invalid kernel size for diamond kernel." << std::endl;
                    return;
                }

                if (KernelSize_u != KernelSize_v){
                    std::cout << "Warning: KernelSize_u != KernelSize_v for diamond dilation. Only u is considered." << std::endl;
                }
            }else{
                std::cerr << "Could not perform dilation, invalid kernel Type." << std::endl;
                return;
            }

            // Invert Depth Data
            DepthMapToProcess = (max_depth * depth_factor) - DepthMapToProcess;
            cv::threshold(DepthMapToProcess, DepthMapToProcess, (max_depth * depth_factor)-1, 0, cv::THRESH_TOZERO_INV);

            //Dialation
            cv::dilate(DepthMapToProcess, DepthMapToProcess, kernel, cv::Point(-1,-1), 1);

            // Reverse Inversion
            DepthMapToProcess = (max_depth * depth_factor) - DepthMapToProcess;
            cv::threshold(DepthMapToProcess, DepthMapToProcess, (max_depth * depth_factor)-1, 0, cv::THRESH_TOZERO_INV);
        }

        void ExtrapolateColsUpwards(cv::Mat& DepthmapToProcess){
            ushort value = 0;

            for(int u = 0; u < DepthmapToProcess.cols; u++){
                value = 0;
                for (int v = DepthmapToProcess.rows; v > 0; v--){
                    if (DepthmapToProcess.at<ushort>(v,u) > 0){
                        value = DepthmapToProcess.at<ushort>(v,u);
                    }else{
                        DepthmapToProcess.at<ushort>(v,u) = value;
                    }
                }
            }
        }

        void AverageBlur (cv::Mat& DepthmapToProcess, const int KernelSize){
            // Apply Gaussian Blur
            blur(DepthmapToProcess, DepthmapToProcess, cv::Size(KernelSize,KernelSize));
        }


        void BilateralFilter (cv::Mat& DepthmapToProcess, const int KernelSize, const float SigmaColor, const float SigmaSpace){
            // Apply Bilateral Filter
            cv::Mat DepthMap_32F, FilterResult;
            DepthmapToProcess.convertTo(DepthMap_32F, CV_32F);
            cv::bilateralFilter(DepthMap_32F, FilterResult, KernelSize, SigmaColor, SigmaSpace);
            FilterResult.convertTo(DepthmapToProcess, CV_16U);
        }

        void IPBasic(cv::Mat& SparseDepthMap, const float max_depth = 100, const float depth_factor = 256, const std::string CustomKernel = "Diamond", const int CustomKernelSize = 5, const bool extrapolate = false, std::string BlurType = "Bilateral"){
            // Invert Depth Data
            SparseDepthMap = (max_depth * depth_factor) - SparseDepthMap;
            cv::threshold(SparseDepthMap, SparseDepthMap, (max_depth * depth_factor)-1, 0, cv::THRESH_TOZERO_INV);

            // Initial Dilation
            DialateDepthmap(SparseDepthMap, CustomKernel, CustomKernelSize, CustomKernelSize);

            // Hole Close
            cv::morphologyEx(SparseDepthMap, SparseDepthMap, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)));

            // Fill empty spaces with dilated values
            cv::Mat Dilated7;
            cv::dilate(SparseDepthMap, Dilated7, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7)), cv::Point(-1,-1), 1);
            cv::Mat Mask;
            cv::threshold(SparseDepthMap, Mask, 1, 1, cv::THRESH_BINARY_INV); // Mask is 1 if pixel is empty and 0 otherwise
            Mask.convertTo(Mask, CV_8U);
            Dilated7.copyTo(SparseDepthMap, Mask);

            // Extrapolate to top of frame and large hole fill
            if (extrapolate){
                // Extrapolate to upper region
                FillUpperRegion (SparseDepthMap);
                // ExtrapolateColsUpwards (SparseDepthMap);

                // Large Hole Fill
                cv::Mat DilatedLarge;
                cv::dilate(SparseDepthMap, DilatedLarge, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(31,31)), cv::Point(-1,-1), 1);
                cv::threshold(SparseDepthMap, Mask, 1, 1, cv::THRESH_BINARY_INV);   // Mask is 1 if pixel is empty and 0 otherwise
                Mask.convertTo(Mask, CV_8U);
                DilatedLarge.copyTo(SparseDepthMap, Mask);
            }

            // Median Blur
            cv::medianBlur(SparseDepthMap, SparseDepthMap, 5);

            if (BlurType == "Bilateral"){
                BilateralFilter (SparseDepthMap, 5, 1.5, 2);
            }else if (BlurType == "Gaussian"){
                cv::Mat GaussianBlured;
                cv::GaussianBlur(SparseDepthMap, GaussianBlured, cv::Size(5,5), 0);
                cv::threshold(SparseDepthMap, Mask, 1, 1, cv::THRESH_BINARY);   // Mask is 1 if pixel is NOT empty and 0 otherwise
                Mask.convertTo(Mask, CV_8U);
                GaussianBlured.copyTo(SparseDepthMap, Mask);
            }else{
                std::cerr << "Could not perform Filtering, please enter a valid filter option: Bilateral or Gaussian." << std::endl;
            }

            // Reverse Inversion
            SparseDepthMap = (max_depth * depth_factor) - SparseDepthMap;
            cv::threshold(SparseDepthMap, SparseDepthMap, (max_depth * depth_factor)-1, 0, cv::THRESH_TOZERO_INV);
        }



        // ===========================================================================================================================
        // ===========================================================================================================================
        // HELPERS
        // ===========================================================================================================================
        // ===========================================================================================================================

        void SaveDepthmapToFile (cv::Mat &img, std::string FolderName){
            // Save Image to a file
            std::string OutName;
            bool sucess;
            OutName = SequencePath + "/" + FolderName + "/" + FrameName.str() + ".png";
            sucess = cv::imwrite(OutName, img);

            // Feedback on saving the image
            if (!sucess){
                std::cerr << "Could not save Image on path " << OutName << std::endl;
            }
        }

        static cv::Mat ConvertToHeatmap(const cv::Mat &src, int maxVal){
            cv::Mat srcTrueDepth8U;
            src.convertTo(srcTrueDepth8U, CV_8U, 1.0/maxVal * 255.0);

            cv::Mat heatmap;
            applyColorMap(srcTrueDepth8U, heatmap, cv::COLORMAP_HSV);
            // heatmap.convertTo(heatmap, CV_16UC3, 65535/255);

            return heatmap;
        }

        void ImageDepthmapOverlay(cv::Mat &img, cv::Mat &Depthmap, cv::Mat &Overlay, const int maxVal, const int CameraId){
            // Create an overlay image of a camera image and the depthmap created for it.
            // MaxVal is required for the scaling of the colormap and should be max_dist * depthfactor from the projection

            // // Transform images to same format
            // cv::Mat img_CV_16UC3, Depthmap_CV_16UC3;
            cv::Mat img_C3;
            img.convertTo(img_C3, CV_8U);
            cvtColor(img_C3, img_C3, cv::COLOR_GRAY2BGR);

            // Transform Depthmap to Colormap
            Overlay = ConvertToHeatmap(Depthmap, maxVal);

            // Perform Overlay
            float alpha, beta;
            alpha = 0.75;
            beta = (1-alpha);
            addWeighted(img_C3, alpha, Overlay, beta, 0.0, Overlay);

            // Save Image
            std::string FolderName;
            if (optDataset == "Odometry"){
                FolderName = "/image_" + std::to_string(CameraId) + "_Depth_Overlay";
            } else if (optDataset == "Completion"){
                FolderName = "/image_" + std::to_string(CameraId) + "_Depth_Overlay";
            } else if (optDataset == "Virtual"){
                FolderName = "/image_0" + std::to_string(CameraId) + "_Depth_Overlay";
            }
            // std::cout << FolderName << std::endl;
            SaveDepthmapToFile (Overlay, FolderName);            // Feedback on saving the image
        }



        // ===============================================
        // Variables
        std::string SequencePath;
        double Frequency;
        int32_t FrameCounter = 0;
        rclcpp::TimerBase::SharedPtr timer;

        // Subscriber & Publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_0;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_1;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_2;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_3;

        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depthmap_0;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depthmap_1;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depthmap_2;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depthmap_3;

        // Image Message
        sensor_msgs::msg::Image msg_image_0;
        sensor_msgs::msg::Image msg_image_1;
        sensor_msgs::msg::Image msg_image_2;
        sensor_msgs::msg::Image msg_image_3;

        sensor_msgs::msg::Image msg_depthmap_0;
        sensor_msgs::msg::Image msg_depthmap_1;
        sensor_msgs::msg::Image msg_depthmap_2;
        sensor_msgs::msg::Image msg_depthmap_3;

        // // Point Cloud Message
        // sensor_msgs::msg::PointCloud2 msg_pcd;

        // Image Variables
        cv::Mat img_0, img_1, img_2, img_3;
        cv::Mat depthmap_0, depthmap_1, depthmap_2, depthmap_3;
        cv::Mat Overlay_0, Overlay_1, Overlay_2, Overlay_3;

        // Pointcloud Variable
        cv::Mat pcd;

        // Nearest Neighbor Variables
        int maskSize = cv::DIST_MASK_5;
        int distType = cv::DIST_L2;

        // Other Variables
        CalibMatrices SequenceCalibration;
        cv::Mat ProjMat0, ProjMat1, ProjMat2, ProjMat3;
        std::stringstream FrameName;
        bool SaveDenseDepth, SaveSparseDepth, CreateOverlay;
        std::string optDataset;
        int OverlayGain = 256*50;

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

        // Timing
        int FrameCounterTime;
        float AvgTime;

        // For Depth Completion
        std::vector<double> vTimestamps;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KittiPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
