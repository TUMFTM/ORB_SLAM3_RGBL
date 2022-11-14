//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include<fstream>
#include<iomanip>

/*
This is the template for a depth map manipulator ros2 node.
It contains a subscriber to an Image Topic (sub), a function where the desired image manipulations can be
performed and finally another publisher that distributes the processed depth map.
*/

class DepthMapSubscriberNode : public rclcpp::Node{
    public:
        DepthMapSubscriberNode() : Node("depth_map_subsriver"){
            // Subscriber
            // sub = this->create_subscription<sensor_msgs::msg::Image>("depthmap/sparse/left", 10, std::bind(&DepthMapSubscriberNode::callbackRecievedDepthMap, this, std::placeholders::_1));
            sub = this->create_subscription<sensor_msgs::msg::Image>("depthmap_left", 10, std::bind(&DepthMapSubscriberNode::callbackRecievedDepthMap, this, std::placeholders::_1));
            
            // Publisher
            pub = this->create_publisher<sensor_msgs::msg::Image>("depthmap/dense/left", 10);
            
            // Terminal Info
            RCLCPP_INFO(this->get_logger(), "Depth Map Subscriber & Publisher have been started.");
        }

    private:
        // ===============================================
        // Subscriber and Publisher; Processing Call
        void callbackRecievedDepthMap(const sensor_msgs::msg::Image::SharedPtr msg){
            // Recieve Depth Map
            try{
                // Ptr_SparseDepthMap_Left = cv_bridge::toCvCopy(msg);
                // SparseDepthMapLeft = Ptr_SparseDepthMap_Left->image;
                SparseDepthMapLeft = cv_bridge::toCvCopy(msg)->image;
            }
            catch (cv_bridge::Exception& e){
                RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Recieved new depth map with shape [%d x %d]", msg->width, msg->height);

            // Process the Depth Map as desired
            ProcessDepthMap(SparseDepthMapLeft);
        }

        void ProcessDepthMap (cv::Mat DepthMapToProcess){
            // Call Manipulating Functions
            NearestNeighbor (DepthMapToProcess, 7);
            // Call Publisher
            publish_dense_depthmap(DepthMapToProcess);
        }

        void publish_dense_depthmap(const cv::Mat DepthMapToPublish){
            // Fill message
            convert_frame_to_message(DepthMapToPublish, msg);
            //Publish Message
            pub->publish(msg);
            //Terminal Info
            RCLCPP_INFO(this->get_logger(), "Published new dense depth map with shape [%d x %d]", msg.width, msg.height);
        }

        // ===============================================
        // Manipulations
        void NearestNeighbor (cv::Mat& SparseDepthImage, const int max_radius){

            // // Upsamples a sparse image with neasrest neighbor within a nxn patch
            // DenseDepthMap = Mat::zeros(SparseDepthImage.rows, SparseDepthImage.cols, CV_16U);

            // Add padding of size "max_radius"
            cv::Mat PaddedImage;
            copyMakeBorder(SparseDepthImage, PaddedImage, max_radius, max_radius, max_radius, max_radius, cv::BORDER_CONSTANT, 0);
            // std::cout << "y: " << PaddedImage.rows << " | x: " << PaddedImage.cols << std::endl;

            // Do distance transform to get the correct search radius for each pixel
            cv::Mat Labels;
            cv::Mat DistMap;
            SparseDepthImage.convertTo(DistMap, CV_8U);
            cv::threshold(DistMap, DistMap, 0, 1, 1); // Inverted Thresh (0 if > 1); threshold(src, dst, threshold_value, max_binary_value, type)
            cv::distanceTransform(DistMap, DistMap, Labels, distType, maskSize);
            // DistMap.convertTo(DistMap, CV_16U);

            // Iterate through all pixels
            int searchradius = 0;
            double min, max;
            cv::Mat SearchBox;

            for (int u = 0; u < SparseDepthImage.cols; u++){     //cols
                for (int v = 0; v < SparseDepthImage.rows; v++){   //rows
                // Get Search Radius for the current pixel
                searchradius = (int)DistMap.at<float>(v, u);

                if (searchradius > 0 && searchradius < max_radius){
                    searchradius++;
                    SearchBox = PaddedImage(cv::Rect(u+max_radius-searchradius, v+max_radius-searchradius, 2*searchradius, 2*searchradius)); //2*(searchradius+1), 2*(searchradius+1)));
                    cv::minMaxLoc(SearchBox, &min, &max);
                    SparseDepthImage.at<ushort>(v, u) = (ushort)max;
                }
                }
            }
            
            // Save Image to a file
            std::stringstream ss;
            ss << std::setfill('0') << std::setw(6) << counter << ".png";
            //std::cout << ss.str() << std::endl;
            imwrite(ss.str(), SparseDepthImage);
            counter++;
        }

        // ===============================================
        // Helpers

        void convert_frame_to_message(
            const cv::Mat & frame, sensor_msgs::msg::Image & msg)
        {
            // copy cv information into ros message
            msg.height = frame.rows;
            msg.width = frame.cols;
            msg.encoding = "mono16"; //mat_type2encoding(frame.type());
            msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            size_t size = frame.step * frame.rows;
            msg.data.resize(size);
            memcpy(&msg.data[0], frame.data, size);
            msg.header.frame_id = "map";
            msg.header.stamp = this->now();
        }

        // ===============================================
        // Variables

        // Subscriber & Publisher
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;   // Image Subscriber
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;      // Image Publisher

        // Image Message 
        sensor_msgs::msg::Image msg;
        
        // Image Variables
        cv::Mat SparseDepthMapLeft;
        cv::Mat DenseDepthMapLeft;

        // Nearest Neighbor Variables
        int maskSize = cv::DIST_MASK_5;
        int distType = cv::DIST_L2;

        // Other 
        int counter = 0;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthMapSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// ========================================================
// Archive

// void ExtractMaxMinFromCVMatrix (cv::Mat DepthMapToProcess){
//     using namespace cv;
//     using namespace std;

//     double minVal; 
//     double maxVal; 
//     Point minLoc; 
//     Point maxLoc;

//     minMaxLoc(DepthMapToProcess, &minVal, &maxVal, &minLoc, &maxLoc);

//     cout << "min val: " << minVal << endl;
//     cout << "max val: " << maxVal << endl;
// }