//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

/*
Canny Edge Detector
*/

class DepthMapSubscriberNode : public rclcpp::Node{
    public:
        DepthMapSubscriberNode() : Node("depth_map_subsriver"){
            // Subscriber
            sub = this->create_subscription<sensor_msgs::msg::Image>("camera/left", 10, std::bind(&DepthMapSubscriberNode::CallbackRecievedImage, this, std::placeholders::_1));
            
            // Publisher
            pub = this->create_publisher<sensor_msgs::msg::Image>("camera/canny/left", 10);
            
            // Terminal Info
            RCLCPP_INFO(this->get_logger(), "Image Subscriber & Canny Publisher have been started.");
        }

    private:
        // ===============================================
        // Subscriber and Publisher; Processing Call
        void CallbackRecievedImage(const sensor_msgs::msg::Image::SharedPtr msg){
            // Recieve Image
            try{
                RawImg = cv_bridge::toCvCopy(msg)->image;
                // std::cout << "Image Type before: " << RawImg.type() << std::endl;
                cvtColor(RawImg, RawImg, cv::COLOR_BGR2GRAY );
                RawImg.convertTo(RawImg, CV_8U);
                // std::cout << "Image Type after: " << RawImg.type() << std::endl;
            }
            catch (cv_bridge::Exception& e){
                RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Recieved new image with shape [%d x %d]", msg->width, msg->height);

            // Process the Depth Map as desired
            ProcessImage(RawImg);
        }

        void ProcessImage (cv::Mat& ImageToProcess){
            // Call Manipulating Functions
            CannyDetector(ImageToProcess);
            // Call Publisher
            //PublishCannyImage(CannyImg);
            PublishCannyImage(dst);
        }

        void PublishCannyImage(const cv::Mat& ImageToPublish){
            // Fill message
            convert_frame_to_message(ImageToPublish, msg);
            //Publish Message
            pub->publish(msg);
            //Terminal Info
            RCLCPP_INFO(this->get_logger(), "Published new canny edge image map with shape [%d x %d]", msg.width, msg.height);
        }

        // ===============================================
        // Manipulations
        void CannyDetector(const cv::Mat& ImageToProcess){
            // 1. Blurring
            cv::blur(ImageToProcess, BlurredImage, cv::Size(Param_Canny_BlurKernelSize, Param_Canny_BlurKernelSize));
            // 2. Edge Detection and Thresholding
            cv::Canny(BlurredImage, CannyImg, Param_Canny_lowThreshold, Param_Canny_hiThreshold, Param_Canny_kernel_size);
            // 3. Convert Edges to 1 everything else to 0
            //cv::threshold(CannyImg, CannyImg, 0, 1, 0); // Convert everything above 0 to 1
            
            dst.create(ImageToProcess.size(), ImageToProcess.type());
            dst = cv::Scalar::all(0);
            ImageToProcess.copyTo(dst, CannyImg);
        }

        // ===============================================
        // Helpers

        void convert_frame_to_message(const cv::Mat & frame, sensor_msgs::msg::Image & msg){
            // copy cv information into ros message
            msg.height = frame.rows;
            msg.width = frame.cols;
            msg.encoding = "mono16";
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
        cv::Mat RawImg;
        cv::Mat BlurredImage;
        cv::Mat CannyImg;
        cv::Mat dst;

        // Canny Parameters
        int Param_Canny_BlurKernelSize = 3;
        int Param_Canny_lowThreshold = 5;
        int Param_Canny_hiThreshold = 10;
        int Param_Canny_kernel_size = 3;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthMapSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
