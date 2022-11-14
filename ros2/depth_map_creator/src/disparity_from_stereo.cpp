//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

/*
This is the template for a depth map manipulator ros2 node.
It contains a subscriber to an Image Topic (sub), a function where the desired image manipulations can be
performed and finally another publisher that distributes the processed depth map.
*/


class DepthMapSubscriberNode : public rclcpp::Node{
    public:
        DepthMapSubscriberNode() : Node("depth_map_subsriver"){
            // Subscriber
            subLeft = this->create_subscription<sensor_msgs::msg::Image>("camera/left", 10, std::bind(&DepthMapSubscriberNode::callback_RecievedLeftImage, this, std::placeholders::_1));
            subRight = this->create_subscription<sensor_msgs::msg::Image>("camera/right", 10, std::bind(&DepthMapSubscriberNode::callback_RecievedRightImage, this, std::placeholders::_1));
            
            // Publisher
            pub = this->create_publisher<sensor_msgs::msg::Image>("camera/disparity", 10);
            
            // Terminal Info
            RCLCPP_INFO(this->get_logger(), "Stereo Image Subscribers & Disparity Publisher have been started.");

            // Set Parameters of SBM
            // sbm->setDisp12MaxDiff(1);
            // sbm->setSpeckleRange(8);
            // sbm->setSpeckleWindowSize(9);
            // sbm->setUniquenessRatio(3);
            // sbm->setTextureThreshold(300);
            sbm->setMinDisparity(0);
            sbm->setPreFilterCap(63);
            // sbm->setPreFilterSize(25);
        }

    private:
        // ===============================================
        // Subscriber and Publisher; Processing Call
        void callback_RecievedLeftImage(const sensor_msgs::msg::Image::SharedPtr msg){
            // Recieve Depth Map
            try{
                LeftImage = cv_bridge::toCvCopy(msg)->image;
                cv::cvtColor(LeftImage, LeftImage, cv::COLOR_RGB2GRAY);
                LeftImage.convertTo(LeftImage_8UC1, CV_8U);
                //std::cout << "Left Type: " << LeftImage_8UC1.type() << std::endl;
            }
            catch (cv_bridge::Exception& e){
                RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Recieved new left image with shape [%d x %d]", msg->width, msg->height);
        }
        void callback_RecievedRightImage(const sensor_msgs::msg::Image::SharedPtr msg){
            // Recieve Depth Map
            try{
                RightImage = cv_bridge::toCvCopy(msg)->image;
                cv::cvtColor(RightImage, RightImage, cv::COLOR_RGB2GRAY);
                RightImage.convertTo(RightImage_8UC1, CV_8U);
                //std::cout << "Rigth Type: " << RightImage_8UC1.type() << std::endl;
            }
            catch (cv_bridge::Exception& e){
                RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Recieved new right image with shape [%d x %d]", msg->width, msg->height);

            ProcessDisparity(LeftImage_8UC1, RightImage_8UC1);
        }

        void ProcessDisparity (const cv::Mat LeftImg, const cv::Mat RightImg){
            // Calc Disparity
            sbm->compute(LeftImg, RightImg, DisparityMapToPublish);
            std::cout << "Rows: " << DisparityMapToPublish.rows << " / Cols: " << DisparityMapToPublish.cols << std::endl;
            // Call Publisher
            publish_disparity_map(DisparityMapToPublish);
        }

        void publish_disparity_map(const cv::Mat DisparityMapToPublish){
            // Fill message
            convert_frame_to_message(DisparityMapToPublish, msg);
            //Publish Message
            pub->publish(msg);
            //Terminal Info
            RCLCPP_INFO(this->get_logger(), "Published new disparity map with shape [%d x %d]", msg.width, msg.height);
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
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subLeft;       // Image Subscriber
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subRight;      // Image Subscriber
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;              // Image Publisher

        // Image Message (Out)
        sensor_msgs::msg::Image msg;
        
        // Image Variables
        cv::Mat LeftImage;
        cv::Mat RightImage;
        cv::Mat LeftImage_8UC1;
        cv::Mat RightImage_8UC1;
        cv::Mat DisparityMap;
        cv::Mat DisparityMapToPublish;

        // // Other
        // // Initialize Stereo
        cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(160, 5);

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