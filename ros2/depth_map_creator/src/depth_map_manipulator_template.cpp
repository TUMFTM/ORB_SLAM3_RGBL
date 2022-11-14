//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

/*
This is the template for a depth map manipulator ros2 node.
It contains a subscriber to an Image Topic (sub), a function where the desired image manipulations can be
performed and finally another publisher that distributes the processed depth map.
*/

class DepthMapSubscriberNode : public rclcpp::Node{
    public:
        DepthMapSubscriberNode() : Node("depth_map_subsriver"){
            // Subscriber
            sub = this->create_subscription<sensor_msgs::msg::Image>("depthmap/sparse/left", 10, std::bind(&DepthMapSubscriberNode::callbackRecievedDepthMap, this, std::placeholders::_1));
            
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