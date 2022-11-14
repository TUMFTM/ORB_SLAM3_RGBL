//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

//Others


/*
This code reads depth map messages of lidar pointclouds (sparse) and performs interpolation to create a dense depth map.
*/

class DepthMapSubscriberNode : public rclcpp::Node{
    public:
        DepthMapSubscriberNode() : Node("depth_map_subsriver"){
            // Subscriber
            sub = this->create_subscription<sensor_msgs::msg::Image>("camera/depth_sparse", 10, std::bind(&DepthMapSubscriberNode::callbackRecievedDepthMap, this, std::placeholders::_1));

            // Publisher
            pub = this->create_publisher<sensor_msgs::msg::Image>("camera/depth", 10);

            // Terminal Info
            RCLCPP_INFO(this->get_logger(), "Depth Map Interpolation Subscriber & Publisher have been started.");
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
            //interpolation_rows(DepthMapToProcess);
            interpolation_cols(DepthMapToProcess);
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
        void interpolation_rows (cv::Mat DepthMapToProcess){
            cv::Mat Row;
            cv::Mat Row_CV8UC1;

            int first_x;
            int last_x;
            int col_idx;

            std::vector<int> idx_n0_x;
            std::vector<cv::Point> idx_n0;

            cv::Mat y1;
            cv::Mat y2;
            int x1;
            int x2;

            for (int y = 0; y<DepthMapToProcess.rows; y++){
                Row = DepthMapToProcess.row(y);
                // Get Non Zero Values
                idx_n0.clear();                             //Holds indices of all non zero elements of Row
                Row.convertTo(Row_CV8UC1, CV_8U);           //Conversion required for findNonZero function
                cv::findNonZero(Row_CV8UC1,  idx_n0);

                //Replace all values before first non zero and after last by the
                //first / last value respectively
                idx_n0_x.clear();
                for (auto const& value: idx_n0){
                    idx_n0_x.push_back(value.x);
                }

                col_idx = 0;
                if (!idx_n0_x.empty()){
                    first_x = idx_n0_x.front();
                    last_x = idx_n0_x.back();
                    for (int idx = 0; idx < Row.cols; idx++){
                        if (idx < first_x){
                            Row.col(idx) = Row.col(first_x);
                        }else if (idx >= last_x){
                            Row.col(idx) = Row.col(last_x);
                        }else if (idx == idx_n0_x[col_idx]){
                            col_idx++;
                        }else{
                            // Linear Interpolation
                            // y = y1 + (x - x1) ((y2 - y1) / (x2 - x1))
                            y1 = Row.col(idx_n0_x[col_idx - 1]);
                            y2 = Row.col(idx_n0_x[col_idx]);
                            x1 = idx_n0_x[col_idx - 1];
                            x2 = idx_n0_x[col_idx];
                            Row.col(idx) = y1 + (idx - x1) * ((y2 - y1) / (x2 - x1));

                            //Row.col(idx) = Row.col(idx_n0_x[col_idx - 1]) + (idx - idx_n0_x[col_idx - 1]) * ((Row.col(idx_n0_x[col_idx]) - Row.col(idx_n0_x[col_idx - 1]) ) / (idx_n0_x[col_idx] - idx_n0_x[col_idx - 1]));

                            //Row.col(idx) = Row.col(idx_n0_x[col_idx - 1]) + (idx - idx_n0_x[col_idx - 1]) * ((Row.col(idx_n0_x[col_idx]) - Row.col(idx_n0_x[col_idx - 1]) ) / (idx_n0_x[col_idx] - idx_n0_x[col_idx - 1]));
                        }
                    }
                }
            }
        }

        void interpolation_cols (cv::Mat DepthMapToProcess){
            cv::Mat Col;
            cv::Mat Col_CV8UC1;

            int first_y;
            int last_y;
            int row_idx;

            std::vector<int> idx_n0_y;
            std::vector<cv::Point> idx_n0;

            cv::Mat y1;
            cv::Mat y2;
            int x1;
            int x2;

            for (int x = 0; x<DepthMapToProcess.cols; x++){
                Col = DepthMapToProcess.col(x);
                // Get Non Zero Values
                idx_n0.clear();                             //Holds indices of all non zero elements of Row
                Col.convertTo(Col_CV8UC1, CV_8U);           //Conversion required for findNonZero function
                cv::findNonZero(Col_CV8UC1,  idx_n0);

                //Replace all values before first non zero and after last by the
                //first / last value respectively
                idx_n0_y.clear();
                for (auto const& value: idx_n0){
                    idx_n0_y.push_back(value.y);
                }

                row_idx = 0;
                if (!idx_n0_y.empty()){
                    first_y = idx_n0_y.front();
                    last_y = idx_n0_y.back();
                    for (int idx = 0; idx < Col.rows; idx++){
                        if (idx < first_y){
                            Col.row(idx) = Col.row(first_y);
                        }else if (idx >= last_y){
                            Col.row(idx) = Col.row(last_y);
                        }else if (idx == idx_n0_y[row_idx]){
                            row_idx++;
                        }else{
                            // Linear Interpolation
                            // y = y1 + (x - x1) ((y2 - y1) / (x2 - x1))
                            y1 = Col.row(idx_n0_y[row_idx - 1]);
                            y2 = Col.row(idx_n0_y[row_idx]);
                            x1 = idx_n0_y[row_idx - 1];
                            x2 = idx_n0_y[row_idx];
                            Col.row(idx) = y1 + (idx - x1) * ((y2 - y1) / (x2 - x1));
                        }
                
                    }
                }
                // else if (y > 0){
                //     Row = DepthMapToProcess.row(y-1);
                // }

                if (x == 100){
                    //std::cout << idx_n0_x[col_idx] << std::endl;
                    for (auto& i: idx_n0_y)
                        std::cout << i << ' ';
                }

            }
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