//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

//Others
//#include <eigen3/Eigen/Dense>
#include <chrono>

/*
This code reads depth map messages of lidar pointclouds (sparse) and performs interpolation to create a dense depth map.

The process is as follows: 
-We apply a convolution with an n x n kernel scaled by 1/n² (e.g. 1/9 * [[1,1,1],[1,1,1],[1,1,1]])
-After the convolution, each pixel holds the avg of all pixels in the nxn neighborhood. 
-Since most pixels are empty (i.e. value = 0) we need to scale this result by the number of valid pixels 
(where value > 0; a distance is known). To get this number, we transform the image to a binary image, where 
every value > 0 is set to 1. A convolution with a n x n unit kernel (e.g. [[1,1,1],[1,1,1],[1,1,1]]) extracts 
the number of non zero pixels each of the n x n neighborhoods.
-This matrix is multiplied (element by element) with the filtered image to get the final image.

-Finally for the upper region of the image, no depth measurements exist (no Lidar beams). The depth value is set to the highest valid pixel
in that column (i.e. for each column the pixel where value > 0 and y minimal).
*/

class DepthMapSubscriberNode : public rclcpp::Node{
    public:
        DepthMapSubscriberNode() : Node("depth_map_subsriver"){
            // Subscriber
            sub = this->create_subscription<sensor_msgs::msg::Image>("camera/depth_sparse", 10, std::bind(&DepthMapSubscriberNode::callbackRecievedDepthMap, this, std::placeholders::_1));

            // Publisher
            pub = this->create_publisher<sensor_msgs::msg::Image>("camera/depth_dense/left", 10);

            // Terminal Info
            RCLCPP_INFO(this->get_logger(), "Depth Map Interpolation Subscriber & Publisher have been started.");
        }

    private:
        // ===============================================
        // Subscriber and Publisher; Processing Call
        void callbackRecievedDepthMap(const sensor_msgs::msg::Image::SharedPtr msg){
            // Recieve Depth Map
            try{
                SparseDepthMapLeft = cv_bridge::toCvCopy(msg)->image;
                SparseDepthMapLeft.convertTo(SparseDepthMapLeft, CV_16U);
            }
            catch (cv_bridge::Exception& e){
                RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Recieved new depth map with shape [%d x %d]", msg->width, msg->height);

            // Process the Depth Map as desired
            ProcessDepthMap_wTiming(SparseDepthMapLeft);
        }

        void ProcessDepthMap_wTiming (cv::Mat& DepthMapToProcess){

            // Start Timer
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

            // Call Manipulating Functions
            FillPixels_nxn(DepthMapToProcess);
            FillUpperRegion(DepthMapToProcess);

            // End Timer, calc required time and output
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

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

        void FillPixels_nxn (cv::Mat& RawImage){
            // Takes sparse depth image and fills empty pixels by calculating the avg of nxn neighboring patch
            // Convolution with unity kernel (1 e [nxn]) and divide by number of non zero pixels in that patch

            //Convolution Parameters
            cv::Point anchor = cv::Point(-1, -1);
            int delta = 0;
            int ddepth = -1;


            // Convolve scaled image with nxn kernel
            cv::Mat FilteredImage;
            RawImage.convertTo(FilteredImage, CV_32F);
            cv::Mat kernel = cv::Mat::ones(PatchSize, PatchSize, CV_32F) / (PatchSize*PatchSize);
            cv::filter2D(FilteredImage, FilteredImage, ddepth , kernel, anchor, delta, cv::BORDER_DEFAULT);

            // Count number of Pixels in each patch
            cv::Mat PixelsPerPatch;
            cv::Mat UnitKernel = cv::Mat::ones(PatchSize, PatchSize, CV_16U);
            cv::threshold(RawImage, PixelsPerPatch, 0, 1, 0); // Convert everything above 0 to 1
            cv::filter2D(PixelsPerPatch, PixelsPerPatch, ddepth , UnitKernel, anchor, delta, cv::BORDER_DEFAULT); // holds number of non zero pixels in patch

            double minVal, maxVal;
            minMaxLoc(PixelsPerPatch, &minVal, &maxVal);
            std::cout << "minVal = " << minVal << std::endl;
            std::cout << "maxVal = " << maxVal << std::endl;

            // Divide the two images
            cv::multiply(FilteredImage, PixelsPerPatch, RawImage, 1, CV_16U);
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

        // Other
        int PatchSize = 8;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthMapSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
