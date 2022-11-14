import sys
import os
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from rclpy.clock import ROSClock
from rclpy.node import Node

import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher_node')

        # Declare parameters
        # To set parameters append "--ros-args -p ParamName:=Value" to the call or use a launch file
        self.declare_parameter("PathToSequence", "")
        self.declare_parameter("FramesPerSecond", 10)

        self.declare_parameter("ImageDirLeft", "image_0/")       # For Odometry Dataset not required to change
        self.declare_parameter("ImageDirRight", "image_1/")      # For Odometry Dataset not required to change

        # Counter Variable
        self.img_cur = 0

        # Get elements in directory
        self.img_dir_path = str(self.get_parameter("PathToSequence").value)

        #Get Left Image Names
        self.ImageDirLeft = self.img_dir_path + "/" + self.get_parameter("ImageDirLeft").value
        self.get_logger().info("Left Image Dir: {}".format(self.ImageDirLeft))
        self.list_image_left = [f for f in os.listdir(self.ImageDirLeft) if os.path.isfile(os.path.join(self.ImageDirLeft, f))]
        self.list_image_left.sort()
        self.get_logger().info("Found {} images from the left camera".format(str(len(self.list_image_left))))

        #Get Right Image Names
        self.ImageDirRight = self.img_dir_path + "/" + self.get_parameter("ImageDirRight").value
        self.get_logger().info("Right Image Dir: {}".format(self.ImageDirRight))
        self.list_image_right = [f for f in os.listdir(self.ImageDirRight) if os.path.isfile(os.path.join(self.ImageDirRight, f))]
        self.list_image_right.sort()
        self.get_logger().info("Found {} images from the right camera".format(str(len(self.list_image_right))))

        #Check List lengths
        if len(self.list_image_left) > len(self.list_image_right):
            self.get_logger().warn("Found {} images from the left camera but only {} images from the right camera".format(len(self.list_image_left), len(self.list_image_right)))
            self.nImages = len(self.list_image_right)
        elif len(self.list_image_left) < len(self.list_image_right):
            self.get_logger().warn("Found {} images from the right camera but only {} images from the left camera".format(len(self.list_image_right), len(self.list_image_left)))
            self.nImages = len(self.list_image_left)
        else:
            self.nImages = len(self.list_image_left)

        #Initialize Publishers
        self.img_publisher_left = self.create_publisher(Image, 'image_left', 10)
        self.img_publisher_right = self.create_publisher(Image, 'image_right', 10)
        self.img_publisher_id = self.create_publisher(Float32, 'image_id', 10)

        #Initialize Timer
        timer_period = 1 / self.get_parameter("FramesPerSecond").value
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Initialize CvBridge
        self.bridge = CvBridge()

    def timer_callback(self):
        #Shutdown after all images were published
        if self.img_cur > self.nImages:
            print('Finished! Published ' + str(len(self.self.nImages)) + ' images.')
            rclpy.shutdown()

        #Publish current Id
        img_id = Float32()
        img_id.data = float(self.img_cur)
        self.img_publisher_id.publish(img_id)
        self.get_logger().info("Next image to be published: {}".format(str(self.img_cur)))

        try:
            cv_image_left = cv2.imread(self.ImageDirLeft + "/" + self.list_image_left[self.img_cur])
            cv_image_right = cv2.imread(self.ImageDirLeft + "/" + self.list_image_right[self.img_cur])
        except CvBridgeError as e:
            print(e)

        # Get Timestamp
        clock = ROSClock()
        timestamp = clock.now()

        # Prepare Left Image Message
        img_left = self.bridge.cv2_to_imgmsg(cv_image_left)
        img_left.header.frame_id = "map"
        img_left.header.stamp = timestamp.to_msg()
    
        # Prepare Right Image Message
        img_right = self.bridge.cv2_to_imgmsg(cv_image_right)
        img_right.header.frame_id = "map"
        img_right.header.stamp = timestamp.to_msg()

        # Publish Images
        self.img_publisher_left.publish(img_left)
        self.get_logger().info("Published Left Image: {}".format(str(self.list_image_right[self.img_cur])))
        self.img_publisher_right.publish(img_right)
        self.get_logger().info("Published Right Image: {}".format(str(self.list_image_right[self.img_cur])))

        # Increment Counter
        self.img_cur += 1


def main(args=None):
    rclpy.init(args=args)
    img_publisher = ImagePublisher()
    rclpy.spin(img_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)