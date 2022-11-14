import sys
import os
import time
from rclpy.time import Time
from rclpy.clock import ROSClock

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

import numpy as np
#import open3d as o3d

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher_node')

        # Declare parameters
        # To set parameters append "--ros-args -p ParamName:=Value" to the call or use a launch file
        self.declare_parameter("PathToSequence", "")
        self.declare_parameter("FramesPerSecond", 10)


        self.declare_parameter("LidarDir", "/velodyne/")       # For Odometry Dataset not required to change

        # Variables
        self.pcd_cur = 0        # Counter Variable
        self.img_cur = 0    
        self.mode = "IAC"       # Defines which file format is expected

        # Get elements in directory
        self.pcd_dir_path = str(self.get_parameter("PathToSequence").value) + str(self.get_parameter("LidarDir").value)
        self.get_logger().info(self.pcd_dir_path)

        # Get elements in directory
        self.list_pcd = [f for f in os.listdir(self.pcd_dir_path) if os.path.isfile(os.path.join(self.pcd_dir_path, f))]
        self.list_pcd.sort()
        self.nPointcloulds = len(self.list_pcd)

        # Initialize Publisher
        self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, '/pointcloud', 10)
        self.pcd_publisher_id = self.create_publisher(std_msgs.Float32, 'pcd_id', 10)

        # Initialize Subscriber to Image Id
        self.img_id_subscriber = self.create_subscription(std_msgs.Float32, "/image_id", self.RecievedImageId, 10)

        # Initialize Timer
        timer_period = 1 / self.get_parameter("FramesPerSecond").value
        self.timer = self.create_timer(timer_period, self.PublishNewPointcloud)

    def RecievedImageId (self, msg):
        # If a new image id is sent (just before a new image gets published) we check wether this is the same id as the pointcloud to be published next
        # In case there is a delta, we reset the current pointcloud counter to be the same as the image counter such that for the next pcd to be published
        # (which triggers the overlay) fits to the currently published image
        if not self.pcd_cur == int(msg.data):
            self.get_logger().info("Recived new image Id: {}".format(str(msg.data)))
            self.pcd_cur = int(msg.data)
            self.get_logger().info("Corrected Pointcloud Id to: {}".format(str(msg.data)))

    def PublishNewPointcloud(self):
        if self.pcd_cur > self.nPointcloulds:
            print('Finished! Published ' + str(self.nPointcloulds) + ' point clouds.')
            rclpy.shutdown()
        
        # Read Pointcloud and convert it to a numpy array
        pcd_full = np.fromfile(self.pcd_dir_path + "/" + self.list_pcd[self.pcd_cur], dtype=np.float32).reshape(-1, 4)
        pcd = np.delete(pcd_full, 3, 1) #Remove reflectance column
        nPoints, nValues = pcd.shape

        # Create Message
        pcd_msg = point_cloud(pcd, 'map')

        #Publish current Id
        pcd_id = std_msgs.Float32()
        pcd_id.data = float(self.pcd_cur)
        self.pcd_publisher_id.publish(pcd_id)

        # Publish PointCloud
        self.pcd_publisher.publish(pcd_msg)
        self.get_logger().info("Published Pointcloud {} with {} points.".format(str(self.list_pcd[self.pcd_cur]), nPoints))

        #Increment Counter
        self.pcd_cur += 1

def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """
    # In a PointCloud2 message, the point cloud is stored as an byte
    # array. In order to unpack it, we also include some parameters
    # which desribes the size of each individual point.

    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes()

    # The fields specify what the bytes represents. The first 4 bytes
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [sensor_msgs.PointField(
        name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which
    # coordinate frame it is represented in.
    header = std_msgs.Header(frame_id=parent_frame)

    clock = ROSClock()
    timestamp = clock.now()
    header.stamp = timestamp.to_msg()

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )


def main(args=None):
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher()
    rclpy.spin(pcd_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcd_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()