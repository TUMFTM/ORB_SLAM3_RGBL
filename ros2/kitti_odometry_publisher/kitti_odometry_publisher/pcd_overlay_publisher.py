import os 
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

import cv2
from numpy.core.numeric import Inf
from numpy.lib.function_base import select
from cv_bridge import CvBridge, CvBridgeError

import time

import rclpy
from rclpy.clock import ROSClock
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

from PIL import Image

# ToDo:
# Extend to both cameras
# Pre Filter PCD before alignment

class PCDSubscriber(Node):
    def __init__(self):
        super().__init__('pcd_subscriber_node')

        # Declare Parameters
        # To set parameters append "--ros-args -p ParamName:=Value" to the call or use a launch file
        self.declare_parameter("CalibPath", "")
        self.declare_parameter("CameraIdLeft", -1)
        self.declare_parameter("EnableTimingAnalysis", False)
        self.declare_parameter("EnableSparsityAnalysis", False)
        self.declare_parameter("MinDistance", 0)
        self.declare_parameter("MaxDistance", 100)
        self.declare_parameter("SaveToFile", False)
        self.declare_parameter("SaveToFilePath", "")

        #Initialize variables
        self.img = None             #Holds the image to be projected onto
        self.img_width = None       #Holds the image width to be projected onto
        self.img_height = None      #Holds the image height to be projected onto
        self.pcd = None             #Holds the pcd to be projected
        self.image_projected = None #Holds the processed image (including projected lidar points)
        self.SaveToFile = self.get_parameter("SaveToFile").value      #Holds the processed image (including projected lidar points)
        self.SaveToFilePath = self.get_parameter("SaveToFilePath").value      #Holds the processed image (including projected lidar points)

        # Variables for timing analysis (optional)
        self.projection_time = []                                                           #Debug - array of times required for projection per frame
        self.EnableTimingAnalysis = self.get_parameter("EnableTimingAnalysis").value        #Debug - bool that enables / disables Timing analysis
        
        # Variables for sparsity analysis (optional)
        self.EnableSparsityAnalysis = self.get_parameter("EnableSparsityAnalysis").value    #Debug - bool that enables / disables sparsity analysis

        #Variables for Depth image
        self.MinDistance = self.get_parameter("MinDistance").value # Minimum Depth Value to be stored (i.e. pixel value of 0)
        self.MaxDistance = self.get_parameter("MaxDistance").value # Maximum Depth Value to be stored (i.e. pixel value self.DepthImageMaxVal)
        self.DepthImageDatatype = np.uint16
        self.DepthImageMaxVal = np.iinfo(self.DepthImageDatatype).max   # scale factor for a pixel

        # Read Calib File
        self.CameraIdLeft = self.get_parameter("CameraIdLeft").value
        self.ProjectionMatrixLeft = self.read_calib_file(str(self.get_parameter("CalibPath").value), self.CameraIdLeft)

        # Pointcloud subscriber
        self.subscriber_ = self.create_subscription(sensor_msgs.PointCloud2, "/pointcloud", self.update_pointcloud, 10)
        self.get_logger().info("Point cloud subsciber has been started.")

        # Pointcloud Id subscriber
        self.pcd_id_subscriber = self.create_subscription(std_msgs.Float32, "/pcd_id", self.RecievedPcdId, 10)

        # Image subscriber
        self.subscriber_ = self.create_subscription(sensor_msgs.Image, "/image_left", self.update_image_left, 10)
        self.get_logger().info("Left image subsciber has been started.")

        # Overlay Publishers
        self.overlay_publisher_left = self.create_publisher(sensor_msgs.Image, 'overlay_left', 10)
        self.get_logger().info("Left Lidar / Image overlay publisher has been started.")

        self.depthmap_publisher_left = self.create_publisher(sensor_msgs.Image, 'depthmap_left', 10)
        self.get_logger().info("Left depth map publisher has been started.")
        # self.overlay_publisher_right = self.create_publisher(Image, 'overlay_right', 10)
        # self.get_logger().info("Right Lidar / Image overlay Publisher has been started.")
        # self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, '/left_pointcloud_in_image', 10)

        #Initialize Clock
        self.clock = ROSClock()

        #Initialize CvBridge
        self.bridge = CvBridge()

        #Sparsity
        if self.EnableSparsityAnalysis:
            self.Sparsity_avg = 0
            self.Sparsity_min = Inf
            self.Sparsity_max = -1
            self.Sparsity_numvals = 0
            self.timer = self.create_timer(5, self.timer_callback)

        #Save to File
        if self.SaveToFile:
            self.SaveToFilePath = self.SaveToFilePath + "/image_" + str(self.CameraIdLeft) + "_DepthMap/"
            self.get_logger().info("Created Directory to store depth maps.")

            #Check if Directory exists, create otherwise
            if not os.path.exists(self.SaveToFilePath):
                os.mkdir(self.SaveToFilePath)

    def RecievedPcdId (self, msg):
        try:
            self.pcd_id = float(msg.data)
        except:
            self.pcd_id = -1
            self.get_logger().warn("Could not read pcd Id.")

        
    def update_pointcloud(self, msg):
        # Called when a new pointcloud message was published
        # Replace current pointcloud and create new overlay
        
        #Read pcd message
        recieved_pcd = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        # nPoints, nValues = recieved_pcd.shape
        # self.get_logger().info("Recieved new pcd with {} points and {} values each.".format(nPoints, nValues))

        #Process pcd
        self.pcd = np.insert(recieved_pcd,3,1,axis=1).T
        self.pcd = np.delete(self.pcd, np.where(self.pcd[0,:]<0),axis=1) # Remove points behind car (cannot lie in image); further clipping may be possible
        nPoints, nValues = recieved_pcd.shape

        # Confirm point cloud has been recieved
        self.get_logger().info("Processed new pcd with {} points. Starting overlay.".format(nPoints))

        # Call overlay function
        if not self.img is None:
            self.pcd_to_image (self.pcd, self.img, self.ProjectionMatrixLeft, self.img_width, self.img_height)

    def update_image_left(self, msg):
        # Extract shape
        self.img_width = msg.width
        self.img_height = msg.height

        # Save new image
        self.img = np.array(msg.data).reshape(msg.height, msg.width, -1)

        self.img_height, self.img_width, _ = self.img.shape

        # Confirm image has been recieved
        self.get_logger().info("Recieved new left image with shape [{} x {}]".format(self.img_width, self.img_height))

    def publish_overlay(self):
        ## Publish Overlay
        # Get Timestamp
        timestamp = self.clock.now()

        # Prepare Image Message
        img_left = self.bridge.cv2_to_imgmsg(self.image_projected)
        img_left.header.frame_id = "map"
        img_left.header.stamp = timestamp.to_msg()

        w, h, _ = self.image_projected.shape

        # Publish Image
        self.overlay_publisher_left.publish(img_left)
        self.get_logger().info("Published new left overlay with shape [{} x {}]".format(w,h))

        ## Publish Depth image
        # Prepare Image Message
        dimg_left = self.bridge.cv2_to_imgmsg(self.DepthImage)
        dimg_left.header.frame_id = "map"
        dimg_left.header.stamp = timestamp.to_msg()

        # Publish Image
        self.depthmap_publisher_left.publish(dimg_left)
        self.get_logger().info("Published new depth map.")

        # Save to file if option is enabled
        if self.SaveToFile:
            try:
                FileName = self.SaveToFilePath + str(int(self.pcd_id)).zfill(6) + ".png"
                result = cv2.imwrite(FileName, self.DepthImage)
                if result:
                    self.get_logger().info("Saved Depth map to file: {}".format(FileName))
                else:
                    self.get_logger().warn("Could not save Depth map to file.")
            except:
                self.get_logger().warn("Could not save Depth map to file.")


    def read_calib_file(self, filepath, CamId):
        # Read in a calibration file and parse into a dictionary.
        # Ref: https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py

        # Read Calib file line by line. Store each line (a calib matrix) in a dict by its key
        data = {}
        with open(filepath, 'r') as f:
            for line in f.readlines():
                line = line.rstrip()
                if len(line) == 0: continue
                key, value = line.split(':', 1)
                # The only non-float values in these files are dates, which
                # we don't care about anyway
                try:
                    data[key] = np.array([float(x) for x in value.split()])
                except ValueError:
                    pass

        # Calculate calibraion matrizes
        # Pi e [3 x 4] (left camera)
        if CamId >= 0 and CamId <= 3:
            Pi = np.matrix(data [str("P" + str(CamId))]).reshape(3,4)
        else:
            self.get_logger().error("Invalid Cam Id. Expected integer from 0 to 3 but got {}.".format(str(CamId)))

        #Tr e [3 x 4] -> add a "1" at (4,4) and fill rest of row with "0"
        Tr = np.matrix(data ["Tr"]).reshape(3,4)
        Tr = np.insert(Tr,3,values=[0,0,0,1],axis=0)    #row

        # Calculate projection matrix as matrix product
        # Refer to readme:   x = Pi * Tr * X
        proj = Pi * Tr

        return proj  

    def pcd_to_image (self, pcd, image, proj, img_width, img_height):
        #Log timing
        if self.EnableTimingAnalysis:
            self.TimingStatistics("start")

        #Store pointcloud without projection but only the points in the image
        self.pcd_in_image_unprojected = pcd

        #Project Pointcloud to image
        # Refer to readme:   x = Pi * Tr * X with Pi*Tr = proj in this context
        self.pcd_in_image = proj * pcd
        #self.pcd_in_image = np.delete(self.pcd_in_image,np.where(self.pcd_in_image[2,:]<0)[1],axis=1) #remove points with negative z values
        #self.pcd_in_image_unprojected = np.delete(self.pcd_in_image_unprojected,np.where(self.pcd_in_image[2,:]<0)[1],axis=1)

        # get u,v,z
        self.pcd_in_image[:2] /= self.pcd_in_image[2,:]         #Project to image coordinates
        self.pcd_in_image[:2] = np.rint(self.pcd_in_image[:2])  #Round to interger values as we can only set points in integer pixel values
        #  Given a 3D point [X Y Z]', the projection (x, y) of the point onto
        #  the rectified image is given by:
        #  [u v w]' = P * [X Y Z 1]'
        #         x = u / w
        #         y = v / w
        # from: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg

        # Remove Points that do not lie in the image
        # i.e. the coordinates are < 0 or larger than width / height respectively
        v,u,z = self.pcd_in_image
        u_out = np.logical_or(u < 0, u >= img_height-0.5)    # -0.5 to compensate rounding to discrete values
        v_out = np.logical_or(v < 0, v >= img_width-0.5)
        z_out = np.logical_or(z < self.MinDistance, z > self.MaxDistance)
        idx_to_be_removed = np.logical_or(np.logical_or(u_out, v_out), z_out)
        self.pcd_in_image = np.delete(self.pcd_in_image, np.where(idx_to_be_removed), axis=1)
        self.pcd_in_image_unprojected = np.delete(self.pcd_in_image_unprojected, np.where(idx_to_be_removed), axis=1)

        # Create Colormap
        cmap = plt.cm.get_cmap("hsv")                                        #Create colormap. See https://matplotlib.org/stable/tutorials/colors/colormaps.html for details
        norm = plt.Normalize(vmin=self.MinDistance, vmax=self.MaxDistance)   #normalize distance in range [vmin vmax] -> scaling of colormap

        # Extract Color values (RGB) according to distance z
        y,x,z = self.pcd_in_image
        colors = cmap(norm(z))
        colors = np.delete(np.squeeze(colors), 3, axis=1)*255   # remove alpha column; multiply by 255 to adjust range [0, 1] -> [0, 255]

        #Cast each pixel location to an integer
        x = np.array([round(float(val)) for val in x.T]).T
        y = np.array([round(float(val)) for val in y.T]).T

        #Replace pixel values in image where a lidar point was found
        self.image_projected = image
        self.image_projected [x,y,0] = colors[:,2]  #R-B
        self.image_projected [x,y,1] = colors[:,1]  #G-G
        self.image_projected [x,y,2] = colors[:,0]  #B-R

        # # Save to file if option is enabled
        # if self.SaveToFile:
        #     try:
        #         FileName = self.SaveToFilePath + str(int(self.pcd_id)).zfill(6) + ".png"
        #         result = cv2.imwrite(FileName, self.image_projected)
        #         if result:
        #             self.get_logger().info("Saved Depth map to file: {}".format(FileName))
        #         else:
        #             self.get_logger().warn("Could not save Depth map to file.")
        #     except:
        #         self.get_logger().warn("Could not save Depth map to file.")

        #Calculate sparsity of depth image
        if self.EnableSparsityAnalysis:
            self.SparsityStatistics (img_width*img_height, len(colors))

        #Create Grayscale image 
        self.DepthImage = np.zeros_like(image[:,:,0], dtype=self.DepthImageDatatype)
        z_scaled = np.array(norm(z) * self.DepthImageMaxVal).astype(self.DepthImageDatatype)
        self.DepthImage[x,y] = z_scaled

        #Log timing 
        if self.EnableTimingAnalysis:
            self.TimingStatistics("end")

        # Call Publisher Callback to publish the image
        self.publish_overlay()

    def TimingStatistics (self, key):
        if key == "start":
            ts_start_s, ts_start_ns = self.clock.now().seconds_nanoseconds()
            self.ts_start = ts_start_s + ts_start_ns / 10**9
        elif key == "end":
            ts_end_s, ts_end_ns = self.clock.now().seconds_nanoseconds()
            self.ts_end = ts_end_s + ts_end_ns / 10**9
            self.projection_time.append(self.ts_end-self.ts_start)
        
        if len(self.projection_time)%20 == 0 and len(self.projection_time) > 0:
            self.get_logger().info("Avg time required for transformation {}".format(sum(self.projection_time)/len(self.projection_time)))
            self.get_logger().info("Min time required for transformation {}".format(min(self.projection_time)))
            self.get_logger().info("Max time required for transformation {}".format(max(self.projection_time)))

            self.projection_time = []   #Clear List

    def SparsityStatistics (self, img_size, num_lidar_pixels):
        #Calculate sparsity in %
        sparsity = (1 - (num_lidar_pixels / img_size)) * 100

        #Update avg
        self.Sparsity_avg =  ((self.Sparsity_numvals * self.Sparsity_avg) + sparsity) / (self.Sparsity_numvals + 1)
        self.Sparsity_numvals += 1

        #Update min and max
        if sparsity < self.Sparsity_min:
            self.Sparsity_min = sparsity

        if sparsity > self.Sparsity_max:
            self.Sparsity_max = sparsity

        self.get_logger().info("Sparsity of depth map: {:2.2f} %. Filled {} of {} pixels.".format(sparsity, num_lidar_pixels, img_size))

    def timer_callback(self):
        # Print sparsity statistics every x seconds
        self.get_logger().info("=================================================================")
        self.get_logger().info("======================New Sparsity Statistics====================")
        self.get_logger().info("=================================================================")

        self.get_logger().info("Avg: {:2.2f} %".format(self.Sparsity_avg))
        self.get_logger().info("Min: {:2.2f} %".format(self.Sparsity_min))
        self.get_logger().info("Max: {:2.2f} %".format(self.Sparsity_max))

        self.get_logger().info("=================================================================")



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
    node = PCDSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()






# def pcd_to_image (pcd, image, proj):
#     velo = np.insert(pcd,3,1,axis=1).T
#     velo = np.delete(velo,np.where(velo[0,:]<0),axis=1)
#     cam = proj * velo
#     cam = np.delete(cam,np.where(cam[2,:]<0)[1],axis=1)
#     # get u,v,z
#     cam[:2] /= cam[2,:]
#     # do projection staff
#     plt.figure(figsize=(12,5),dpi=96,tight_layout=True)
#     IMG_H, IMG_W, _ = img.shape
#     # restrict canvas in range
#     plt.axis([0,IMG_W,IMG_H,0])
#     plt.imshow(img)

#     # filter point out of canvas
#     u,v,z = cam
#     u_out = np.logical_or(u<0, u>IMG_W)
#     v_out = np.logical_or(v<0, v>IMG_H)
#     outlier = np.logical_or(u_out, v_out)
#     cam = np.delete(cam,np.where(outlier),axis=1)

#     # generate color map from depth
#     u,v,z = cam
#     plt.scatter([u],[v],c=[z],cmap='rainbow_r',alpha=0.5,s=2)
#     plt.title("Overlay")
#     plt.show()


# basepath = "/media/martin/Volume/datasets/03_KITTI/03_Odometry/dataset/sequences/00/"   #Must end with /

# # Read image
# img_path = basepath + "image_0/000005.png"
# img = cv2.imread(img_path)

# # Read point cloud
# pcd_path = basepath + "velodyne/000005.bin"
# pcd_full = np.fromfile(pcd_path, dtype=np.float32).reshape(-1, 4)
# #pcd = np.delete(pcd_full, 3, 1) #Remove reflectance column
# pcd = pcd_full[:, 0:3] #Remove reflectance column

# # Read Calib File
# calib_path = basepath + "calib.txt"
# calib = read_calib_file(calib_path)

# # Transform 
# image_with_pcd, reduced_pcd = pcd_to_image (pcd, img, calib)