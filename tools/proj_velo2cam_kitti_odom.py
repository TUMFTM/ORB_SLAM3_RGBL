from multiprocessing.resource_sharer import stop
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os
import copy
import imageio as im

sequence = "00"

show_overlay = False

cur_dir = os.path.abspath(os.path.dirname(__file__))

image_path = os.path.join(cur_dir, "../", sequence, "image_2")
velodyne_path = os.path.join("../", sequence,  "velodyne")
calib_path = os.path.join(cur_dir, "..", sequence)
save_path = os.path.join("../depth_png", sequence)

filelist = [os.path.splitext(filename)[0] for filename in os.listdir(os.path.join(cur_dir, "..", sequence, "velodyne"))]
filelist.sort()
print("Generating " + str(len(filelist)) + " overlay images!")

image_counter = 0
num_images = len(filelist)

for name in filelist:
    img = os.path.join(image_path, name + ".png")
    binary = os.path.join(velodyne_path, name + ".bin")
    with open(os.path.join(calib_path, "calib.txt"),'r') as f:
        calib = f.readlines()

    # P2 (3 x 4) for left eye
    P2 = np.matrix([float(x) for x in calib[2].strip('\n').split(' ')[1:]]).reshape(3,4)
    
    # Odometry: No rect needed
    # R0_rect = np.matrix([float(x) for x in calib[4].strip('\n').split(' ')[1:]]).reshape(3,3)
    # Add a 1 in bottom-right, reshape to 4 x 4
    # R0_rect = np.insert(R0_rect,3,values=[0,0,0],axis=0)
    # R0_rect = np.insert(R0_rect,3,values=[0,0,0,1],axis=1)

    # Odometry: index 4 needed instead of 5
    Tr_velo_to_cam = np.matrix([float(x) for x in calib[4].strip('\n').split(' ')[1:]]).reshape(3,4)
    Tr_velo_to_cam = np.insert(Tr_velo_to_cam,3,values=[0,0,0,1],axis=0)

    # read raw data from binary
    scan = np.fromfile(binary, dtype=np.float32).reshape((-1, 4))
    points = scan[:, 0:3] # lidar xyz (front, left, up)
    # TODO: use fov filter? 
    velo = np.insert(points, 3, 1, axis=1).T
    velo = np.delete(velo, np.where(velo[0,:]<0), axis=1)
    
    # Odometry: no rect needed
    # cam = P2 * R0_rect * Tr_velo_to_cam * velo
    cam = P2 * Tr_velo_to_cam * velo

    cam = np.delete(cam, np.where(cam[2,:]<0)[1], axis=1)
    # get u,v,z
    cam[:2] /= cam[2,:]
    # do projection staff
    
    png = mpimg.imread(img)
    IMG_H,IMG_W,_ = png.shape

    # restrict canvas in range
    # filter point out of canvas
    
    u,v,z = cam
    u_out = np.logical_or(u<0, u>IMG_W)
    v_out = np.logical_or(v<0, v>IMG_H)
    outlier = np.logical_or(u_out, v_out)
    cam = np.delete(cam, np.where(outlier),axis=1)
    # generate color map from depth
    u,v,z = cam

    if show_overlay:
        plt.figure(1, figsize=(12,5), dpi=96, tight_layout=True)
        plt.axis([0,IMG_W,IMG_H,0])
        plt.imshow(png)
        plt.scatter([u], [v], c=[z], cmap='rainbow_r', alpha=0.5, s=2)
        # plt.title(name)
        plt.axis("off")
        plt.show()
    
    image_data = np.array(cam)
    image_matrix = np.ones((IMG_H, IMG_W), dtype=np.uint16) * 0

    index=0
    for i in image_data[1,:]:
        image_matrix[int(i), int(image_data[0, index])] = image_data[2, index] * (256.)
        # print(image_matrix[int(i), int(image_data[0, index])])
        index = index+1

    im.imwrite(os.path.join(save_path, name + ".png"), im=(image_matrix))

    print("Finished ", image_counter, " out of ", num_images) 
    image_counter += 1
