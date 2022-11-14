import os
from glob import glob
from setuptools import setup

package_name = 'kitti_odometry_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='martin',
    maintainer_email='martin.rudolph@tum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "image_publisher = kitti_odometry_publisher.image_publisher:main",
            "pcd_publisher = kitti_odometry_publisher.pcd_publisher:main",
            "pcd_overlay_publisher = kitti_odometry_publisher.pcd_overlay_publisher:main",
        ],
    },
)
