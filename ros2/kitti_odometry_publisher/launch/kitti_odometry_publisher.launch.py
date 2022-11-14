import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    SequencePath = "/home/martin/data/datasets/03_KITTI/03_Odometry/dataset/sequences/02/"
    FPS = 5

    #Initialize Launcher
    ld = launch.LaunchDescription()

    # Initialize Nodes
    image_publisher_node = launch_ros.actions.LifecycleNode(
        package="kitti_odometry_publisher",
        executable="image_publisher",
        name="image_publisher",
        parameters=[
            {"PathToSequence": SequencePath},
            {"FramesPerSecond": FPS}
        ]
    )
    pcd_publisher_node = launch_ros.actions.LifecycleNode(
        package="kitti_odometry_publisher",
        executable="pcd_publisher",
        name="pcd_publisher",
        parameters=[
            {"PathToSequence": SequencePath},
            {"FramesPerSecond": FPS}
        ]
    )
    pcd_overlay_publisher_node = launch_ros.actions.LifecycleNode(
        package="kitti_odometry_publisher",
        executable="pcd_overlay_publisher",
        name="overlay_publisher",
        parameters=[
            {"CalibPath": SequencePath + "calib.txt"},
            {"EnableTimingAnalysis": False},
            {"EnableSparsityAnalysis": False},
            {"CameraIdLeft": 0},
            {"SaveToFile": True},
            {"SaveToFilePath": SequencePath}
        ]
    )

    ld.add_action(image_publisher_node)
    ld.add_action(pcd_publisher_node)
    ld.add_action(pcd_overlay_publisher_node)

    return ld