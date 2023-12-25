from os import path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    launch_path = path.realpath(__file__)
    launch_dir = path.join(path.dirname(launch_path), '..')
    param_dir = path.join(launch_dir, "param")

    lidar_3d_object_detection_node = Node(
        package = 'mmdet3d_inference',
        executable = 'lidar_inferencer_node',
        name = 'lidar_3d_object_detection_node',
        parameters=[
            (path.join(param_dir, "mmdet3d_lidar_inference_params.yaml")),
        ],
    )

    object3d_visualizer_node = Node(
        package='object_visualization',
        executable='object3d_visualizer_node',
        name='object3d_visualizer_node',
    )

    return LaunchDescription([
        lidar_3d_object_detection_node,
        object3d_visualizer_node
    ])