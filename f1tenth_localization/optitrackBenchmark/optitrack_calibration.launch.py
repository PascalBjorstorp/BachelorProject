"""Click map landmarks in RViz and fit the static map -> OptiTrack transform."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("f1tenth_localization")
    planner_pkg_dir = get_package_share_directory("f1tenth_planning")
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(pkg_dir)))
    )
    source_benchmark_dir = os.path.join(
        workspace_root,
        "f1tenth_localization",
        "optitrackBenchmark",
    )

    default_landmarks = os.path.join(
        source_benchmark_dir,
        "optitrack_landmarks.yaml",
    )
    default_map = os.path.join(planner_pkg_dir, "maps", "my_track_map.yaml")
    default_output = os.path.join(
        source_benchmark_dir,
        "optitrack_map_transform.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_file",
            default_value=default_map,
            description="Path to the map YAML file for map_server",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock for simulation time",
        ),
        DeclareLaunchArgument(
            "landmarks_file",
            default_value=default_landmarks,
            description="YAML file with OptiTrack landmark coordinates",
        ),
        DeclareLaunchArgument(
            "output_file",
            default_value=default_output,
            description="Where the fitted transform YAML is written",
        ),
        DeclareLaunchArgument(
            "map_frame",
            default_value="",
            description="Override map frame. Empty means use YAML/default map",
        ),
        DeclareLaunchArgument(
            "optitrack_frame",
            default_value="",
            description="Override OptiTrack frame. Empty means use YAML/default world",
        ),
        DeclareLaunchArgument(
            "clicked_point_topic",
            default_value="/clicked_point",
            description="RViz Publish Point topic",
        ),
        DeclareLaunchArgument(
            "calibration_mode",
            default_value="auto",
            description="auto uses 3D when landmarks are [x,y,z], otherwise 2D",
        ),
        DeclareLaunchArgument(
            "map_point_z",
            default_value="0.0",
            description="Map-frame z value assigned to RViz clicks in 3D mode",
        ),
        DeclareLaunchArgument(
            "use_clicked_z",
            default_value="false",
            description="Use z from RViz clicked point instead of map_point_z",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="Publish fitted static TF while this node is alive",
        ),
        DeclareLaunchArgument(
            "keep_alive_after_fit",
            default_value="true",
            description="Keep node alive after fitting so /tf_static remains available",
        ),
        DeclareLaunchArgument(
            "z_translation",
            default_value="0.0",
            description="Z translation for map -> OptiTrack TF",
        ),
        LifecycleNode(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            namespace="/",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "yaml_filename": map_file,
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server"],
                "bond_timeout": 0.0,
            }],
        ),
        Node(
            package="f1tenth_localization",
            executable="optitrack_map_calibrator.py",
            name="optitrack_map_calibrator",
            output="screen",
            parameters=[{
                "landmarks_file": LaunchConfiguration("landmarks_file"),
                "output_file": LaunchConfiguration("output_file"),
                "map_frame": LaunchConfiguration("map_frame"),
                "optitrack_frame": LaunchConfiguration("optitrack_frame"),
                "clicked_point_topic": LaunchConfiguration("clicked_point_topic"),
                "calibration_mode": LaunchConfiguration("calibration_mode"),
                "map_point_z": LaunchConfiguration("map_point_z"),
                "use_clicked_z": LaunchConfiguration("use_clicked_z"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "keep_alive_after_fit": LaunchConfiguration("keep_alive_after_fit"),
                "z_translation": LaunchConfiguration("z_translation"),
            }],
        ),
    ])
