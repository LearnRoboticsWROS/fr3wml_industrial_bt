#
# Minimal launch for the ArUco hand-eye calibration procedure.
#
# Brings up ONLY:
#   - robot_state_publisher (reads the URDF, listens to /joint_states from the
#     fairino_bridge, publishes the full TF tree: base_link, wrist3_link, ...)
#
# Does NOT bring up:
#   - move_group / OMPL / RViz
#   - scene_manager_node
#   - bt_runner_node*
#
# Use case: the user runs ros2_cmd_server + fairino_bridge as usual, then this
# launch in a third terminal, then aruco_calibration_node.py in a fourth.
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_pkg_share = get_package_share_directory(
        "fr3wml_fr5_camera_gripper_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder(
            "fairino3mt_v6_robot",
            package_name="fr3wml_fr5_camera_gripper_moveit_config")
        .robot_description(file_path="config/fairino3mt_v6_robot.urdf.xacro")
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    return LaunchDescription([robot_state_publisher])
