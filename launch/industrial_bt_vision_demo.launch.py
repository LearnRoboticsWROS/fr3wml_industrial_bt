#
# FR3WML bringup — vision variant (headless, no Groot2).
#
# Same layout as industrial_bt_demo.launch.py, with two differences:
#   * bt_tree_file points at bottle_capsule_task_vision.xml
#   * bt_runner_node receives an additional YAML file (bt_runner_vision.yaml)
#     on top of the base bt_runner.yaml; the extra file only adds the
#     `task_parameters.capsule_*` keys consumed by the new vision BT nodes.
#
# The Jetson must be publishing /detected_objects/poses_6d (and TFs on
# ROS_DOMAIN_ID=10). Start ros2_cmd_server and fairino_bridge separately as
# with the non-vision demo.
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share        = get_package_share_directory("fr3wml_industrial_bt")
    moveit_pkg_share = get_package_share_directory(
        "fr3wml_fr5_camera_gripper_moveit_config")

    default_bt_params        = os.path.join(pkg_share, "config", "bt_runner.yaml")
    default_vision_params    = os.path.join(pkg_share, "config", "bt_runner_vision.yaml")
    default_motion_profiles  = os.path.join(pkg_share, "config", "motion_profiles.yaml")
    default_scene_params     = os.path.join(pkg_share, "config", "scene_bottle_capsule.yaml")
    default_backend_params   = os.path.join(pkg_share, "config", "fairino_movel_backend.yaml")
    default_bt_tree          = os.path.join(pkg_share, "bt_trees",
                                            "bottle_capsule_task_vision.xml")

    args = [
        DeclareLaunchArgument("bt_params_file",         default_value=default_bt_params),
        DeclareLaunchArgument("vision_params_file",     default_value=default_vision_params),
        DeclareLaunchArgument("motion_profiles_file",   default_value=default_motion_profiles),
        DeclareLaunchArgument("scene_params_file",      default_value=default_scene_params),
        DeclareLaunchArgument("cartesian_backend_params_file",
                              default_value=default_backend_params),
        DeclareLaunchArgument("bt_tree_file",           default_value=default_bt_tree),
        DeclareLaunchArgument("bt_tree_id",             default_value="MainTree"),
        DeclareLaunchArgument("use_rviz",               default_value="true"),
    ]

    moveit_config = (
        MoveItConfigsBuilder(
            "fairino3mt_v6_robot",
            package_name="fr3wml_fr5_camera_gripper_moveit_config")
        .robot_description(file_path="config/fairino3mt_v6_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/fairino3mt_v6_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            {"moveit_controller_manager":
                "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"controllers_file": os.path.join(
                moveit_pkg_share, "config", "moveit_controllers.yaml")},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="both",
        arguments=["-d", os.path.join(moveit_pkg_share, "config", "moveit.rviz")],
        parameters=[moveit_config.to_dict()],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )

    moveit_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["moveit_joint_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    delay_moveit_joint_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[moveit_joint_controller_spawner],
        )
    )

    scene_manager = Node(
        package="industrial_bt_framework",
        executable="scene_manager_node",
        name="scene_manager_node",
        output="both",
        parameters=[LaunchConfiguration("scene_params_file")],
    )

    bt_runner = Node(
        package="industrial_bt_framework",
        executable="bt_runner_node",
        name="bt_runner_node",
        output="both",
        parameters=[
            LaunchConfiguration("bt_params_file"),
            LaunchConfiguration("vision_params_file"),
            {
                "motion_profiles_file":           LaunchConfiguration("motion_profiles_file"),
                "bt_tree_file":                   LaunchConfiguration("bt_tree_file"),
                "bt_tree_id":                     LaunchConfiguration("bt_tree_id"),
                "cartesian_backend_params_file":  LaunchConfiguration("cartesian_backend_params_file"),
            },
        ],
    )

    delayed_scene = TimerAction(period=5.0,  actions=[scene_manager])
    delayed_bt    = TimerAction(period=10.0, actions=[bt_runner])

    return LaunchDescription(args + [
        robot_state_publisher,
        move_group_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        delay_moveit_joint_controller,
        delayed_scene,
        delayed_bt,
    ])
