#
# Minimal bringup for manual MoveIt teaching / waypoint capture.
#
# Brings up:
#   - robot_state_publisher (URDF from fr3wml_industrial_bt via the MoveIt
#     config xacro include)
#   - move_group (MoveIt planner)
#   - RViz with the MoveIt motion planning panel
#   - joint_state_broadcaster + moveit_joint_controller (so the robot can
#     accept FollowJointTrajectory actions from MoveIt)
#   - scene_manager_node (loads bottle/crate/capsule into the planning scene
#     from scene_bottle_capsule.yaml — same as the production launches)
#
# Does NOT bring up:
#   - bt_runner_node (no behavior tree)
#   - aruco_calibration_node
#   - bridges (the user must run ros2_cmd_server and fairino_bridge separately)
#
# Use case: drive the robot to a desired pose via the MoveIt RViz UI, read
# the resulting joint values from /joint_states, and copy them into
# bt_runner.yaml as e.g. pre_pick_capsule_joints_deg / pre_place_bottle_joints_deg.
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_pkg_share = get_package_share_directory(
        "fr3wml_fr5_camera_gripper_moveit_config")
    this_pkg = get_package_share_directory("fr3wml_industrial_bt")

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
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
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
        output="screen",
        arguments=["-d", os.path.join(moveit_pkg_share, "config", "moveit.rviz")],
        parameters=[moveit_config.to_dict()],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    moveit_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["moveit_joint_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Same scene_manager_node used by the production BT launches — reads
    # scene_bottle_capsule.yaml and exposes /scene/remove_object and
    # /scene/set_object_pose services. For manual teaching you usually don't
    # touch those services, but having the planning scene populated is useful
    # to see collisions in RViz.
    scene_manager = Node(
        package="industrial_bt_framework",
        executable="scene_manager_node",
        name="scene_manager_node",
        output="screen",
        parameters=[os.path.join(this_pkg, "config", "scene_bottle_capsule.yaml")],
    )

    delay_moveit_joint_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[moveit_joint_controller_spawner],
        )
    )

    delayed_scene_manager = TimerAction(period=5.0, actions=[scene_manager])

    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        delay_moveit_joint_controller,
        delayed_scene_manager,
    ])
