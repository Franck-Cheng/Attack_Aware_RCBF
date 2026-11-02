"""Proposed V2 safety filter launch — three-tier STRICT -> SOFT -> MIN_RISK.

ISOLATED ADDITION.  `final_robust_sf_v2.launch.py` is left untouched; this file
mirrors its wiring but starts `proposed_v2_filter_node` and provides a goal
(7.5, 0.0) for the node diagnostics.

These are paper-target defaults for new runs; historical settings require verification.

Command chain (unchanged):
  Nav2 -> /nav2_cmd_vel -> SF(V2) -> /cmd_vel_safe -> actuator_attack -> /cmd_vel -> Gazebo
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("attack_safe_tb3_sim")
    gz = get_package_share_directory("gazebo_ros")
    tb3 = get_package_share_directory("turtlebot3_description")
    world = PathJoinSubstitution([pkg, "worlds", LaunchConfiguration("world", default="auto_gate_trap.world")])
    sdf = os.path.join(pkg, "models", "turtlebot3_burger", "model.sdf")
    nav2 = LaunchConfiguration("nav2_params_file",
                               default=os.path.join(pkg, "params", "nav2_params_v2.yaml"))
    robot_desc = ParameterValue(
        Command(["xacro ", os.path.join(tb3, "urdf", "turtlebot3_burger.urdf"),
                 " use_sim_time:=true"]), value_type=str)

    attack_mode = LaunchConfiguration("attack_mode", default="rollout_min_clearance")
    a_v_max = LaunchConfiguration("a_v_max", default="0.132")
    a_w_max = LaunchConfiguration("a_w_max", default="0.60")
    sf_a_v_max = LaunchConfiguration("sf_a_v_max", default="0.132")
    sf_a_w_max = LaunchConfiguration("sf_a_w_max", default="0.60")
    v_safe_min = LaunchConfiguration("v_safe_min", default="-0.22")
    v_safe_max = LaunchConfiguration("v_safe_max", default="0.22")
    w_safe_max = LaunchConfiguration("w_safe_max", default="1.0")
    alpha_h = LaunchConfiguration("alpha_h", default="1.50")
    w_v = LaunchConfiguration("w_v", default="20.0")
    w_w = LaunchConfiguration("w_w", default="0.8")
    v_nom_max = LaunchConfiguration("v_nom_max", default="0.22")
    lambda_delta = LaunchConfiguration("lambda_delta", default="10.0")
    minrisk_tau = LaunchConfiguration("minrisk_tau", default="1e-9")
    task_mode = LaunchConfiguration("task_mode", default="not_reverse")
    goal_x = LaunchConfiguration("goal_x", default="7.5")
    goal_y = LaunchConfiguration("goal_y", default="0.0")
    rollout_horizon = LaunchConfiguration("rollout_horizon", default="24")
    trigger_clearance = LaunchConfiguration("trigger_clearance", default="0.45")
    omega_eff_gain = LaunchConfiguration("omega_eff_gain", default="0.55")
    omega_eff_limit = LaunchConfiguration("omega_eff_limit", default="0.83")

    return LaunchDescription([
        SetEnvironmentVariable(name="GAZEBO_MODEL_PATH",
                               value=os.path.join(pkg, "models") + ":" + "/usr/share/gazebo-11/models"),
        SetEnvironmentVariable(name="GAZEBO_MODEL_DATABASE_URI", value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value="auto_gate_trap.world"),
        DeclareLaunchArgument("attack_mode", default_value="rollout_min_clearance"),
        DeclareLaunchArgument("a_v_max", default_value="0.132"),
        DeclareLaunchArgument("a_w_max", default_value="0.60"),
        DeclareLaunchArgument("sf_a_v_max", default_value="0.132"),
        DeclareLaunchArgument("sf_a_w_max", default_value="0.60"),
        DeclareLaunchArgument("v_safe_min", default_value="-0.22"),
        DeclareLaunchArgument("v_safe_max", default_value="0.22"),
        DeclareLaunchArgument("w_safe_max", default_value="1.0"),
        DeclareLaunchArgument("nav2_params_file",
                              default_value=os.path.join(pkg, "params", "nav2_params_v2.yaml")),
        DeclareLaunchArgument("alpha_h", default_value="1.50"),
        DeclareLaunchArgument("w_v", default_value="20.0"),
        DeclareLaunchArgument("w_w", default_value="0.8"),
        DeclareLaunchArgument("v_nom_max", default_value="0.22"),
        DeclareLaunchArgument("lambda_delta", default_value="10.0"),
        DeclareLaunchArgument("minrisk_tau", default_value="1e-9"),
        DeclareLaunchArgument("task_mode", default_value="not_reverse"),
        DeclareLaunchArgument("goal_x", default_value="7.5"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("rollout_horizon", default_value="24"),
        DeclareLaunchArgument("trigger_clearance", default_value="0.45"),
        DeclareLaunchArgument("omega_eff_gain", default_value="0.55"),
        DeclareLaunchArgument("omega_eff_limit", default_value="0.83"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, "launch", "gzserver.launch.py")),
            launch_arguments={"world": world, "verbose": "false"}.items()),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-entity", "ugv", "-file", sdf,
                        "-x", LaunchConfiguration("spawn_x"),
                        "-y", LaunchConfiguration("spawn_y"), "-z", "0.01"],
             output="screen"),
        TimerAction(period=2.0, actions=[
            Node(package="scan_range_limiter", executable="scan_range_limiter_node",
                 name="scan_range_limiter",
                 parameters=[{"use_sim_time": True, "max_range": 1.5,
                              "input_topic": "/scan",
                              "output_topic": "/scan_defense_1p5"}], output="screen")]),
        TimerAction(period=3.0, actions=[
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 name="robot_state_publisher",
                 parameters=[{"use_sim_time": True, "robot_description": robot_desc}])]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_odom_tf",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        TimerAction(period=5.0, actions=[
            Node(package="nav2_map_server", executable="map_server", name="map_server",
                 parameters=[{"use_sim_time": True,
                              "yaml_filename": os.path.join(pkg, "maps", "blank_map.yaml")}]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_mgr_map",
                 parameters=[{"use_sim_time": True, "autostart": True,
                              "node_names": ["map_server"]}])]),
        TimerAction(period=5.0, actions=[
            Node(package="nav2_controller", executable="controller_server", name="controller_server",
                 parameters=[nav2, {"use_sim_time": True}],
                 remappings=[("/cmd_vel", "/nav2_cmd_vel")]),
            Node(package="nav2_planner", executable="planner_server", name="planner_server",
                 parameters=[nav2, {"use_sim_time": True}]),
            # Recovery behaviours must enter the same upstream topic as the
            # controller; otherwise Spin/BackUp bypass the SF and attack chain.
            Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
                 parameters=[nav2, {"use_sim_time": True}],
                 remappings=[("/cmd_vel", "/nav2_cmd_vel")]),
            Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
                 parameters=[nav2, {"use_sim_time": True}]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_mgr_nav",
                 parameters=[{"use_sim_time": True, "autostart": True,
                              "node_names": ["controller_server", "planner_server",
                                             "behavior_server", "bt_navigator"],
                              "bond_timeout": 30.0}])]),

        # ── Proposed V2 SF (t=16s) ──
        TimerAction(period=16.0, actions=[
            Node(package="safety_filter", executable="proposed_v2_filter_node",
                 name="safety_filter_node",
                 remappings=[("/cmd_vel_nom", "/nav2_cmd_vel"),
                             ("/scan", "/scan_defense_1p5")],
                 parameters=[{"use_sim_time": True,
                              "final_mode": True,
                              "goal_x": goal_x, "goal_y": goal_y,
                              "v_safe_min": v_safe_min, "v_safe_max": v_safe_max,
                              "w_safe_max": w_safe_max,
                              "a_v_max": sf_a_v_max, "a_w_max": sf_a_w_max,
                              "alpha_h": alpha_h,
                              "max_scan_age": 0.50,
                              "d_geom": 0.01, "d_delay": 0.005, "d_hold": 0.005,
                              "w_v": w_v, "w_w": w_w,
                              "v_nom_max": v_nom_max,
                              "lambda_delta": lambda_delta,
                              "minrisk_tau": minrisk_tau,
                              "task_mode": task_mode}],
                 output="screen")]),
        # ── attack (t=18s) — unchanged position: AFTER the safety filter ──
        TimerAction(period=18.0, actions=[
            Node(package="actuator_attack", executable="actuator_attack_node",
                 name="actuator_attack_node",
                 parameters=[{"use_sim_time": True,
                              "attack_mode": attack_mode,
                              "a_v_max": a_v_max, "a_w_max": a_w_max,
                              "rollout_horizon": rollout_horizon,
                              "trigger_clearance": trigger_clearance,
                              "omega_eff_gain": omega_eff_gain,
                              "omega_eff_limit": omega_eff_limit}],
                 output="screen")]),
    ])