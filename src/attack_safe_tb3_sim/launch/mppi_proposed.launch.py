#!/usr/bin/env python3
"""mppi_proposed.launch.py — Nav2/MPPI + Proposed RCBF safety filter (Gazebo demo).

Control chain (this launch starts ONLY the Proposed filter, no baselines):

    Nav2 MPPI  --/nav2_cmd_vel-->  ProposedV2 SF  --/cmd_vel_safe-->  [attacker]  -->  /cmd_vel  -->  Gazebo

`attack:=true` inserts the post-filter actuator-command injection node between the
filter output and the plant (u_act = u_sf + a).

Speed presets (the two operating points used for the MPPI experiments):

    speed:=low    MPPI vx_max 0.22, wz_max 1.0    SF v<=0.242,  |w|<=1.10
    speed:=high   MPPI vx_max 0.35, wz_max 1.6    SF v<=0.385,  |w|<=1.76

Usage:
    ros2 launch attack_safe_tb3_sim mppi_proposed.launch.py speed:=low
    ros2 launch attack_safe_tb3_sim mppi_proposed.launch.py speed:=high attack:=true

The goal must be sent separately, e.g.:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
        "{pose: {header: {frame_id: map}, pose: {position: {x: 7.5, y: 0.0}}}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Proposed design parameters shared by both speeds (must match the paper setup).
ALPHA_H = 2.30
W_V = 20.420091
W_W = 0.379909
LAMBDA_DELTA = 10.0
MINRISK_TAU = 1e-9
TASK_MODE = "not_reverse"

# Per-speed presets. The filter assumes the full attacker bound (beta = 1.0):
# `sf_*` equals `atk_*` in every preset. `atk_*` are the amplitudes injected
# when attack:=true.
SPEED_PRESETS = {
    "low": dict(
        params="nav2_params_mppi_low.yaml",
        v_safe_min=-0.242, v_safe_max=0.242, w_safe_max=1.10, v_nom_max=0.242,
        sf_a_v_max=0.132, sf_a_w_max=0.600,
        atk_a_v_max=0.132, atk_a_w_max=0.600,
    ),
    "high": dict(
        params="nav2_params_mppi_high.yaml",
        v_safe_min=-0.242, v_safe_max=0.385, w_safe_max=1.76, v_nom_max=0.385,
        sf_a_v_max=0.210, sf_a_w_max=0.960,
        atk_a_v_max=0.210, atk_a_w_max=0.960,
    ),
}


def _setup(context, *args, **kwargs):
    def g(name):
        return LaunchConfiguration(name).perform(context)

    speed = (g("speed") or "low").strip()
    if speed not in SPEED_PRESETS:
        raise RuntimeError(f"speed must be one of {sorted(SPEED_PRESETS)}, got {speed!r}")
    p = SPEED_PRESETS[speed]

    world = g("world") or "auto_gate_trap.world"
    with_attack = (g("attack") or "false").lower() in ("1", "true", "yes", "on")

    pkg = get_package_share_directory("attack_safe_tb3_sim")
    gz = get_package_share_directory("gazebo_ros")
    tb3 = get_package_share_directory("turtlebot3_description")

    world_path = os.path.join(pkg, "worlds", world)
    sdf = os.path.join(pkg, "models", "turtlebot3_burger", "model.sdf")
    nav2 = os.path.join(pkg, "params", p["params"])
    robot_desc = ParameterValue(
        Command(["xacro ", os.path.join(tb3, "urdf", "turtlebot3_burger.urdf"),
                 " use_sim_time:=true"]), value_type=str)

    # MPPI publishes on /cmd_vel; the filter takes it from /nav2_cmd_vel.
    ctrl_remap = [("/cmd_vel", "/nav2_cmd_vel")]
    # Filter output lands on /cmd_vel directly, or on /cmd_vel_safe when the
    # attacker forwards it to /cmd_vel.
    sf_out = "/cmd_vel_safe" if with_attack else "/cmd_vel"

    actions = [
        SetEnvironmentVariable(name="GAZEBO_MODEL_PATH",
                               value=os.path.join(pkg, "models") + ":" +
                               "/usr/share/gazebo-11/models"),
        SetEnvironmentVariable(name="GAZEBO_MODEL_DATABASE_URI", value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, "launch", "gzserver.launch.py")),
            launch_arguments={"world": world_path, "verbose": "false"}.items()),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-entity", "ugv", "-file", sdf,
                        "-x", g("spawn_x") or "0.0",
                        "-y", g("spawn_y") or "0.0", "-z", "0.01"],
             output="screen"),
        # Defender perception: limit LiDAR range fed to the filter (1.5 m).
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
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="map_to_odom_tf",
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
            Node(package="nav2_controller", executable="controller_server",
                 name="controller_server",
                 parameters=[nav2, {"use_sim_time": True}], remappings=ctrl_remap),
            Node(package="nav2_planner", executable="planner_server", name="planner_server",
                 parameters=[nav2, {"use_sim_time": True}]),
            Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
                 parameters=[nav2, {"use_sim_time": True}]),
            Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
                 parameters=[nav2, {"use_sim_time": True}]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_mgr_nav",
                 parameters=[{"use_sim_time": True, "autostart": True,
                              "node_names": ["controller_server", "planner_server",
                                             "behavior_server", "bt_navigator"],
                              "bond_timeout": 30.0}])]),
        # ---- Proposed RCBF safety filter ----
        TimerAction(period=16.0, actions=[
            Node(package="safety_filter", executable="proposed_v2_filter_node",
                 name="safety_filter_node",
                 parameters=[{
                     "use_sim_time": True,
                     "final_mode": True,
                     "goal_x": float(g("goal_x") or "7.5"),
                     "goal_y": float(g("goal_y") or "0.0"),
                     "v_safe_min": p["v_safe_min"],
                     "v_safe_max": p["v_safe_max"],
                     "w_safe_max": p["w_safe_max"],
                     "v_nom_max": p["v_nom_max"],
                     "a_v_max": p["sf_a_v_max"],
                     "a_w_max": p["sf_a_w_max"],
                     "alpha_h": ALPHA_H,
                     "w_v": W_V,
                     "w_w": W_W,
                     "lambda_delta": LAMBDA_DELTA,
                     "minrisk_tau": MINRISK_TAU,
                     "task_mode": TASK_MODE,
                     "max_scan_age": 0.50,
                     "d_geom": 0.01, "d_delay": 0.005, "d_hold": 0.005,
                 }],
                 remappings=[("/cmd_vel_nom", "/nav2_cmd_vel"),
                             ("/cmd_vel_safe", sf_out),
                             ("/scan", "/scan_defense_1p5")],
                 output="screen")]),
    ]

    # ---- optional post-filter actuator-command injection (attack demo) ----
    if with_attack:
        actions.append(TimerAction(period=18.0, actions=[
            Node(package="actuator_attack", executable="actuator_attack_node",
                 name="actuator_attack_node",
                 parameters=[{
                     "use_sim_time": True,
                     "attack_mode": "rollout_min_clearance",
                     "rho": 1.0,
                     "a_v_max": p["atk_a_v_max"],
                     "a_w_max": p["atk_a_w_max"],
                     "rollout_horizon": 24,
                     "rollout_dt": 0.05,
                     "rollout_margin": 0.03,
                     "trigger_clearance": 0.45,
                     "omega_eff_gain": 0.55,
                     "omega_eff_limit": 0.83,
                     "candidate_levels": 5,
                 }],
                 output="screen")]))

    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument("speed", default_value="low",
                              description="low | high (selects Nav2 params and SF authority)"),
        DeclareLaunchArgument("attack", default_value="false",
                              description="true inserts the post-filter actuator attack node"),
        DeclareLaunchArgument("world", default_value="auto_gate_trap.world"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("goal_x", default_value="7.5"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
