# Verification log — MPPI modes

Environment: ROS 2 Humble, Gazebo Classic 11, Python 3.10, headless (`gzserver` only),
`numpy 1.21.5`, `scipy 1.8.0`.

All checks below were executed in an isolated workspace
(`/tmp/rcbf_ws_b`, `src` → this repository's `src/`), so nothing in the
experiment workspaces was modified.

## 1. Build

```
$ colcon build --symlink-install
Summary: 4 packages finished [4.23s]
```

Console scripts resolved to `lib/<pkg>/`:

```
safety_filter        proposed_v2_filter_node
actuator_attack      actuator_attack_node
scan_range_limiter   scan_range_limiter_node
```

Note: `setup.cfg` is required (`install_scripts=$base/lib/<pkg>`); without it colcon installs
the scripts under `bin/` and `ros2 launch` fails with *"libexec directory does not exist"*.

## 2. MPPI low speed (`speed:=low`, attack off)

Node graph (after a goal was sent):

```
/safety_filter_node
/controller_server
/gazebo
```

Topic wiring:

```
/nav2_cmd_vel : publisher  controller_server
                subscriber safety_filter_node
/cmd_vel      : publisher  safety_filter_node (+ behavior_server recovery)
                subscriber 1  (gazebo diff_drive)
/scan_defense_1p5 : publisher scan_range_limiter, subscriber safety_filter_node, local_costmap
```

Filter behaviour:

```
/filter_status                -> data: V2_STRICT
/final log                    -> "[PROPOSED-V2] three-tier filter enabled | lambda_delta=10.0 |
                                 tau=1e-09 | task_row=NOT_REVERSE(u_nav) | MIN_RISK=lexicographic_2stage"
U_safe banner                 -> v∈[-0.242, 0.242], ω∈[-1.100, 1.100]
assumed attack banner         -> a_v∈[-0.13, 0.13], a_w∈[-0.60, 0.60]   (beta = 1.0)
```

The filter output differed from the nominal command (e.g. `/nav2_cmd_vel` linear.x 0.2122
vs plant input 0.1802 sampled at different instants), confirming the filter is in the loop
rather than bypassed.

## 3. MPPI high speed + attack (`speed:=high attack:=true`)

Filter banners:

```
U_safe: v∈[-0.242, 0.385], ω∈[-1.760, 1.760]
Attack: a_v∈[-0.13, 0.13], a_w∈[-0.60, 0.60]
[PROPOSED-V2] three-tier filter enabled ...
```

Topics present: `/nav2_cmd_vel`, `/cmd_vel_safe`, `/attack_signal`, `/cmd_vel`, `/filter_status`.

All three tiers exercised during a single short run (log lines, `anom_total=0` throughout):

```
stage=V2_STRICT
stage=V2_SOFT     reason=strict_infeasible_soft_feasible
stage=V2_MIN_RISK reason=safety_infeasible_minrisk  eta=-0.0228
```

Robot advanced under Nav2 (odom x reached 1.707 m from a spawn at x=0).

## 4. DWB low speed (`proposed_v2_sf.launch.py`, `attack_mode:=zero`)

```bash
ros2 launch attack_safe_tb3_sim proposed_v2_sf.launch.py \
  use_sim_time:=true world:=auto_gate_trap.world attack_mode:=zero
```

* `/filter_status` → `V2_STRICT`
* filter banners: `U_safe: v∈[-0.242, 0.242], ω∈[-1.100, 1.100]`; assumed attack `a_v∈[-0.13, 0.13], a_w∈[-0.60, 0.60]`
* `/nav2_cmd_vel`: publisher `controller_server` (+ `behavior_server` recoveries), subscriber `safety_filter_node`
* `/cmd_vel_safe`: publisher `safety_filter_node`, subscriber `actuator_attack_node`
* `/cmd_vel`: publisher `actuator_attack_node`, subscriber `turtlebot3_diff_drive`
* controller really is DWB: `FollowPath.max_vel_x=0.22`, `max_vel_theta=1.0`, `min_vel_x=-0.22`, `sim_time=1.5`
* robot advanced under Nav2 after the goal (odom x reached 3.42 m)

## 5. DWB high speed (`nav2_params_highspeed.yaml`)

```bash
SHARE=$(ros2 pkg prefix attack_safe_tb3_sim)/share/attack_safe_tb3_sim
ros2 launch attack_safe_tb3_sim proposed_v2_sf.launch.py \
  use_sim_time:=true world:=auto_gate_trap.world attack_mode:=zero \
  nav2_params_file:=$SHARE/params/nav2_params_highspeed.yaml \
  v_safe_max:=0.385 w_safe_max:=1.76 v_nom_max:=0.35
```

* filter banners: `U_safe: v∈[-0.242, 0.385], ω∈[-1.760, 1.760]`
* `/filter_status` → `V2_STRICT`
* `FollowPath.max_vel_x=0.35`, `max_vel_theta=1.6`, `min_vel_x=-0.22`
* SF authority read back: `v_safe_max=0.385`, `w_safe_max=1.76`, `v_nom_max=0.35`
* `/nav2_cmd_vel`: `controller_server` → `safety_filter_node`; `/cmd_vel_safe` → `actuator_attack_node`
* robot advanced under Nav2 after the goal (odom x reached 1.55 m)

## 6. Observed startup caveat

For roughly the first second after the filter node starts, TF lookups for `odom` may fail
because the TF buffer has not yet been populated at the scan stamp; the node logs
`world-lock: TF2 unavailable/failed -> DEGRADED_SENSOR`. It recovers automatically once the
buffer fills (all subsequent cycles reported `V2_*` tiers). This is the intended
`DEGRADED_SENSOR` fallback, not a failure of the release build.

## 7. What is NOT verified

* No mission-level outcome, collision, clearance or timing statistics are claimed.
* Verification is a single short run per mode on one machine; not a soak test.
* `V2_SOFT` / `V2_MIN_RISK` were exercised only in the MPPI high + attack run; both DWB runs
  stayed in `V2_STRICT` (benign demo scenario).
* The DWB modes were driven with `attack_mode:=zero`; the DWB launch's default
  `rollout_min_clearance` attack was not re-run here.
* The `u_act = u_sf + a` identity was not re-derived numerically for this snapshot
  (the attack node publishes `u_sf + a` to `/cmd_vel` by construction).
