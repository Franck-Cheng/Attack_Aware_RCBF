# Source provenance and version correspondence

Everything in `src/` was copied from the machine that produced the formal
experiments. **No source file was edited except the three build/entry-point files
listed in §4.** Full checksums are in `SHA256SUMS_A_DWB.txt`.

## 1. Authoritative workspace

| Item | Value |
|---|---|
| Workspace root | `<A_workspace_root>` |
| Type | colcon ROS 2 workspace (`src/ build/ install/ log/`) |
| Packages used by the formal runs | `safety_filter`, `attack_safe_tb3_sim`, `actuator_attack`, `scan_range_limiter` |
| Launch entry used in every formal trial | `attack_safe_tb3_sim` / `proposed_v2_sf.launch.py` |
| Low-speed Nav2 config | `src/attack_safe_tb3_sim/params/nav2_params_v2.yaml` |
| High-speed Nav2 config | `config/controller/nav2_params_highspeed.FROZEN.yaml` |
| OS / ROS / Gazebo | Ubuntu 22.04.5 · ROS 2 Humble · Gazebo Classic 11.10.2 · Python 3.10.12 |

`<A_workspace_root>` **is** the workspace that ran the experiments: the
trial driver (`experiment/run_dwa_trial.py`) resolves every asset relative to it
and sources `<root>/install/setup.bash` before launching.

## 2. Other versions found on this machine (NOT used)

| Path | What it is | Why it is not the source |
|---|---|---|
| `<older_workspace_root>` | Older / unrelated workspace; `src/` has only `attack_safe_tb3_sim` + `waypoint_table_nav` | Contains **no** `safety_filter`, `actuator_attack` or `scan_range_limiter`, and no `proposed_v2_sf.launch.py`. Its `params/` only has comparison planners (`nav2_params_oracle.yaml`, `nav2_params_smac.yaml`, `nav2_params_theta.yaml`). |
| `~/.bashrc` | Sources `~/attack_safe_nav_ws/install/setup.bash` and sets `GAZEBO_MODEL_PATH` to that workspace's models | Leftover from earlier work. The formal launch overrides `GAZEBO_MODEL_PATH` with its own package path, so it did not affect the experiments. **Source this release last to avoid it.** |

### Robot SDF versions differ

`turtlebot3_burger/model.sdf` in the two workspaces is **not** the same file. The
release uses the `DWA_A_TRANSFER` copy (the one Gazebo actually spawned in the
formal trials):

| Parameter | `attack_safe_nav_ws` (old) | `DWA_A_TRANSFER` (used, released) |
|---|---:|---:|
| LiDAR `update_rate` | 5 Hz | **10 Hz** |
| LiDAR `range/max` | 2.0 m | **3.5 m** |
| `max_wheel_torque` | 20 | **200** |
| `max_wheel_acceleration` | 1.0 | **100.0** |

Collision geometry is identical in both: primitives only (box / cylinder /
sphere); meshes are visual-only.

## 3. File-by-file correspondence

Status legend: `identical` = byte-for-byte equal to the source;

| Release path | Original path (under `<A_workspace_root>`) | SHA256 (16) | Status |
|---|---|---|---|
| `src/safety_filter/safety_filter/proposed_v2_filter_node.py` | same | `03aa4eb1be7c8d7e` | identical |
| `src/safety_filter/safety_filter/proposed_v2.py` | same | `cafd633e7f68b155` | identical |
| `src/safety_filter/safety_filter/cbf_filter_node.py` | same | `b65e9b3bee79a696` | identical |
| `src/safety_filter/safety_filter/barrier_builder.py` | same | `c195eb38f14a947c` | identical |
| `src/safety_filter/safety_filter/safety_filter_core.py` | same | `d5c429247d25ee89` | identical |
| `src/safety_filter/safety_filter/geometry_utils.py` | same | `766dd2ce4d2a6a90` | identical |
| `src/safety_filter/safety_filter/final_robust.py` | same | `dbe6231879097b2b` | identical |
| `src/safety_filter/safety_filter/simple_perception.py` | same | `db9bda74ee85b760` | identical |
| `src/safety_filter/safety_filter/__init__.py` | same | `e3b0c44298fc1c14` | identical |
| `src/safety_filter/package.xml` | same | `05651986294f37c6` | identical |
| `src/safety_filter/setup.cfg` | same | `1c9e63f8277dbfd2` | identical |
| `src/safety_filter/resource/safety_filter` | same | `e3b0c44298fc1c14` | identical |
| `src/safety_filter/setup.py` | same | — | **modified** (§4) |
| `src/attack_safe_tb3_sim/launch/proposed_v2_sf.launch.py` | same | `76a3360182740930` | identical |
| `src/attack_safe_tb3_sim/params/nav2_params_v2.yaml` | `src/attack_safe_tb3_sim/params/nav2_params_v2.yaml` | `db1ecee5a20d6c25` | identical |
| `src/attack_safe_tb3_sim/params/nav2_params_highspeed.yaml` | `config/controller/nav2_params_highspeed.FROZEN.yaml` | `cc1db778734acd3f` | identical (renamed) |
| `src/attack_safe_tb3_sim/worlds/auto_gate_trap.world` | same | `93721b6a00ad530f` | identical |
| `src/attack_safe_tb3_sim/maps/blank_map.pgm` | same | `7981b45fc8c6391b` | identical |
| `src/attack_safe_tb3_sim/maps/blank_map.yaml` | same | `914b926e706e8880` | identical |
| `src/attack_safe_tb3_sim/models/turtlebot3_burger/model.sdf` | same | `c162eddc3c070b1c` | identical |
| `src/attack_safe_tb3_sim/models/turtlebot3_burger/model-1_4.sdf` | same | `40ca7f541a520cdf` | identical |
| `src/attack_safe_tb3_sim/models/turtlebot3_burger/model.config` | same | `c78e4ecdb4d0b843` | identical |
| `src/attack_safe_tb3_sim/package.xml` | same | `dfd4decf45773cb9` | identical |
| `src/attack_safe_tb3_sim/CMakeLists.txt` | same | — | **modified** (§4) |
| `src/actuator_attack/actuator_attack/actuator_attack_node.py` | same | `dc266e63a382ffb5` | identical |
| `src/actuator_attack/actuator_attack/__init__.py` | same | `e3b0c44298fc1c14` | identical |
| `src/actuator_attack/package.xml` | same | `c6d4dfea70b3ce31` | identical |
| `src/actuator_attack/setup.cfg` | same | `1445270118ba1930` | identical |
| `src/actuator_attack/resource/actuator_attack` | same | `e3b0c44298fc1c14` | identical |
| `src/actuator_attack/setup.py` | same | — | **modified** (§4) |
| `src/scan_range_limiter/scan_range_limiter/scan_range_limiter_node.py` | same | `0bffc29b47f53640` | identical |
| `src/scan_range_limiter/scan_range_limiter/__init__.py` | same | `e3b0c44298fc1c14` | identical |
| `src/scan_range_limiter/package.xml` | same | `d07f746412c648b9` | identical |
| `src/scan_range_limiter/setup.cfg` | same | `2acf966569ed8769` | identical |
| `src/scan_range_limiter/resource/scan_range_limiter` | same | `e3b0c44298fc1c14` | identical |

`e3b0c442…` is the SHA256 of an empty file (`__init__.py`, `resource/*` markers).

## 4. Release-time modifications (3 files, all non-algorithmic)

| File | Change | Reason |
|---|---|---|
| `src/safety_filter/setup.py` | `entry_points` reduced to `proposed_v2_filter_node` only | Drops the exports of Standard CBF (`generic_robust_cbf_node`), the legacy base node (`cbf_safety_filter_node`) and the P1 variant. The modules themselves are unchanged/absent; the Proposed code path is unaffected. |
| `src/actuator_attack/setup.py` | `entry_points` reduced to `actuator_attack_node` only | Drops `joint_effort_attack_node` (a different attack surface, unused by this chain). |
| `src/attack_safe_tb3_sim/CMakeLists.txt` | Removed the `install(DIRECTORY urdf …)`, `install(DIRECTORY scripts …)` and `install(PROGRAMS scripts/relay_node.py …)` rules | `urdf/` and `scripts/` are not shipped and not used by this launch (the URDF comes from the installed `turtlebot3_description` package). Without this the install step fails on missing directories. |

Renamed only: `nav2_params_highspeed.FROZEN.yaml` → `nav2_params_highspeed.yaml`
(located under `params/` so the launch can point at it). Content unchanged.

## 5. Deliberately excluded

| Excluded | Why |
|---|---|
| `safety_filter/safety_filter/generic_robust_cbf_node.py` | Standard-CBF comparison node. |
| `safety_filter/safety_filter/p1_cbf_filter_node.py`, `p1_robust.py` | P1 ablation variant, not part of Proposed. |
| `actuator_attack/actuator_attack/joint_effort_attack_node.py` | Joint-effort attack surface, unused by the `/cmd_vel` chain. |
| `attack_safe_tb3_sim/launch/{baseline0_nav2_only, final_robust_sf_v2, attack_nav_v2}.launch.py` | Baseline and Standard-CBF launches. |
| `attack_safe_tb3_sim/worlds/final_H1..H5*.world`, `holdout_obs.json` | Formal test environments and evaluator obstacle table. |
| `attack_safe_tb3_sim/urdf/*`, `scripts/relay_node.py`, `params/` extras | Not referenced by the Proposed launch. |
| `experiment/` (batch driver, recorder, evaluator, conditions) | Experiment orchestration and metrics; the batch also contains A01–A20 matrices, baselines and comparison methods. |
| `config/`, `results/`, `docs/`, `build/`, `install/`, `log/` | Protocol definitions, trial records, traces, statistics, caches. |

`final_robust.py` **is** kept: it is not the Standard-CBF node but the shared
helper module (world-lock `scan_points_world_at_ts`, `barrier_rows_from_worldpoints`,
degradation state constants) imported by both `cbf_filter_node.py` and
`proposed_v2_filter_node.py`. Removing it breaks the Proposed filter.

## 6. Verification performed on this release

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# → Summary: 4 packages finished [8.94s]
```

| Check | Command | Outcome |
|---|---|---|
| Low launch | `ros2 launch attack_safe_tb3_sim proposed_v2_sf.launch.py use_sim_time:=true world:=auto_gate_trap.world attack_mode:=zero` | all nodes up; `gzserver` running the release world |
| Low goal | `ros2 action send_goal /navigate_to_pose … (7.5, 0)` | `SUCCEEDED` |
| Low filter | `ros2 topic echo /filter_status` | `V2_STRICT` at ~10 Hz |
| High params | `ros2 param get /safety_filter_node {v_safe_max,w_safe_max,v_nom_max}` | `0.385 / 1.76 / 0.35` |
| High DWB | `ros2 param get /controller_server FollowPath.max_vel_x/max_vel_theta` | `0.35 / 1.6` |
| High goal | same goal on domain 78 | `SUCCEEDED` |

Filter parameters confirmed at runtime: `alpha_h=2.3`, `w_v=20.420091`,
`w_w=0.379909`, `lambda_delta=10.0`, `minrisk_tau=1e-09`, `task_mode=not_reverse`.

## 7. Known gaps / unverified items

1. **`SOFT` and `MIN_RISK` were not triggered in the demo world** during the smoke
   run — the demo scenario is benign enough that the filter stayed in `STRICT`.
   Both tiers exist in `proposed_v2.py` (`three_tier`) and were exercised in the
   formal experiments, but this release ships **no unit test** for them.
2. **Attack injection was only smoke-tested with `attack_mode:=zero`.**
   `rollout_min_clearance` (the formal attack) is present and is the launch
   default, but was not re-run here.
3. **Robot visual meshes** (`model://turtlebot3_common`) are not on the launch's
   `GAZEBO_MODEL_PATH`, so Gazebo warns about missing meshes. Physics and the
   filter are unaffected (collision geometry is primitive-based). This matches
   the original experiment behaviour; it was left untouched for fidelity.
4. **TF is briefly unavailable at filter start-up** (`odom` frame not yet
   published), producing a few `DEGRADED_SENSOR` warnings right after t≈16 s.
   The filter recovers automatically once the diff-drive plugin publishes TF.
   This is original behaviour, not introduced by the release.
5. **`~/.bashrc` on this machine sources another workspace**; nothing in the
   release depends on it, but a fresh shell must source this workspace last.
6. No `LICENSE` file ships here — the original packages carry placeholder
   `TODO: License declaration` fields. To be decided by the repository owner.
