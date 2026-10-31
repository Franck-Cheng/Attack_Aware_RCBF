# Source provenance — B (MPPI) contribution

Everything under `src/` that is shared with the DWB release is **byte-identical** to the
frozen modules that produced the MPPI experiments. Only build metadata and the launch /
Nav2-config files are contribution-specific.

## 1. Authoritative workspace

| Item | Value |
|---|---|
| Workspace root | `<B_workspace_root>` |
| Packages | `safety_filter`, `attack_safe_tb3_sim`, `actuator_attack`, `scan_range_limiter` |
| Campaign that used this code | `experiments/final/mppi_B01_B20_formal_120` (Nav2/MPPI, groups B01–B20) |
| Launch entry used in every MPPI trial | `mppi_speed_smoke/mppi_chain.launch.py` (`chain:=proposed`) |
| Low-speed Nav2 config | `mppi_speed_smoke/nav2_params_mppi_low.yaml` |
| High-speed Nav2 config | `mppi_speed_smoke/nav2_params_mppi_high.yaml` |
| OS / ROS / Gazebo | Ubuntu 22.04 · ROS 2 Humble · Gazebo Classic 11 · Python 3.10 |

The released launch `src/attack_safe_tb3_sim/launch/mppi_proposed.launch.py` is a
**re-write of the Proposed branch only** of `mppi_chain.launch.py`: the baseline /
Standard-CBF / `cbf_attack` branches and the diagnostic side-node of the experiment
launcher are removed; the node set, timers, remaps and parameter values of the Proposed
path are preserved.

## 2. File correspondence

| Release path | SHA256 (16) | Status |
|---|---|---|
| `src/safety_filter/safety_filter/proposed_v2.py` | `cafd633e7f68b155` | identical to experiment |
| `src/safety_filter/safety_filter/proposed_v2_filter_node.py` | `03aa4eb1be7c8d7e` | identical |
| `src/safety_filter/safety_filter/cbf_filter_node.py` | `b65e9b3bee79a696` | identical |
| `src/safety_filter/safety_filter/final_robust.py` | `dbe6231879097b2b` | identical |
| `src/safety_filter/safety_filter/barrier_builder.py` | `c195eb38f14a947c` | identical |
| `src/safety_filter/safety_filter/geometry_utils.py` | `766dd2ce4d2a6a90` | identical |
| `src/safety_filter/safety_filter/simple_perception.py` | `db9bda74ee85b760` | identical |
| `src/safety_filter/safety_filter/safety_filter_core.py` | `d5c429247d25ee89` | identical |
| `src/safety_filter/safety_filter/__init__.py` | `e3b0c44298fc1c14` | empty |
| `src/actuator_attack/.../actuator_attack_node.py` | `dc266e63a382ffb5` | identical |
| `src/scan_range_limiter/.../scan_range_limiter_node.py` | `0bffc29b47f53640` | identical |
| `src/attack_safe_tb3_sim/params/nav2_params_mppi_low.yaml` | `e28039e679afa903` | identical |
| `src/attack_safe_tb3_sim/params/nav2_params_mppi_high.yaml` | `7f1e2bb9afd7dbda` | identical |
| `src/attack_safe_tb3_sim/worlds/auto_gate_trap.world` | `93721b6a00ad530f` | identical |
| `src/attack_safe_tb3_sim/models/turtlebot3_burger/model.sdf` | `c162eddc3c070b1c` | identical |
| `src/attack_safe_tb3_sim/launch/mppi_proposed.launch.py` | `63b237ee65a7cbac` | **new** (Proposed branch only) |
| `src/safety_filter/setup.py`, `package.xml` | — | **rewritten** (entry points + deps, see §3) |
| `src/actuator_attack/setup.py` | — | **rewritten** (single entry point) |
| `src/attack_safe_tb3_sim/CMakeLists.txt`, `package.xml` | — | **rewritten** (trimmed install rules, added deps) |

## 3. Release-time modifications (non-algorithmic)

| File | Change | Reason |
|---|---|---|
| `safety_filter/setup.py` | only `proposed_v2_filter_node` exported | removes the Standard-CBF / generic / P1 console scripts |
| `safety_filter/package.xml` | description, maintainer, `python3-numpy` / `python3-scipy` exec deps | packaging hygiene |
| `actuator_attack/setup.py` | only `actuator_attack_node` exported | `joint_effort_attack_node` is a different attack surface, not shipped |
| `attack_safe_tb3_sim/CMakeLists.txt` | dropped `urdf/` and `scripts/` install rules | neither directory is shipped; the URDF comes from `turtlebot3_description` |
| `attack_safe_tb3_sim/package.xml` | added nav2/`dwb_core`/gazebo exec deps | declares the runtime requirements of both launches |
| `setup.cfg` (all three python packages) | kept | required so console scripts install to `lib/<pkg>/`; without it `ros2 launch` fails |

## 4. Deliberately excluded

`generic_robust_cbf_node.py` (Standard CBF), `p1_robust.py` / `p1_cbf_filter_node.py`
(P1 ablation), `joint_effort_attack_node.py`, every `final_*` / `baseline*` /
`attack_nav*` launch, the H1–H5 formal worlds and `holdout_obs.json`, the batch
scheduler, trial records, `qp_diag` streams, logs and parameter dumps.

## 5. Relation to the DWB contribution

The eight Proposed core modules and the two actuator/perception nodes are **byte-identical**
to the DWB release (`docs/SOURCE_PROVENANCE_A_DWB.md`). A single shared `safety_filter`
package is therefore shipped; the two contributions differ only in their launch entry and
Nav2 configuration files.
