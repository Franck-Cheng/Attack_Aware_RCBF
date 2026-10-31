# Integrating the DWB (Computer A) packages — record and procedure

Status: **done (2026-10-30)**. The DWB delivery was received and merged. This document keeps
the procedure that was followed so a future merge can be reproduced.

## Outcome of the 2026-10-30 merge

| Check | Result |
|---|---|
| Delivery | `rcbf_release_A` (40 files) at `<A_delivery_dir>` |
| Proposed core, 9 modules | **byte-identical** to the MPPI release → one shared `safety_filter` package |
| `actuator_attack_node.py`, `scan_range_limiter_node.py` | byte-identical |
| `auto_gate_trap.world`, `turtlebot3_burger/model.sdf`, `blank_map.*` | byte-identical |
| Differing files | only build metadata (`setup.py`, `package.xml`, `CMakeLists.txt`) and the launch / Nav2-config sets |
| Build | 4/4 packages |
| DWB low / DWB high | launch + goal + `/filter_status = V2_STRICT` verified (`docs/VERIFICATION.md`) |

Nothing was overwritten: A's `launch/proposed_v2_sf.launch.py`, `params/nav2_params_v2.yaml`
and `params/nav2_params_highspeed.yaml` were added **verbatim** next to the MPPI launch and
configs, and the shared modules exist once. Because the cores proved identical, a second
implementation was not needed.

---

## Procedure (as followed)

## 1. Intake

1. Obtain the archive (e.g. `rcbf_release_A.tar.gz`) and its published SHA256.
2. Verify integrity **before** unpacking into the tree:

   ```bash
   sha256sum rcbf_release_A.tar.gz          # compare with the value supplied by A
   mkdir -p /tmp/rcbf_A_check && tar -xzf rcbf_release_A.tar.gz -C /tmp/rcbf_A_check
   find /tmp/rcbf_A_check -type f | sort    # compare against the declared file list
   ```

3. Record the file list and hashes in `docs/VERIFICATION.md` (append a section
   "DWB intake") so the merged state is traceable.

## 2. Compare the Proposed core before merging

The Proposed filter may be identical between the two machines or may have diverged.
**Never overwrite B's filter with A's on the assumption that they match.**

```bash
for f in proposed_v2.py proposed_v2_filter_node.py cbf_filter_node.py \
         final_robust.py barrier_builder.py geometry_utils.py \
         simple_perception.py safety_filter_core.py; do
  sha256sum src/safety_filter/safety_filter/$f /tmp/rcbf_A_check/src/safety_filter/safety_filter/$f
done
```

Decision rule:

| Outcome | Action |
|---|---|
| All hashes equal | Keep one shared `safety_filter` package; note "identical to A" in `docs/VERIFICATION.md`. |
| Differ only in comments / formatting | Diff to confirm semantics identical; keep one package, record the diff. |
| Differ in solver, constraints, bounds or tier logic | **Keep both implementations** as separate packages (e.g. `safety_filter` for the shared core and `safety_filter_dwb` for A's variant), document the semantic differences, and state which one each launch uses. |

A's filter must never silently replace B's: the two entry points must be distinguishable and
each launch must name the executable it actually starts.

## 3. Merge layout

* DWB packages go under `src/` next to the existing ones; they must not shadow
  `safety_filter`, `actuator_attack`, `scan_range_limiter` or `attack_safe_tb3_sim`.
* If A ships its own world/model assets, keep them inside A's package; do not overwrite
  `attack_safe_tb3_sim`.
* If A ships its own `/cmd_vel` remap convention, keep it local to A's launch file.
* No experiment data, schedulers, batch configs or logs are merged — only what is needed to
  run the Proposed filter with DWB at the two speeds.

## 4. Acceptance checks for each DWB mode

All four must pass before the README table is updated:

1. `colcon build` succeeds for the merged workspace (no duplicate package names).
2. `ros2 launch <A launch> speed:=low|high` brings the node graph up:
   `controller_server`, `safety_filter_node`, `gazebo` all present.
3. The filter is genuinely in the path — for the DWB modes the analogue of B's check is:

   ```
   ros2 topic info -v <nominal topic>   # publisher = controller_server, subscriber = safety_filter_node
   ros2 topic info -v /cmd_vel          # publisher  = safety_filter_node (+ recovery behaviours)
   ```

4. `/filter_status` publishes `V2_STRICT` (and, during a run, `V2_SOFT` / `V2_MIN_RISK`
   when the corresponding conditions occur).

Only after 1–4 may the README rows be changed from
"not available in this snapshot" to "build + runtime verified", with the exact command and
the date recorded in `docs/VERIFICATION.md`.

## 5. Explicitly out of scope

Merging A/B experiment result sets, comparative statistics, schedulers, hold-out worlds and
per-trial records. This repository ships the Proposed implementation only.
