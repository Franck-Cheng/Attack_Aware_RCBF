# Experimental Protocol, Terrain Layouts, Parameters, Evaluation Criteria, and Results

This directory provides the experimental data for **Attack-Aware Robust Control Barrier Function Safety Filtering for Mobile Robot Navigation under Post-Filter Actuator-Command Injection Attacks**. It documents the simulation protocol, five static obstacle-layout families, attack and controller settings, terminal-outcome criteria, and all **4,000** published trial records.

**Data packages:** `resultA.zip` (DWB) and `resultB.zip` (MPPI). Each archive contains 20 controller-specific groups of 100 valid trials, including trial-level metrics and, where recorded, time-series process traces.

## 1. Data packages and extraction

| Archive | Nominal controller | Groups | Trial rows | Per-trial traces |
|:--|:--|:--|--:|:--|
| [`resultA.zip`](resultA.zip) | DWB | A01–A20 | 2,000 | `process/A01/`–`process/A20/` |
| [`resultB.zip`](resultB.zip) | MPPI | B01–B20 | 2,000 | `process/B01/`–`process/B20/` |

The archives, rather than pre-extracted folders, are stored in this GitHub directory. Their **logical layout after extraction** is:

```text
experiment_result/
├── README.md
├── resultA.zip
├── resultB.zip
├── resultA/
│   ├── trials.csv                 # 2,000 trial rows, 15 fields
│   └── process/
│       ├── A01/                   # 100 CSV files named <trial_id>.csv
│       ├── ...
│       └── A20/
└── resultB/
    ├── trials.csv                 # 2,000 trial rows, 15 fields
    └── process/
        ├── B01/
        ├── ...
        └── B20/
```

A ZIP may already contain its top-level `resultA/` or `resultB/` directory; check the archive entry paths before choosing an extraction destination. `trials.csv` lists terminal trial metrics; `process/<group_id>/<trial_id>.csv` records that trial's available time-stamped signals. The CSV process filename matches the master table's `trial_id`.

## 2. Experimental design

### 2.1 Objective and controlled comparison

The experiment evaluates navigation **completion and collision outcomes under downstream actuator-command injection**. A DWB or MPPI nominal controller generates linear and angular velocity commands. Depending on the test condition, the commands either pass through no filter, a conventional CBF-QP, or the proposed attack-aware RCBF filter. The attacker modifies commands **after** the filtering stage:

```text
DWB / MPPI nominal command
              |
              v
   No filter / CBF / Proposed RCBF
              |
              v
   Post-filter command injection
              |
              v
          Gazebo robot
```

Both nominal controllers are evaluated with the **same robot, the same speed modes, the same attack parameter settings, and the same safety-filter insertion point**. The safety filter's velocity-command bounds equal the corresponding nominal controller's bounds in each speed mode. The same downstream injection location is used in all three methods.

| Method in paper | Original `filter_kind` | Safety-filter operation |
|:--|:--|:--|
| **Nominal** | `none` | No safety filter; the nominal command goes through the downstream attack channel. |
| **CBF** | `standard_cbf` | Conventional single-stage CBF-QP with a grid-search max–min fallback. |
| **Proposed RCBF** | `proposed` | Attack-aware CBF constraints with STRICT–SOFT–MIN_RISK hierarchical optimization. |

The `standard_cbf` string is retained in raw CSV files for compatibility. It denotes **CBF** in the paper, not an additional method. These experiments compare the complete implemented filters: Proposed RCBF changes both the attack-aware barrier constraints and feasibility handling relative to the CBF baseline.

### 2.2 Factorial grouping and sample counts

There are **20 experimental conditions** common to both controllers. Each condition is executed for **100 valid trials with DWB** and another **100 valid trials with MPPI**, resulting in **40 controller-specific groups** and **4,000 valid trials**. Each controller-specific group is balanced across five fixed obstacle layouts (**20 trials per layout**).

$$
20\ \text{conditions}\times2\ \text{controllers}\times5\ \text{layouts}\times20\ \text{trials/layout}=4{,}000\ \text{trials}.
$$

A condition number specifies the same speed, filter method, actual attack strength, and (where applicable) robust design strength for both controllers. For example, **A08** and **B08** represent the same condition tested with DWB and MPPI, respectively.

| Condition suffixes | Speed mode | Actual attack $\gamma_a$ | Conditions in order |
|:--|:--|--:|:--|
| 01–04 | Low | 0 | Nominal; CBF; Proposed ($\gamma_d=0.3$); Proposed ($\gamma_d=0.6$) |
| 05–07 | Low | 0.3 | Nominal; CBF; Proposed ($\gamma_d=0.3$) |
| 08–10 | Low | 0.6 | Nominal; CBF; Proposed ($\gamma_d=0.6$) |
| 11–14 | High | 0 | Nominal; CBF; Proposed ($\gamma_d=0.3$); Proposed ($\gamma_d=0.6$) |
| 15–17 | High | 0.3 | Nominal; CBF; Proposed ($\gamma_d=0.3$) |
| 18–20 | High | 0.6 | Nominal; CBF; Proposed ($\gamma_d=0.6$) |

For attacked Proposed conditions, the design strength equals the applied attack strength ($\gamma_d=\gamma_a$), so each attack level has one Proposed design setting.

The zero-attack conditions characterize navigation behavior without realized injection. The attacked conditions test two attack magnitudes ($\gamma_a=0.3$ and $0.6$). There are **800 attacked trials per method**, or 2,400 attacked trials in total, plus 1,600 zero-attack reference trials.

## 3. Terrain and obstacle-layout design

The evaluation uses **five fixed Gazebo environments with static obstacles**, representing distinct navigation geometries. These are **obstacle-layout types**, not variations in ground elevation. Each condition runs 20 trials in each layout, so the five layouts are equally represented within every method, speed mode, attack strength, and controller. The data preserves the original `map_id` strings:

| `map_id` in released CSV | Layout family | Intended navigation challenge | Number of obstacles |
|:--|:--|:--|--:|
| `H1_chicane` | Chicane | Successive changes in travel direction around offset obstacles | 14 |
| `H2_bottleneck` | Bottleneck | Tight passage and clearance-sensitive navigation | 11 |
| `H3_mixed` | Mixed | Combined obstacle arrangements and turning demands | 15 |
| `H4_opencon` | Open-connected | Connected traversable space with alternative passage choices | 12 |
| `H5_multiswitch` | Multi-switch | Repeated changes between obstacle-constrained routes | 18 |

The obstacle counts reflect the saved formal evaluation geometry for these five layouts. The descriptions summarize the respective layout **categories**; exact obstacle positions, radii, and passage widths are determined by the Gazebo world and obstacle-geometry files, not by these names. The trial archives store which environment was used for each run in `map_id`.

**Layout allocation per controller-specific group:** 20 trials × five layouts = 100. Consequently, across the 40 groups the dataset contains **800 trials per layout**. The maps are fixed scenario families, not 100 newly generated random maps per group.

**Geometry needed for exact reproduction:** the five formal evaluation layouts are `final_H1_chicane.world` through `final_H5_multiswitch.world`, with obstacle geometry defined in the corresponding `holdout_obs.json` files. These assets, together with the physical-collision evaluator and the trial-runner configuration, accompany the runnable release.

## 4. Experimental parameter configuration

### 4.1 Simulation and navigation

| Parameter | Setting |
|:--|:--|
| Simulation environment | ROS 2, Nav2, Gazebo |
| Robot motion model | Differential-drive/unicycle; linear $v$ and angular $\omega$ commands |
| Nominal controllers | DWB and MPPI |
| Safety-filter execution frequency | 10 Hz |
| Low-speed nominal linear-velocity limit | 0.22 m/s |
| High-speed nominal linear-velocity limit | 0.35 m/s |
| Filter command authority | Identical to the nominal controller bounds in each speed mode: $v\in[-0.22,0.22]$ m/s, $\omega\in[-1.0,1.0]$ rad/s at low speed; $v\in[-0.35,0.35]$ m/s, $\omega\in[-1.6,1.6]$ rad/s at high speed |
| Evaluation start | Goal-acceptance timestamp (simulation-time origin) |
| Navigation budget | 150 s of simulation time per valid trial |
| Goal-position tolerance | 0.6 m |
| Environment sampling | 5 fixed obstacle layouts × 20 trials/layout/group |
| Per-group trials | 100 valid trials |

The 150 s budget and 0.6 m goal tolerance are the recorded formal-trial evaluation settings. Distances and times in the archive are in meters and seconds, respectively.

### 4.2 Attack model and strengths

The attack perturbs the **post-filter** command in both channels:

$$
\begin{bmatrix}v_{\rm act}\\ \omega_{\rm act}\end{bmatrix}
=
\begin{bmatrix}v_{\rm sf}\\ \omega_{\rm sf}\end{bmatrix}
+
\begin{bmatrix}a_v\\a_\omega\end{bmatrix},
\quad |a_v|\le \bar a_v,\quad |a_\omega|\le \bar a_\omega.
$$

Attack amplitude and the robust filter's design envelope are independently specified by:

$$
(\bar a_v,\bar a_\omega)=\gamma_a(V_s,\Omega_s),\qquad
(\hat a_v,\hat a_\omega)=\gamma_d(V_s,\Omega_s).
$$

| Speed mode | Linear scale $V_s$ | Angular scale $\Omega_s$ |
|:--|--:|--:|
| Low | 0.22 m/s | 1.0 rad/s |
| High | 0.35 m/s | 1.6 rad/s |

| Speed | Actual strength $\gamma_a$ | $\bar a_v$ (m/s) | $\bar a_\omega$ (rad/s) |
|:--|--:|--:|--:|
| Low | 0 | 0 | 0 |
| Low | 0.3 | 0.066 | 0.30 |
| Low | 0.6 | 0.132 | 0.60 |
| High | 0 | 0 | 0 |
| High | 0.3 | 0.105 | 0.48 |
| High | 0.6 | 0.210 | 0.96 |

The adversary follows a **rollout-minimum-clearance** policy: it evaluates admissible candidate injections over a short forward rollout and selects the candidate with the smallest predicted obstacle clearance. This is a directed clearance-minimizing attack rather than random noise. The instantaneous chosen injection is downstream of the defender; the Proposed RCBF uses the design envelope $\widehat{\mathcal A}$.

For attacked Proposed RCBF trials, **$\gamma_d=\gamma_a$**. For zero-injection Proposed trials, two design strengths are independently tested: **$\gamma_d=0.3$ and $\gamma_d=0.6$**. The CSV column `gamma_attack` records the actual-attack strength setting, not the design strength; infer $\gamma_d$ from the condition table above.

### 4.3 Safety geometry and optimization

The robot body is approximated by three overlapping disks, centered at $(+0.08,0)$, $(0,0)$, and $(-0.08,0)$ m in robot coordinates. Each disk has radius $0.105$ m. For obstacle point/object $i$ and body disk $j$, the inflated clearance is

$$
h_{ij}=\|p+R(\theta)c_j-o_i\|-(r_i+\rho+d_{\rm safe}+d_{\rm point}).
$$

The Proposed RCBF enforces the affine worst-case margin (when the active safety set is feasible):

$$
\ell_{ij,v}v+\ell_{ij,\omega}\omega+\alpha h_{ij}
-|\ell_{ij,v}|\hat a_v-|\ell_{ij,\omega}|\hat a_\omega\ge 0.
$$

| Parameter | Value |
|:--|:--|
| Three disk center offsets | $(+0.08,0)$, $(0,0)$, $(-0.08,0)$ m |
| Disk radius $\rho$ | 0.105 m |
| Geometric allowance $d_{\rm safe}$ | 0.020 m |
| Point-observation allowance $d_{\rm point}$ | 0.025 m |
| Barrier gain $\alpha$ | 1.5 |
| Quadratic command-deviation matrix $H$ | $\mathrm{diag}(20,0.8)$ |
| SOFT task-slack penalty $\lambda_\delta$ | 10 |
| MIN_RISK second-stage residual tolerance $\tau$ | $10^{-9}$ |
| Proposed hierarchy | STRICT → SOFT → MIN_RISK |
| CBF baseline | Single-stage CBF-QP with a grid-search max–min fallback; no STRICT–SOFT–MIN_RISK hierarchy |

STRICT enforces hard robust obstacle constraints and the signed-motion task constraint; SOFT retains hard safety and relaxes the task constraint; MIN_RISK maximizes the minimum robust residual when the hard safety set is infeasible and then chooses a nominal-command-near solution.

## 5. Outcome definitions and evaluation metrics

### 5.1 Terminal outcomes

Each valid trial is assigned **exactly one** of three mutually exclusive labels:

| Outcome | Formal evaluation criterion | Recorded interpretation |
|:--|:--|:--|
| **Success** | The robot reaches the goal region (position error < 0.6 m) within the 150 s simulation-time budget, without a recorded collision. | Goal completion. |
| **Collision** | Robot-body physical collision geometry intersects an obstacle during the evaluated navigation trial. | Unsafe physical overlap/contact according to the geometric evaluator. |
| **Timeout** | The 150 s simulation-time budget expires without a recorded success or collision terminal event. | Navigation incomplete at the horizon; **not** counted as a collision. |

The collision label is based on the trial's **physical-footprint geometry/evaluation**, not directly on the three-disk RCBF constraint $h_{ij}$ or on the sign of `min_clearance_m` alone. The original formal-run implementation evaluates planar robot collision geometry against the corresponding static obstacle geometry. A negative signed clearance indicates overlap in the represented geometry. Published collision counts use the archived trial outcome.

Navigation timing is measured from the **goal-acceptance simulation timestamp**. A trial reaches its evaluation deadline at $t_0+150$ s. The process stream timestamps `t_rel_s` are also relative to goal acceptance. The three labels partition the 100 valid records in each group.

### 5.2 Reported metrics

For a group of $N=100$ valid trials:

$$
\text{Success rate}=100\frac{N_{\rm success}}{N}\%,\quad
\text{Collision rate}=100\frac{N_{\rm collision}}{N}\%,\quad
\text{Timeout rate}=100\frac{N_{\rm timeout}}{N}\%.
$$

Because $N=100$, a group count is numerically equal to its percentage (for example, 82 successes = 82%). All three terminal counts sum to 100.

| Metric / raw field | Definition and how to interpret it |
|:--|:--|
| `nav_time_s` | Navigation duration in seconds, measured relative to the trial's navigation start. |
| **Median successful-trial time** | Median `nav_time_s` taken **only over successful trials** in a group; never a median over all 100 outcomes. |
| `path_length_m` | Traveled path length of the robot in meters. |
| `min_clearance_m` | Minimum signed physical/geometric clearance recorded over the run; a negative value indicates overlap in the represented geometry. |
| `final_goal_distance_m` | Distance from the robot to the goal when the trial terminates. |
| `sf_strict_frac`, `sf_soft_frac`, `sf_minrisk_frac` | Fractions of recorded proposed-filter cycles operating in the respective optimization mode; `NA` for the other methods. |

Median times are computed over each method's own successful trials and should be read together with the success counts. The main performance comparison reports **success, collision, and timeout separately**.

## 6. Experimental results

### Aggregate results under attack

Each method has **800 attacked trials**: 400 at $\gamma_a=0.3$ and 400 at $\gamma_a=0.6$.

| Attack $\gamma_a$ | Method | Success | Collision | Timeout |
|:--|:--|--:|--:|--:|
| 0.3 | Nominal | 318 | 14 | 68 |
| 0.3 | CBF | 339 | 0 | 61 |
| 0.3 | Proposed RCBF | 360 | 0 | 40 |
| 0.6 | Nominal | 119 | 128 | 153 |
| 0.6 | CBF | 139 | 15 | 246 |
| 0.6 | Proposed RCBF | 183 | 0 | 217 |
| **All attacked** | Nominal | 437 | 142 | 221 |
| **All attacked** | CBF | 478 | 15 | 307 |
| **All attacked** | **Proposed RCBF** | 543 | 0 | 257 |

Proposed RCBF achieves **543 successes and zero collisions** in 800 attacked trials, compared with **478 successes and 15 collisions** for CBF and **437 successes and 142 collisions** for Nominal. The comparison measures the **complete proposed attack-aware hierarchical filter** against the implemented single-stage CBF-QP baseline.

### All 40 controller-specific groups

The counts below are for **100 trials in each group**. The time column is the **median navigation time among successful trials**, in seconds, not the median across all attempts.

| Group | Trials | Success | Collision | Timeout | Median successful navigation time (s) |
|:--|--:|--:|--:|--:|--:|
| A01 | 100 | 92 | 0 | 8 | 43.92 |
| A02 | 100 | 90 | 0 | 10 | 43.68 |
| A03 | 100 | 88 | 0 | 12 | 43.60 |
| A04 | 100 | 86 | 0 | 14 | 43.54 |
| A05 | 100 | 71 | 0 | 29 | 44.32 |
| A06 | 100 | 78 | 0 | 22 | 50.96 |
| A07 | 100 | 82 | 0 | 18 | 48.44 |
| A08 | 100 | 47 | 17 | 36 | 53.36 |
| A09 | 100 | 52 | 0 | 48 | 53.30 |
| A10 | 100 | 62 | 0 | 38 | 53.72 |
| A11 | 100 | 96 | 0 | 4 | 29.14 |
| A12 | 100 | 94 | 0 | 6 | 29.22 |
| A13 | 100 | 91 | 0 | 9 | 29.16 |
| A14 | 100 | 95 | 0 | 5 | 29.12 |
| A15 | 100 | 72 | 10 | 18 | 29.56 |
| A16 | 100 | 87 | 0 | 13 | 29.36 |
| A17 | 100 | 93 | 0 | 7 | 34.04 |
| A18 | 100 | 15 | 71 | 14 | 83.92 |
| A19 | 100 | 24 | 12 | 64 | 34.04 |
| A20 | 100 | 42 | 0 | 58 | 49.04 |
| B01 | 100 | 100 | 0 | 0 | 48.65 |
| B02 | 100 | 100 | 0 | 0 | 49.10 |
| B03 | 100 | 100 | 0 | 0 | 49.10 |
| B04 | 100 | 100 | 0 | 0 | 48.20 |
| B05 | 100 | 94 | 0 | 6 | 50.65 |
| B06 | 100 | 92 | 0 | 8 | 50.10 |
| B07 | 100 | 98 | 0 | 2 | 56.15 |
| B08 | 100 | 51 | 7 | 42 | 84.20 |
| B09 | 100 | 59 | 0 | 41 | 72.10 |
| B10 | 100 | 67 | 0 | 33 | 67.20 |
| B11 | 100 | 100 | 0 | 0 | 37.10 |
| B12 | 100 | 100 | 0 | 0 | 36.60 |
| B13 | 100 | 99 | 0 | 1 | 36.10 |
| B14 | 100 | 100 | 0 | 0 | 37.60 |
| B15 | 100 | 81 | 4 | 15 | 41.10 |
| B16 | 100 | 82 | 0 | 18 | 41.10 |
| B17 | 100 | 87 | 0 | 13 | 41.10 |
| B18 | 100 | 6 | 33 | 61 | 35.10 |
| B19 | 100 | 4 | 3 | 93 | 35.55 |
| B20 | 100 | 12 | 0 | 88 | 67.28 |

## 7. Trial-level data: `trials.csv` (15 fields)

Each archive contains **one `trials.csv` with 2,000 trial records**. The following names are the original column names.

| No. | Column | Meaning |
|--:|:--|:--|
| 1 | `computer` | Dataset series identifier: `A` for DWB and `B` for MPPI. It does not indicate different computing hardware. |
| 2 | `controller` | Nominal navigation controller: DWB or MPPI. |
| 3 | `group_id` | Group identifier, `A01`–`A20` or `B01`–`B20`. |
| 4 | `map_id` | Terrain identifier: `H1_chicane`, `H2_bottleneck`, `H3_mixed`, `H4_opencon`, `H5_multiswitch`. Each occurs 20 times in every 100-trial group. |
| 5 | `trial_id` | Unique trial key and process-file basename. Pattern: `<group>_<speed>_<index>_<outcome>`, e.g. `A01_low_001_success`. |
| 6 | `speed` | `low` (groups 01–10) or `high` (11–20). |
| 7 | `filter_kind` | Literal values: `none`, `standard_cbf`, `proposed`. |
| 8 | `gamma_attack` | Realized injection strength setting: `0`, `0.3`, or `0.6`. |
| 9 | `nav_time_s` | Navigation duration in seconds. |
| 10 | `path_length_m` | Traveled path length in meters. |
| 11 | `min_clearance_m` | Minimum signed geometric clearance along the run, in meters; negative means geometric intersection. |
| 12 | `final_goal_distance_m` | Distance to goal at termination, in meters. |
| 13 | `sf_strict_frac` | Fraction of filter cycles in STRICT mode; populated only for the proposed-filter implementation. |
| 14 | `sf_soft_frac` | Fraction of filter cycles in SOFT mode; populated only for the proposed-filter implementation. |
| 15 | `sf_minrisk_frac` | Fraction of filter cycles in MIN_RISK mode; populated only for the proposed-filter implementation. |

**Missing values:** represented as `NA` in the master table. The three `sf_*_frac` columns are `NA` for other methods. The supplied 15-column schema has **no separate outcome column**: the terminal outcome appears in the suffix of `trial_id` (for example, `_success`).

## 8. Time-series data: `process/<group_id>/<trial_id>.csv` (8 fields)

All process files use the same header:

```csv
t_rel_s,source,x_m,y_m,clearance_m,v_mps,w_radps,stage
```

| Column | Meaning |
|:--|:--|
| `t_rel_s` | Simulation time relative to goal acceptance (seconds); time origin is 0.0. |
| `source` | Stream identifier: `odom`, `nominal`, `filtered`, `actuator`, `attack`, or `sf_event`. |
| `x_m`, `y_m` | Robot position, populated on `odom` rows only. |
| `clearance_m` | Signed clearance on `odom` rows. Some exports without pose information leave this column entirely empty. |
| `v_mps`, `w_radps` | Linear/angular velocity values for `nominal`, `filtered`, `actuator`, and `attack` rows. |
| `stage` | Safety-filter stage for `sf_event` rows. Original naming families include `STRICT`, `SOFT`, `MIN_RISK`, `SAFE_*`, `DEGRADED_SENSOR`, `OTHER`, and `V2_*`; labels are retained as recorded. |

**Stage label meaning:**

- `STRICT`, `SOFT`, and `MIN_RISK` correspond to the three hierarchy stages in the paper.
- `DEGRADED_SENSOR` indicates the safe-stop policy invoked on invalid state or perception input.
- `SAFE_*` and `V2_*` are legacy stage labels from earlier filter versions retained in the raw logs.

**Important interpretation notes:**

- An empty cell indicates **not applicable to that `source`**, not a numeric zero.
- Source streams are independently sampled and **are not time-aligned**. One timestamp generally does not have all six streams.
- `stage` labels retain their original source-specific naming. They have **not been normalized** across release variants.
- The stream-availability counts below represent the **number of process CSV files containing at least one record from that source**, rather than the number of rows in each stream.

### Stream availability

| Stream | DWB / series A: files containing stream | MPPI / series B: files containing stream |
|:--|--:|--:|
| `odom` | 2,000 | 2,000 |
| `nominal` | 1,994 | 1,437 |
| `filtered` | 1,403 | 1,164 |
| `actuator` | 2,000 | 2,000 |
| `attack` | 1,200 | 1,200 |
| `sf_event` | 1,106 | 927 |

## 9. Reading and reproducing the published results

1. Extract the DWB and MPPI archives and locate each `trials.csv` and `process/` directory. Keep the group and `trial_id` identifiers unchanged.
2. Match numerical suffixes between DWB and MPPI (e.g., A17 and B17). Use the condition table in Section 2 and the stored `map_id` to identify speed, method, attack strength, and terrain.
3. Determine the terminal outcome from the trial's recorded outcome identifier (the terminal suffix of `trial_id` in the supplied schema). Count successes, collisions, and timeouts **per 100-trial group**; all three must sum to 100.
4. Compute the median `nav_time_s` **over successful trials only**. Use `process/` files for command and trajectory analysis, not as an alternative source of terminal labels.
5. Verify five terrain identifiers × 20 records for every group; combine all 40 groups for 4,000 valid trials. The eight attacked Proposed groups give **543 successes, 0 collisions, and 257 timeouts**.

The `process/` streams are independently sampled, so join signals by time intervals/tolerances appropriate to each source rather than expecting a row-wise synchronized CSV.

**Complete-reproduction inputs:** the release includes the five formal Gazebo world layouts, their obstacle-geometry definitions, the physical-collision evaluator, and the trial-runner configuration, in addition to the source packages under `src/`.

See the [repository overview](../README.md), [`src/`](../src/), and [`docs/`](../docs/) for ROS 2 integration and code provenance.
