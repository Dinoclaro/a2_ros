# a2_orchestrator — Autonomous Mission Package

This package runs the **autonomous survey mission**: stand up the robot, explore with TARE, save a DLIO map, and navigate home with FAR.

Most mission logic lives here so changes stay isolated from `a2_ros` launch files.

---

## Quick Start

```bash
# Terminal 1 — sim + robot stack (must be running first)
export A2_MODE=sim
a2 sim

# Terminal 2 — mission (DLIO + orchestrator)
a2 mission save_dir:=/tmp/run1

# Monitor progress
ros2 topic echo /mission/status
```

Real robot: start `nuc.launch.py` + `pc2_bridge.sh`, then `a2 mission save_dir:=/tmp/run1`.

---

## Package Layout

```
a2_orchestrator/
├── README.md                          ← this file
├── config/
│   └── mission_defaults.yaml          ← default orchestrator parameters
├── launch/
│   └── mission.launch.py              ← entry point: DLIO + orchestrator
└── a2_orchestrator/
    ├── mission_state.py               ← state enum (easy to extend)
    ├── stack_manager.py               ← subprocess spawn/kill for explore/nav
    ├── mission_orchestrator.py        ← main state machine node
    └── detection_logger.py            ← optional CSV detection logger
```

**External stacks** (unchanged, in `a2_ros`):

- `exploration.launch.py` — TARE + terrain + local planner (spawned as subprocess)
- `navigation.launch.py` — FAR + terrain + local planner (spawned as subprocess)

---

## How It Works

### High-Level Flow

```
Prerequisites (a2 sim)  →  mission.launch.py  →  mission_orchestrator
                              │
                              ├─ optional DLIO
                              │
                              └─ state machine:
                                   stand → unlock → walk → record home
                                   → spawn exploration.launch.py
                                   → explore until finish or timeout
                                   → save map (SavePCD)
                                   → spawn navigation.launch.py
                                   → publish home goal → arrive → done
```

### Motion During Exploration

```
tare_planner  →  /way_point  →  localPlanner  →  pathFollower  →  /nav_vel  →  twist_mux  →  /cmd_vel
```

`twist_mux` and the locomotion FSM come from `a2 sim`, not from this package.

### Motion During Return Home

```
mission_orchestrator  →  /goal_point  →  far_planner  →  /way_point  →  localPlanner  →  ...
```

---

## State Machine

States are defined in `mission_state.py`. The tick loop in `mission_orchestrator.py` dispatches to one handler per state:

| State | What happens |
|-------|----------------|
| `CHECK_PREREQS` | Wait for `/a2/set_mode`, lidar, camera |
| `STAND` → `WAIT_STAND` → `UNLOCK` → `WALK` | Locomotion mode sequence (2→3→4) |
| `RECORD_HOME` | Write `{save_dir}/origin.txt` from odometry |
| `SPAWN_EXPLORE` | Subprocess: `ros2 launch a2_ros exploration.launch.py` |
| `EXPLORING` | Run until `/exploration_finish`, timeout, or stack crash |
| `KILL_EXPLORE` | Stop explore subprocess |
| `SAVE_MAP` | Call DLIO `SavePCD` → `clean_map.pcd` |
| `SPAWN_NAV` | Subprocess: `ros2 launch a2_ros navigation.launch.py` |
| `NAV_HOME` | Publish `/goal_point`, wait until near origin |
| `KILL_NAV` / `DONE` | Cleanup |

Status is published on `/mission/status` as `STATE:detail` (e.g. `EXPLORING:exploring (42s, limit=600s)`).

---

## Key Modules

### `stack_manager.py`

Handles **one subprocess at a time** (explore OR nav, never both):

- `spawn_stack(package, launch_name, args, save_dir)` — runs `ros2 launch`, logs to `{save_dir}/explore_launch.log` or `nav_launch.log`
- `kill_stack()` — SIGINT process group, then SIGKILL after 10 s
- `topic_publisher_count()`, `node_running()` — used to verify TARE started

**To change spawn behavior** (e.g. extra launch args), edit `_launch_args()` in `mission_orchestrator.py`.

### `mission_orchestrator.py`

Main ROS node. Organized as:

- `_declare_parameters` / `_load_parameters` — all tunables
- `_tick_*` methods — one per state (add new states here)
- Mode service client — `/a2/set_mode`
- **Do not** `declare_parameter('use_sim_time')` — launch pre-declares it; re-declaring crashes the node

### `detection_logger.py`

Optional node that writes `{save_dir}/detections.csv`. Disabled in `mission.launch.py` until the object_detection ONNX model path is fixed. To enable, uncomment the detection blocks in `launch/mission.launch.py`.

---

## Parameters

Defaults live in `config/mission_defaults.yaml`. Override via launch:

```bash
a2 mission save_dir:=/tmp/run1 exploration_timeout_sec:=300 skip_home:=true
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `save_dir` | `./runs/a2_mission` | Output directory |
| `exploration_timeout_sec` | `60` | Max explore time (seconds) |
| `skip_home` | `false` | Skip return navigation after map save |
| `home_arrival_threshold_m` | `0.5` | Distance to origin considered "home" |
| `explore_stack_warmup_sec` | `8` | Wait after spawn before TARE check |
| `explore_stack_ready_timeout_sec` | `45` | Extra wait for TARE `/way_point` |
| `stand_wait_sec` | `6` | Pause after stand before unlock |
| `stack_rviz` | `false` | Pass `rviz:=true` to spawned explore/nav stacks |
| `camera_image_topic` | `/camera/image/compressed` | Prereq camera (sim: `/camera/image_raw`) |
| `dlio_save_pcd_service` | `/save_pcd` | Map save service |

Launch-level args in `mission.launch.py`:

| Launch arg | Default | Description |
|------------|---------|-------------|
| `use_sim_time` | `false` | Required `true` in sim |
| `include_dlio` | `true` | Set `false` if DLIO already running |

---

## Outputs

All files under `save_dir`:

| File | Description |
|------|-------------|
| `origin.txt` | Home position `x y z` |
| `clean_map.pcd` | DLIO voxel map |
| `explore_launch.log` | Explore stack stdout/stderr |
| `nav_launch.log` | Nav stack stdout/stderr |
| `detections.csv` | Optional detection log |

---

## Troubleshooting

### Robot doesn't move during exploration

1. Check TARE is running: `ros2 node list | grep tare`
2. Check waypoint publisher: `ros2 topic info /way_point` (Publisher count should be ≥ 1)
3. Read spawn log: `cat {save_dir}/explore_launch.log`
4. Ensure sim uses `use_sim_time:=true` (`a2 mission` sets this when `A2_MODE=sim`)

### Mission fails at prerequisites

- Start `a2 sim` (or nuc + pc2 bridge) **before** `a2 mission`
- Sim needs `camera_image_topic:=/camera/image_raw` (set automatically by `a2 mission`)

### Exploration never finishes

- TARE must publish on `/exploration_finish` (absolute topic in `a2_ros/config/autonomy/tare_a2.yaml`)
- Or mission stops at `exploration_timeout_sec`

### Cleanup orphaned processes

```bash
a2 down mission explore nav
```

---

## Modifying the Mission

### Add a new state

1. Add enum value in `mission_state.py`
2. Add `_tick_your_state()` in `mission_orchestrator.py`
3. Register it in the `_tick()` handlers dict
4. Transition into/out of it from adjacent states

### Change explore/nav stacks

Edit parameters in `mission_defaults.yaml`:

```yaml
explore_launch: "launch/exploration.launch.py"   # relative to a2_ros share
nav_launch: "launch/navigation.launch.py"
```

Or change `LAUNCH_PACKAGE = 'a2_ros'` in `mission_orchestrator.py` if stacks move packages.

### Enable object detection logging

1. Fix ONNX model path in `object_detection` package
2. Uncomment detection launch + `detection_logger` in `launch/mission.launch.py`
3. Rebuild and run with `sim_detection:=true` in sim

### Change stop condition for exploration

Edit `_tick_exploring()` in `mission_orchestrator.py` — currently stops on:

- `/exploration_finish` Bool true
- `exploration_timeout_sec` elapsed
- explore subprocess exit

---

## Build

```bash
colcon build --packages-select a2_orchestrator --symlink-install
source install/setup.bash
```

---

## Minimal Changes Outside This Package

To keep impact low, only these non-orchestrator files were touched:

| File | Change |
|------|--------|
| `a2_ros/config/autonomy/tare_a2.yaml` | `/exploration_finish` absolute topic |
| `a2_ros/launch/exploration.launch.py` | `use_sim_time` launch arg |
| `a2_ros/launch/navigation.launch.py` | `use_sim_time` launch arg |
| `scripts/a2` | `a2 mission` command |
| `scripts/a2_shell.sh` | tab completion |

Explore and navigation launch files were **not** refactored to `autonomy_base` — they remain as-is in `a2_ros`.
