# a2_orchestrator — Autonomous Mission Package

This package runs the **autonomous survey mission**: stand up the robot, explore with TARE, save a DLIO map, and navigate home with FAR.

The orchestrator assumes the **mega stack** (TARE + FAR + terrain + `waypoint_mux`) is already running. It controls locomotion modes and switches planners via topics — no subprocess stack swapping.

---

## Quick Start

```bash
# Terminal 1 — sim
export A2_MODE=sim
a2 sim

# Terminal 2 — DLIO + mega stack (TARE + FAR + mux)
a2 dlio
ros2 launch a2_ros mega.launch.py use_sim_time:=true rviz:=false

# Terminal 3 — mission orchestrator
a2 mission save_dir:=/tmp/run1

# Monitor progress
ros2 topic echo /mission/status
```

Real robot: start `nuc.launch.py` + pc2 bridge, DLIO, and mega stack, then run the orchestrator.

---

## Package Layout

```
a2_orchestrator/
├── README.md
├── config/
│   └── mission_defaults.yaml
├── launch/
│   └── mission.launch.py              ← DLIO + orchestrator (optional entry)
└── a2_orchestrator/
    ├── mission_state.py               ← state enum
    ├── mission_orchestrator.py        ← main state machine node
    ├── waypoint_mux.py                ← TARE/FAR waypoint multiplexer
    ├── stack_manager.py               ← legacy (unused by simplified orchestrator)
    └── detection_logger.py            ← optional CSV detection logger
```

**Prerequisite stack** (in `a2_ros`):

- `mega.launch.py` — terrain, local planner, TARE, FAR, `waypoint_mux`

---

## How It Works

### High-Level Flow

```
Prerequisites (sim + mega + DLIO)  →  mission_orchestrator
                                           │
                                           └─ state machine:
                                                stand → unlock → walk
                                                → select TARE + start exploration
                                                → explore until finish or timeout
                                                → save map (SavePCD)
                                                → select FAR + goal (0,0,0)
                                                → arrive → sit down → done
```

### Motion During Exploration

```
tare_planner  →  /tare/way_point  →  waypoint_mux  →  /way_point  →  localPlanner  →  ...
```

Orchestrator publishes `/planner/select` = `tare` and `/start_exploration` = `true`.

### Motion During Return Home

```
mission_orchestrator  →  /planner/select far
                     →  /goal_point (0,0,0)
far_planner  →  /far/way_point  →  waypoint_mux  →  /way_point  →  localPlanner  →  ...
```

On tare→far switch, `waypoint_mux` publishes a stop goal at the current pose before FAR takes over.

---

## State Machine

| State | What happens |
|-------|----------------|
| `CHECK_PREREQS` | Wait for `/a2/set_mode` and odometry |
| `STAND` → `WAIT_STAND` → `UNLOCK` → `WALK` | Locomotion mode sequence (2→3→4) |
| `RECORD_HOME` | Write `{save_dir}/origin.txt` (actual start pose) |
| `START_EXPLORE` | `/planner/select` = `tare`, `/start_exploration` = `true` |
| `EXPLORING` | Wait for `/exploration_finish` or timeout |
| `SAVE_MAP` | Async call to DLIO `SavePCD` → `clean_map.pcd` |
| `NAV_HOME` | `/planner/select` = `far`, `/goal_point` at home goal (default 0,0,0) |
| `SIT_DOWN` | Modes 3 → 1 (`BALANCE_STAND` then `STAND_DOWN`) |
| `DONE` | Mission complete |

Status is published on `/mission/status` as `STATE:detail`.

---

## Key Modules

### `mission_orchestrator.py`

Main ROS node. Publishes:

| Topic | Message | Purpose |
|-------|---------|---------|
| `/planner/select` | `std_msgs/String` | `tare` or `far` |
| `/start_exploration` | `std_msgs/Bool` | Start TARE when `kAutoStart: false` |
| `/goal_point` | `PointStamped` | FAR navigation goal |
| `/mission/status` | `std_msgs/String` | State machine status |

**Do not** `declare_parameter('use_sim_time')` — launch pre-declares it.

### `waypoint_mux.py`

Runs inside `mega.launch.py`. Forwards TARE or FAR waypoints to `/way_point`. Listens on `/planner/select`.

Manual switch:

```bash
ros2 topic pub --once /planner/select std_msgs/msg/String "{data: 'far'}"
ros2 topic pub --once /planner/select std_msgs/msg/String "{data: 'tare'}"
```

---

## Parameters

Defaults in `config/mission_defaults.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `save_dir` | `/tmp/a2_mission` | Output directory |
| `exploration_timeout_sec` | `600` | Max explore time (seconds) |
| `skip_home` | `false` | Skip return navigation after map save |
| `home_arrival_threshold_m` | `0.5` | Distance to home goal considered "arrived" |
| `home_goal_x/y/z` | `0.0` | FAR return-home goal in `map` frame |
| `planner_select_topic` | `/planner/select` | Mux control topic |
| `nav_home_timeout_sec` | `600` | Max time for return navigation |
| `stand_wait_sec` | `4.0` | Pause after stand before unlock |
| `dlio_save_pcd_service` | `/save_pcd` | Map save service |

Override via launch:

```bash
a2 mission save_dir:=/tmp/run1 exploration_timeout_sec:=300 skip_home:=true
```

---

## Outputs

| File | Description |
|------|-------------|
| `origin.txt` | Actual start pose `x y z` |
| `clean_map.pcd` | DLIO voxel map |

---

## Troubleshooting

### Robot doesn't move during exploration

1. Ensure mega stack is running and mux is on `tare`: check `/mission/status`
2. TARE needs `kAutoStart: false` if you rely on `/start_exploration` only
3. Check `/tare/way_point` and `/way_point` have traffic

### Exploration never finishes

- TARE publishes `/exploration_finish` when done
- Or mission stops at `exploration_timeout_sec`

### Nav home doesn't start

- FAR visibility graph must be initialized before goals are accepted
- Check `/goal_point` and `/far/way_point` after `SAVE_MAP` state

---

## Modifying the Mission

### Add a new state

1. Add enum value in `mission_state.py`
2. Add `_tick_your_state()` in `mission_orchestrator.py`
3. Register it in the `_tick()` handlers dict
4. Transition from adjacent states

### Change home goal

Set `home_goal_x`, `home_goal_y`, `home_goal_z` in `mission_defaults.yaml` or via launch args.

### Change exploration stop condition

Edit `_tick_exploring()` — currently stops on `/exploration_finish` or timeout.

---

## Build

```bash
colcon build --packages-select a2_orchestrator --symlink-install
source install/setup.bash
```
