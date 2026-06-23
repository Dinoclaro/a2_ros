#!/usr/bin/env python3
"""Autonomous survey mission: stand, explore, save map, return home."""

from __future__ import annotations

import math
import os
from typing import List, Optional

import rclpy
from a2_interfaces.msg import OperatingMode
from a2_interfaces.srv import SetOperatingMode
from direct_lidar_inertial_odometry.srv import SavePCD
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from std_msgs.msg import Bool, String

from a2_orchestrator.mission_state import MissionState
from a2_orchestrator.stack_manager import StackManager, node_running, topic_publisher_count


class MissionOrchestrator(Node):
    """State-machine node coordinating locomotion, explore/nav stacks, and map save."""

    MODE_STAND = OperatingMode.STAND_UP
    MODE_UNLOCK = OperatingMode.BALANCE_STAND
    MODE_WALK = OperatingMode.VELOCITY_MOVE

    LAUNCH_PACKAGE = 'a2_ros'

    def __init__(self) -> None:
        super().__init__('mission_orchestrator')
        self._declare_parameters()
        self._load_parameters()

        self._state = MissionState.CHECK_PREREQS
        self._state_entered_at = self.get_clock().now()
        self._exploring_started_at = None
        self._explore_stop_reason = ''

        self._mode_request_pending = False
        self._last_mode_accepted: Optional[bool] = None

        self._lidar_seen = False
        self._camera_seen = False
        self._exploration_finished = False
        self._last_odom: Optional[Odometry] = None
        self._home = Point()
        self._home_recorded = False
        self._home_stable_count = 0
        self._nav_goal_sent = False
        self._map_save_done = False
        self._done_logged = False
        self._nav_walk_requested = False

        self._stack = StackManager()

        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._goal_pub = self.create_publisher(PointStamped, self._goal_topic, 10)
        self._start_exploration_pub = self.create_publisher(
            Bool, self._start_exploration_topic, 10
        )

        self.create_subscription(
            Odometry, self._odom_topic, self._odom_callback, qos_sensor
        )
        self.create_subscription(
            PointCloud2, self._lidar_topic, self._lidar_callback, qos_sensor
        )
        self.create_subscription(
            Bool, self._exploration_finish_topic, self._exploration_finish_callback, 10
        )

        if '/compressed' in self._camera_topic:
            self.create_subscription(
                CompressedImage,
                self._camera_topic,
                self._camera_callback,
                qos_sensor,
            )
        else:
            self.create_subscription(
                Image, self._camera_topic, self._camera_callback, qos_sensor
            )

        self._mode_client = self.create_client(SetOperatingMode, '/a2/set_mode')
        self.create_timer(0.2, self._tick)
        self._set_status('initialized')

    # ------------------------------------------------------------------ params

    def _declare_parameters(self) -> None:
        self.declare_parameter('save_dir', './runs/a2_mission')
        self.declare_parameter('stand_wait_sec', 3.0)
        self.declare_parameter('exploration_finish_topic', '/exploration_finish')
        self.declare_parameter('exploration_timeout_sec', 60.0)
        self.declare_parameter('home_arrival_threshold_m', 0.5)
        self.declare_parameter('nav_home_timeout_sec', 60.0)
        self.declare_parameter('skip_home', False)
        self.declare_parameter('map_leaf_size', 0.15)
        self.declare_parameter('explore_launch', 'launch/exploration.launch.py')
        self.declare_parameter('nav_launch', 'launch/navigation.launch.py')
        # use_sim_time is pre-declared by rclpy / launch; do not declare again.
        self.declare_parameter('stack_rviz', False)
        self.declare_parameter('lidar_topic', '/front_lidar/points')
        self.declare_parameter('camera_image_topic', '/camera/image/compressed')
        self.declare_parameter('prereq_timeout_sec', 60.0)
        self.declare_parameter('odom_topic', '/state_estimation')
        self.declare_parameter('goal_topic', '/goal_point')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('status_topic', '/mission/status')
        self.declare_parameter('dlio_save_pcd_service', '/save_pcd')
        self.declare_parameter('registered_scan_topic', '/registered_scan')
        self.declare_parameter('start_exploration_topic', '/start_exploration')
        self.declare_parameter('explore_stack_warmup_sec', 8.0)
        self.declare_parameter('explore_stack_ready_timeout_sec', 45.0)

    def _load_parameters(self) -> None:
        self._save_dir = self.get_parameter('save_dir').value
        self._stand_wait_sec = self.get_parameter('stand_wait_sec').value
        self._exploration_finish_topic = self.get_parameter(
            'exploration_finish_topic'
        ).value
        self._exploration_timeout_sec = self.get_parameter(
            'exploration_timeout_sec'
        ).value
        self._home_threshold = self.get_parameter('home_arrival_threshold_m').value
        self._nav_home_timeout_sec = self.get_parameter('nav_home_timeout_sec').value
        self._skip_home = self.get_parameter('skip_home').value
        self._map_leaf_size = self.get_parameter('map_leaf_size').value
        self._explore_launch = self.get_parameter('explore_launch').value
        self._nav_launch = self.get_parameter('nav_launch').value
        self._use_sim_time = (
            self.get_parameter('use_sim_time').value
            if self.has_parameter('use_sim_time')
            else False
        )
        self._stack_rviz = self.get_parameter('stack_rviz').value
        self._lidar_topic = self.get_parameter('lidar_topic').value
        self._camera_topic = self.get_parameter('camera_image_topic').value
        self._prereq_timeout_sec = self.get_parameter('prereq_timeout_sec').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._goal_topic = self.get_parameter('goal_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._status_topic = self.get_parameter('status_topic').value
        self._dlio_save_pcd_service = self.get_parameter('dlio_save_pcd_service').value
        self._start_exploration_topic = self.get_parameter(
            'start_exploration_topic'
        ).value
        self._explore_stack_warmup_sec = self.get_parameter(
            'explore_stack_warmup_sec'
        ).value
        self._explore_stack_ready_timeout_sec = self.get_parameter(
            'explore_stack_ready_timeout_sec'
        ).value

        os.makedirs(self._save_dir, exist_ok=True)

    # ------------------------------------------------------------------ helpers

    @staticmethod
    def _launch_name_from_param(launch_param: str) -> str:
        """Extract ``exploration.launch.py`` from ``launch/exploration.launch.py``."""
        return os.path.basename(launch_param)

    def _launch_args(self) -> List[str]:
        args = [f'rviz:={"true" if self._stack_rviz else "false"}']
        if self._use_sim_time:
            args.append('use_sim_time:=true')
        return args

    def _set_status(self, detail: str) -> None:
        msg = String()
        msg.data = f'{self._state.name}:{detail}'
        self._status_pub.publish(msg)
        self.get_logger().info(f'[{self._state.name}] {detail}')

    def _transition(self, new_state: MissionState, detail: str = '') -> None:
        self._state = new_state
        self._state_entered_at = self.get_clock().now()
        self._set_status(detail or new_state.name.lower())

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._state_entered_at).nanoseconds * 1e-9

    def _exploring_elapsed(self) -> float:
        if self._exploring_started_at is None:
            return 0.0
        return (
            self.get_clock().now() - self._exploring_started_at
        ).nanoseconds * 1e-9

    def _publish_start_exploration(self) -> None:
        msg = Bool()
        msg.data = True
        self._start_exploration_pub.publish(msg)

    def _spawn_explore_stack(self) -> None:
        self._stack.spawn_stack(
            self.LAUNCH_PACKAGE,
            self._launch_name_from_param(self._explore_launch),
            self._launch_args(),
            self._save_dir,
        )
        self.get_logger().info(
            f'Explore stack spawned (log: {self._stack.log_path})'
        )

    def _spawn_nav_stack(self) -> None:
        self._stack.spawn_stack(
            self.LAUNCH_PACKAGE,
            self._launch_name_from_param(self._nav_launch),
            self._launch_args(),
            self._save_dir,
        )
        self.get_logger().info(f'Nav stack spawned (log: {self._stack.log_path})')

    # ------------------------------------------------------------------ callbacks

    def _odom_callback(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _lidar_callback(self, _msg: PointCloud2) -> None:
        self._lidar_seen = True

    def _camera_callback(self, _msg) -> None:
        self._camera_seen = True

    def _exploration_finish_callback(self, msg: Bool) -> None:
        if msg.data:
            self._exploration_finished = True

    # ------------------------------------------------------------------ mode FSM

    def _request_mode(self, mode: int) -> bool:
        if self._mode_request_pending or self._last_mode_accepted is not None:
            return False
        if not self._mode_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn('Waiting for /a2/set_mode...')
            return False

        req = SetOperatingMode.Request()
        req.mode = mode
        self._mode_request_pending = True
        future = self._mode_client.call_async(req)
        future.add_done_callback(lambda f: self._on_mode_response(f, mode))
        return True

    def _mode_response_ready(self) -> bool:
        return not self._mode_request_pending and self._last_mode_accepted is not None

    def _consume_mode_response(self) -> bool:
        accepted = bool(self._last_mode_accepted)
        self._last_mode_accepted = None
        return accepted

    def _on_mode_response(self, future, requested_mode: int) -> None:
        self._mode_request_pending = False
        try:
            response = future.result()
        except Exception as ex:  # noqa: BLE001
            self.get_logger().error(f'Mode {requested_mode} call failed: {ex}')
            self._last_mode_accepted = False
            self._transition(MissionState.FAILED, 'mode service error')
            return

        self._last_mode_accepted = response.success
        if response.success:
            self.get_logger().info(
                f'Mode {requested_mode} accepted: {response.message}'
            )
            return

        self.get_logger().error(
            f'Mode {requested_mode} rejected: {response.message}'
        )
        self._transition(MissionState.FAILED, response.message)

    # ------------------------------------------------------------------ home / map

    def _record_home(self) -> bool:
        if self._last_odom is None:
            return False
        self._home = self._last_odom.pose.pose.position
        self._home_recorded = True
        origin_path = os.path.join(self._save_dir, 'origin.txt')
        with open(origin_path, 'w', encoding='utf-8') as handle:
            handle.write(
                f'{self._home.x:.6f} {self._home.y:.6f} {self._home.z:.6f}\n'
            )
        self.get_logger().info(
            f'Home recorded at ({self._home.x:.2f}, {self._home.y:.2f}, '
            f'{self._home.z:.2f})'
        )
        return True

    def _publish_home_goal(self) -> None:
        goal = PointStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._map_frame
        goal.point = self._home
        self._goal_pub.publish(goal)

    def _distance_to_home(self) -> Optional[float]:
        if self._last_odom is None or not self._home_recorded:
            return None
        dx = self._last_odom.pose.pose.position.x - self._home.x
        dy = self._last_odom.pose.pose.position.y - self._home.y
        return math.hypot(dx, dy)

    def _save_map(self) -> bool:
        client = self.create_client(SavePCD, self._dlio_save_pcd_service)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('SavePCD service unavailable')
            return False

        req = SavePCD.Request()
        req.leaf_size = float(self._map_leaf_size)
        req.save_path = self._save_dir
        try:
            response = client.call(req)
        except Exception as ex:  # noqa: BLE001
            self.get_logger().error(f'SavePCD failed: {ex}')
            return False

        if response.success:
            self.get_logger().info(
                f'Map saved to {self._save_dir}/clean_map.pcd'
            )
            return True

        self.get_logger().error('SavePCD returned success=False')
        return False

    # ------------------------------------------------------------------ tick dispatch

    def _tick(self) -> None:
        if self._state in (MissionState.FAILED, MissionState.DONE):
            return

        handlers = {
            MissionState.CHECK_PREREQS: self._tick_prereqs,
            MissionState.STAND: self._tick_stand,
            MissionState.WAIT_STAND: self._tick_wait_stand,
            MissionState.UNLOCK: self._tick_unlock,
            MissionState.WALK: self._tick_walk,
            MissionState.RECORD_HOME: self._tick_record_home,
            MissionState.SPAWN_EXPLORE: self._tick_spawn_explore,
            MissionState.EXPLORING: self._tick_exploring,
            MissionState.KILL_EXPLORE: self._tick_kill_explore,
            MissionState.SAVE_MAP: self._tick_save_map,
            MissionState.SPAWN_NAV: self._tick_spawn_nav,
            MissionState.NAV_HOME: self._tick_nav_home,
            MissionState.KILL_NAV: self._tick_kill_nav,
            MissionState.DONE: self._tick_done,
        }
        handlers[self._state]()

    def _tick_prereqs(self) -> None:
        if self._elapsed() > self._prereq_timeout_sec:
            self._transition(
                MissionState.FAILED,
                'prerequisite timeout (start sim or nuc+pc2 first)',
            )
            return
        if not self._mode_client.wait_for_service(timeout_sec=0.0):
            self._set_status('waiting for /a2/set_mode')
            return
        if not self._lidar_seen:
            self._set_status('waiting for lidar')
            return
        if not self._camera_seen:
            self._set_status('waiting for camera')
            return
        self._transition(MissionState.STAND)

    def _tick_stand(self) -> None:
        if not self._mode_request_pending and self._last_mode_accepted is None:
            self._request_mode(self.MODE_STAND)
            return
        if not self._mode_response_ready():
            return
        if not self._consume_mode_response():
            return
        self._transition(MissionState.WAIT_STAND, 'stand accepted')

    def _tick_wait_stand(self) -> None:
        if self._elapsed() < self._stand_wait_sec:
            return
        self._transition(MissionState.UNLOCK)

    def _tick_unlock(self) -> None:
        if not self._mode_request_pending and self._last_mode_accepted is None:
            self._request_mode(self.MODE_UNLOCK)
            return
        if not self._mode_response_ready():
            return
        if not self._consume_mode_response():
            return
        self._transition(MissionState.WALK, 'unlock accepted')

    def _tick_walk(self) -> None:
        if not self._mode_request_pending and self._last_mode_accepted is None:
            self._request_mode(self.MODE_WALK)
            return
        if not self._mode_response_ready():
            return
        if not self._consume_mode_response():
            return
        self._transition(MissionState.RECORD_HOME, 'walk accepted')

    def _tick_record_home(self) -> None:
        if not self._record_home():
            self._set_status('waiting for odometry')
            return
        self._exploration_finished = False
        self._transition(MissionState.SPAWN_EXPLORE)

    def _tick_spawn_explore(self) -> None:
        if not self._stack.is_running() and not self._stack.has_exited():
            try:
                self._spawn_explore_stack()
            except Exception as ex:  # noqa: BLE001
                self._transition(MissionState.FAILED, f'explore spawn failed: {ex}')
            return

        if self._stack.has_exited():
            log_hint = self._stack.log_path or 'explore_launch.log'
            self._transition(
                MissionState.FAILED,
                f'explore stack exited early; see {log_hint}',
            )
            return

        if self._elapsed() < self._explore_stack_warmup_sec:
            self._set_status('waiting for explore stack')
            return

        tare_ready = (
            node_running('tare_planner')
            or topic_publisher_count('/way_point') > 0
        )
        if not tare_ready:
            self._publish_start_exploration()
            ready_deadline = (
                self._explore_stack_warmup_sec
                + self._explore_stack_ready_timeout_sec
            )
            if self._elapsed() < ready_deadline:
                self._set_status('waiting for TARE (/way_point publisher)')
                return
            log_hint = self._stack.log_path or 'explore_launch.log'
            self._transition(
                MissionState.FAILED,
                f'TARE not ready; see {log_hint}',
            )
            return

        self._publish_start_exploration()
        self._exploring_started_at = self.get_clock().now()
        self._transition(MissionState.EXPLORING, 'exploring')

    def _tick_exploring(self) -> None:
        if self._stack.has_exited():
            self._explore_stop_reason = 'stack_exited'
            self._transition(MissionState.KILL_EXPLORE, 'explore stack exited')
            return
        if self._exploration_finished:
            self._explore_stop_reason = 'complete'
            self._transition(MissionState.KILL_EXPLORE, 'exploration complete')
            return
        if (
            self._exploration_timeout_sec > 0.0
            and self._exploring_elapsed() >= self._exploration_timeout_sec
        ):
            self._explore_stop_reason = 'timeout'
            self._transition(MissionState.KILL_EXPLORE, 'exploration timeout')
            return
        self._set_status(
            f'exploring ({self._exploring_elapsed():.0f}s, '
            f'limit={self._exploration_timeout_sec:.0f}s)'
        )

    def _tick_kill_explore(self) -> None:
        self._stack.kill_stack()
        self._transition(
            MissionState.SAVE_MAP,
            f'stopped: {self._explore_stop_reason}',
        )

    def _tick_save_map(self) -> None:
        if not self._map_save_done:
            self._save_map()
            self._map_save_done = True
        if self._skip_home:
            self._transition(MissionState.DONE, 'map saved, skip_home')
            return
        self._transition(MissionState.SPAWN_NAV, 'map saved, starting nav')

    def _tick_spawn_nav(self) -> None:
        if not self._stack.is_running() and not self._stack.has_exited():
            if not self._nav_walk_requested:
                if not self._mode_request_pending and self._last_mode_accepted is None:
                    self._request_mode(self.MODE_WALK)
                    return
                if self._mode_request_pending or not self._mode_response_ready():
                    return
                self._consume_mode_response()
                self._nav_walk_requested = True

            try:
                self._spawn_nav_stack()
            except Exception as ex:  # noqa: BLE001
                self._transition(MissionState.FAILED, f'nav spawn failed: {ex}')
            return

        if self._stack.has_exited():
            log_hint = self._stack.log_path or 'nav_launch.log'
            self._transition(
                MissionState.FAILED,
                f'nav stack exited early; see {log_hint}',
            )
            return

        if self._elapsed() < self._explore_stack_warmup_sec:
            self._set_status('waiting for nav stack')
            return

        self._nav_goal_sent = False
        self._home_stable_count = 0
        self._transition(MissionState.NAV_HOME, 'navigating home')

    def _tick_nav_home(self) -> None:
        if self._stack.has_exited():
            self._transition(MissionState.FAILED, 'nav stack exited early')
            return
        if self._elapsed() > self._nav_home_timeout_sec:
            self._transition(MissionState.FAILED, 'nav home timeout')
            return
        if not self._nav_goal_sent:
            self._publish_home_goal()
            self._nav_goal_sent = True
            self._set_status('goal published')
            return

        dist = self._distance_to_home()
        if dist is not None and dist <= self._home_threshold:
            self._home_stable_count += 1
            if self._home_stable_count >= 5:
                self._transition(
                    MissionState.KILL_NAV,
                    f'home reached ({dist:.2f} m)',
                )
            return

        self._home_stable_count = 0
        detail = (
            f'navigating home ({dist:.2f} m)' if dist is not None else 'navigating'
        )
        self._set_status(detail)

    def _tick_kill_nav(self) -> None:
        self._stack.kill_stack()
        self._transition(MissionState.DONE, 'nav complete')

    def _tick_done(self) -> None:
        if self._done_logged:
            return
        if not self._mode_request_pending and self._last_mode_accepted is None:
            self._request_mode(self.MODE_UNLOCK)
        elif self._mode_response_ready():
            self._consume_mode_response()
            self._done_logged = True
            self._set_status(f'finished ({self._explore_stop_reason})')

    def destroy_node(self) -> None:
        self._stack.kill_stack()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
