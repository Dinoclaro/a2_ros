#!/usr/bin/env python3
"""Autonomous mission state machine: stand, explore, detect, return home, save map."""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from a2_interfaces.msg import OperatingMode
from a2_interfaces.srv import SetOperatingMode
from direct_lidar_inertial_odometry.srv import SavePCD
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from object_detection_msgs.msg import (
    ObjectDetectionInfo,
    ObjectDetectionInfoArray,
    PointCloudArray,
)
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class MissionState(Enum):
    CHECK_PREREQS = auto()
    STAND = auto()
    WAIT_STAND = auto()
    UNLOCK = auto()
    RECORD_HOME = auto()
    WAIT_DLIO = auto()
    WAIT_PRE_WALK = auto()
    WALK = auto()
    START_EXPLORE = auto()
    EXPLORING = auto()
    STOP_EXPLORE = auto()
    NAV_HOME = auto()
    SAVE_MAP = auto()
    DONE = auto()
    FAILED = auto()


@dataclass
class SavedDetection:
    class_id: str = ''
    confidence: float = 0.0
    position_map: Point = field(default_factory=Point)
    source_frame: str = ''
    stamp_sec: float = 0.0
    point_clouds: List[PointCloud2] = field(default_factory=list)


class MissionOrchestrator(Node):
    MODE_STAND = OperatingMode.STAND_UP
    MODE_UNLOCK = OperatingMode.BALANCE_STAND
    MODE_WALK = OperatingMode.VELOCITY_MOVE

    def __init__(self):
        super().__init__('mission_orchestrator')
        self._declare_parameters()
        self._load_parameters()

        self._state = MissionState.CHECK_PREREQS
        self._state_entered_at = self.get_clock().now()
        self._mode_request_pending = False
        self._last_mode_accepted: Optional[bool] = None
        self._map_save_done = False

        self._lidar_seen = False
        self._camera_seen = False
        self._registered_scan_count = 0
        self._last_odom: Optional[Odometry] = None

        self._detection_streak = 0
        self._latest_detections = ObjectDetectionInfoArray()
        self._latest_detection_clouds = PointCloudArray()
        self._saved_detection: Optional[SavedDetection] = None

        self._home = Point()
        self._home_recorded = False
        self._nav_home_started_at = None

        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._start_explore_pub = self.create_publisher(
            Bool, self._start_exploration_topic, 10
        )
        self._goal_pub = self.create_publisher(PointStamped, self._goal_topic, 10)
        self._waypoint_source_pub = self.create_publisher(
            String, self._waypoint_source_topic, 10
        )

        self.create_subscription(
            Odometry, self._odom_topic, self._odom_callback, qos_sensor
        )
        self.create_subscription(
            PointCloud2, self._registered_scan_topic, self._scan_callback, qos_sensor
        )
        self.create_subscription(
            PointCloud2, self._lidar_topic, self._lidar_callback, qos_sensor
        )
        self.create_subscription(
            ObjectDetectionInfoArray,
            self._detection_info_topic,
            self._detection_info_callback,
            10,
        )
        self.create_subscription(
            PointCloudArray,
            self._detection_clouds_topic,
            self._detection_clouds_callback,
            10,
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

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_timer(0.2, self._tick)
        self._set_status('initialized')

    def _declare_parameters(self):
        self.declare_parameter('stand_wait_sec', 5.0)
        self.declare_parameter('pre_walk_wait_sec', 5.0)
        self.declare_parameter('dlio_ready_timeout_sec', 30.0)
        self.declare_parameter('dlio_min_registered_scan_count', 5)
        self.declare_parameter('lidar_topic', '/front_lidar/points')
        self.declare_parameter('camera_image_topic', '/camera/image/compressed')
        self.declare_parameter('registered_scan_topic', '/registered_scan')
        self.declare_parameter('detection_info_topic', '/detection_info')
        self.declare_parameter('detection_clouds_topic', '/detection_point_clouds')
        self.declare_parameter('prereq_timeout_sec', 60.0)
        self.declare_parameter('target_class', 'bottle')
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('detection_frames_required', 3)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_topic', '/state_estimation')
        self.declare_parameter('goal_topic', '/goal_point')
        self.declare_parameter('home_arrival_threshold_m', 0.5)
        self.declare_parameter('nav_home_timeout_sec', 300.0)
        self.declare_parameter('save_dir', '/tmp/a2_mission')
        self.declare_parameter('map_leaf_size', 0.15)
        self.declare_parameter('dlio_save_pcd_service', '/dlio_map_node/save_pcd')
        self.declare_parameter('status_topic', '/mission/status')
        self.declare_parameter('waypoint_source_topic', '/mission/waypoint_source')
        self.declare_parameter('start_exploration_topic', '/start_exploration')

    def _load_parameters(self):
        self._stand_wait_sec = self.get_parameter('stand_wait_sec').value
        self._pre_walk_wait_sec = self.get_parameter('pre_walk_wait_sec').value
        self._dlio_ready_timeout_sec = self.get_parameter('dlio_ready_timeout_sec').value
        self._dlio_min_scan_count = self.get_parameter(
            'dlio_min_registered_scan_count'
        ).value
        self._lidar_topic = self.get_parameter('lidar_topic').value
        self._camera_topic = self.get_parameter('camera_image_topic').value
        self._registered_scan_topic = self.get_parameter('registered_scan_topic').value
        self._detection_info_topic = self.get_parameter('detection_info_topic').value
        self._detection_clouds_topic = self.get_parameter(
            'detection_clouds_topic'
        ).value
        self._prereq_timeout_sec = self.get_parameter('prereq_timeout_sec').value
        self._target_class = self.get_parameter('target_class').value
        self._min_confidence = self.get_parameter('min_confidence').value
        self._detection_frames_required = self.get_parameter(
            'detection_frames_required'
        ).value
        self._map_frame = self.get_parameter('map_frame').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._goal_topic = self.get_parameter('goal_topic').value
        self._home_threshold = self.get_parameter('home_arrival_threshold_m').value
        self._nav_home_timeout_sec = self.get_parameter('nav_home_timeout_sec').value
        self._save_dir = self.get_parameter('save_dir').value
        self._map_leaf_size = self.get_parameter('map_leaf_size').value
        self._dlio_save_pcd_service = self.get_parameter(
            'dlio_save_pcd_service'
        ).value
        self._status_topic = self.get_parameter('status_topic').value
        self._waypoint_source_topic = self.get_parameter('waypoint_source_topic').value
        self._start_exploration_topic = self.get_parameter(
            'start_exploration_topic'
        ).value

    def _set_status(self, detail: str):
        msg = String()
        msg.data = f'{self._state.name}:{detail}'
        self._status_pub.publish(msg)
        self.get_logger().info(f'[{self._state.name}] {detail}')

    def _transition(self, new_state: MissionState, detail: str = ''):
        self._state = new_state
        self._state_entered_at = self.get_clock().now()
        self._set_status(detail or new_state.name.lower())

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._state_entered_at).nanoseconds * 1e-9

    def _odom_callback(self, msg: Odometry):
        self._last_odom = msg

    def _scan_callback(self, _msg: PointCloud2):
        self._registered_scan_count += 1

    def _lidar_callback(self, _msg: PointCloud2):
        self._lidar_seen = True

    def _camera_callback(self, _msg):
        self._camera_seen = True

    def _detection_info_callback(self, msg: ObjectDetectionInfoArray):
        self._latest_detections = msg
        if self._state != MissionState.EXPLORING:
            return

        matched = self._find_target_detection(msg)
        if matched is None:
            self._detection_streak = 0
            return

        self._detection_streak += 1
        if self._detection_streak >= self._detection_frames_required:
            self._capture_detection(matched)
            self._transition(MissionState.STOP_EXPLORE, f'detected {matched.class_id}')

    def _detection_clouds_callback(self, msg: PointCloudArray):
        self._latest_detection_clouds = msg

    def _find_target_detection(
        self, msg: ObjectDetectionInfoArray
    ) -> Optional[ObjectDetectionInfo]:
        for info in msg.info:
            if info.class_id != self._target_class:
                continue
            if info.confidence < self._min_confidence:
                continue
            return info
        return None

    def _capture_detection(self, info: ObjectDetectionInfo):
        source_frame = self._latest_detections.header.frame_id
        position_map = Point(x=info.position.x, y=info.position.y, z=info.position.z)

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                source_frame,
                rclpy.time.Time(),
            )
            ps = PointStamped()
            ps.header.frame_id = source_frame
            ps.header.stamp = self._latest_detections.header.stamp
            ps.point = info.position
            position_map = do_transform_point(ps, transform).point
        except TransformException as ex:
            self.get_logger().warn(
                f'TF {source_frame}->{self._map_frame} failed, using raw point: {ex}'
            )

        clouds_in_map: List[PointCloud2] = []
        for cloud in self._latest_detection_clouds.point_clouds:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    cloud.header.frame_id,
                    rclpy.time.Time(),
                )
                clouds_in_map.append(do_transform_cloud(cloud, tf))
            except TransformException as ex:
                self.get_logger().warn(f'Skipping detection cloud transform: {ex}')

        stamp = self._latest_detections.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self._saved_detection = SavedDetection(
            class_id=info.class_id,
            confidence=float(info.confidence),
            position_map=position_map,
            source_frame=source_frame,
            stamp_sec=stamp_sec,
            point_clouds=clouds_in_map,
        )

    def _request_mode(self, mode: int) -> bool:
        if self._mode_request_pending or self._last_mode_accepted is not None:
            return False
        if not self._mode_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(
                'Waiting for /a2/set_mode (is a2 sim or pc2 bridge running?)...'
            )
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

    def _on_mode_response(self, future, requested_mode: int):
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
                f'Mode {requested_mode} accepted (now {response.current_mode}): '
                f'{response.message}'
            )
            return

        self.get_logger().error(
            f'Mode {requested_mode} rejected: {response.message}'
        )
        self._transition(MissionState.FAILED, response.message)

    def _publish_waypoint_source(self, source: str):
        msg = String()
        msg.data = source
        self._waypoint_source_pub.publish(msg)

    def _publish_home_goal(self):
        if not self._home_recorded:
            return
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

    def _tick(self):
        if self._state in (MissionState.FAILED, MissionState.DONE):
            return

        if self._state == MissionState.CHECK_PREREQS:
            if self._elapsed() > self._prereq_timeout_sec:
                self._transition(
                    MissionState.FAILED,
                    'prerequisite timeout (start a2 sim or nuc+pc2 first)',
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
            return

        if self._state == MissionState.STAND:
            if not self._mode_request_pending and self._last_mode_accepted is None:
                self._request_mode(self.MODE_STAND)
                return
            if not self._mode_response_ready():
                return
            if not self._consume_mode_response():
                return
            self._transition(MissionState.WAIT_STAND, 'stand accepted, waiting for motion')
            return

        if self._state == MissionState.WAIT_STAND:
            if self._elapsed() < self._stand_wait_sec:
                return
            self._transition(MissionState.UNLOCK)
            return

        if self._state == MissionState.UNLOCK:
            if not self._mode_request_pending and self._last_mode_accepted is None:
                self._request_mode(self.MODE_UNLOCK)
                return
            if not self._mode_response_ready():
                return
            if not self._consume_mode_response():
                return
            self._transition(MissionState.RECORD_HOME, 'unlock accepted')
            return

        if self._state == MissionState.RECORD_HOME:
            if self._mode_request_pending:
                return
            if self._last_odom is None:
                self._set_status('waiting for odometry to record home')
                return
            self._home = self._last_odom.pose.pose.position
            self._home_recorded = True
            self.get_logger().info(
                f'Home recorded at ({self._home.x:.2f}, {self._home.y:.2f}, '
                f'{self._home.z:.2f})'
            )
            self._registered_scan_count = 0
            self._transition(MissionState.WAIT_DLIO)
            return

        if self._state == MissionState.WAIT_DLIO:
            if self._elapsed() > self._dlio_ready_timeout_sec:
                self._transition(MissionState.FAILED, 'DLIO timeout')
                return
            if self._registered_scan_count < self._dlio_min_scan_count:
                self._set_status(
                    f'waiting for DLIO scans ({self._registered_scan_count}/'
                    f'{self._dlio_min_scan_count})'
                )
                return
            self._transition(MissionState.WAIT_PRE_WALK)
            return

        if self._state == MissionState.WAIT_PRE_WALK:
            if self._elapsed() < self._pre_walk_wait_sec:
                return
            self._transition(MissionState.WALK)
            return

        if self._state == MissionState.WALK:
            if not self._mode_request_pending and self._last_mode_accepted is None:
                self._request_mode(self.MODE_WALK)
                return
            if not self._mode_response_ready():
                return
            if not self._consume_mode_response():
                return
            self._transition(MissionState.START_EXPLORE, 'walk accepted')
            return

        if self._state == MissionState.START_EXPLORE:
            if self._mode_request_pending:
                return
            self._publish_waypoint_source('tare')
            explore_msg = Bool()
            explore_msg.data = True
            self._start_explore_pub.publish(explore_msg)
            self._transition(MissionState.EXPLORING, 'exploration started')
            return

        if self._state == MissionState.EXPLORING:
            return

        if self._state == MissionState.STOP_EXPLORE:
            self._publish_waypoint_source('hold')
            self._nav_home_started_at = None
            self._transition(MissionState.NAV_HOME, 'holding before homing')
            return

        if self._state == MissionState.NAV_HOME:
            if self._nav_home_started_at is None:
                self._nav_home_started_at = self.get_clock().now()
                self._publish_waypoint_source('far')
                self._publish_home_goal()
                self._set_status('navigating home')
                return

            if self._elapsed() > self._nav_home_timeout_sec:
                self._transition(MissionState.FAILED, 'nav home timeout')
                return

            dist = self._distance_to_home()
            if dist is not None and dist <= self._home_threshold:
                self._transition(MissionState.SAVE_MAP, f'home reached ({dist:.2f} m)')
            return

        if self._state == MissionState.SAVE_MAP:
            if not self._map_save_done:
                self._save_mission_artifacts()
                self._map_save_done = True
                self._publish_waypoint_source('hold')
                if not self._mode_request_pending:
                    self._request_mode(self.MODE_UNLOCK)
            if self._mode_request_pending:
                return
            self._transition(MissionState.DONE, f'map saved to {self._save_dir}')
            return

    def _save_mission_artifacts(self):
        os.makedirs(self._save_dir, exist_ok=True)

        save_ok = False
        for service_name in (
            self._dlio_save_pcd_service,
            '/dlio_map_node/save_pcd',
            '/save_pcd',
        ):
            client = self.create_client(SavePCD, service_name)
            if not client.wait_for_service(timeout_sec=2.0):
                continue
            req = SavePCD.Request()
            req.leaf_size = float(self._map_leaf_size)
            req.save_path = self._save_dir
            try:
                response = client.call(req)
            except Exception as ex:  # noqa: BLE001
                self.get_logger().warn(f'SavePCD via {service_name} failed: {ex}')
                continue
            if response.success:
                save_ok = True
                self.get_logger().info(f'DLIO map saved via {service_name}')
                break

        if not save_ok:
            self.get_logger().error('DLIO SavePCD failed on all known service names')

        objects_path = os.path.join(self._save_dir, 'detected_objects.pcd')
        if self._saved_detection and self._saved_detection.point_clouds:
            self._write_merged_pcd(objects_path, self._saved_detection.point_clouds)
        else:
            self.get_logger().warn('No detection point clouds to export')

        summary = {
            'status': 'completed' if save_ok else 'completed_with_map_save_error',
            'save_dir': self._save_dir,
            'home': {'x': self._home.x, 'y': self._home.y, 'z': self._home.z},
            'detection': None,
            'artifacts': {
                'dlio_clean_map': os.path.join(self._save_dir, 'clean_map.pcd'),
                'detected_objects': objects_path,
            },
        }
        if self._saved_detection:
            det = self._saved_detection
            summary['detection'] = {
                'class_id': det.class_id,
                'confidence': det.confidence,
                'position_map': {
                    'x': det.position_map.x,
                    'y': det.position_map.y,
                    'z': det.position_map.z,
                },
                'source_frame': det.source_frame,
                'stamp_sec': det.stamp_sec,
            }

        summary_path = os.path.join(self._save_dir, 'mission_summary.json')
        with open(summary_path, 'w', encoding='utf-8') as handle:
            json.dump(summary, handle, indent=2)
        self.get_logger().info(f'Mission summary written to {summary_path}')

    @staticmethod
    def _write_merged_pcd(path: str, clouds: List[PointCloud2]):
        points = []
        for cloud in clouds:
            for x, y, z in point_cloud2.read_points(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True
            ):
                points.append((float(x), float(y), float(z)))

        with open(path, 'w', encoding='utf-8') as handle:
            handle.write('# .PCD v0.7 - Point Cloud Data file format\n')
            handle.write('VERSION 0.7\n')
            handle.write('FIELDS x y z\n')
            handle.write('SIZE 4 4 4\n')
            handle.write('TYPE F F F\n')
            handle.write('COUNT 1 1 1\n')
            handle.write(f'WIDTH {len(points)}\n')
            handle.write('HEIGHT 1\n')
            handle.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            handle.write(f'POINTS {len(points)}\n')
            handle.write('DATA ascii\n')
            for x, y, z in points:
                handle.write(f'{x} {y} {z}\n')


def main(args=None):
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
