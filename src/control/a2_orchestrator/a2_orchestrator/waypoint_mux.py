#!/usr/bin/env python3
"""Multiplex TARE and far_planner waypoints onto /way_point for local_planner."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

VALID_SOURCES = frozenset({'hold', 'tare', 'far'})


class WaypointMux(Node):
    def __init__(self):
        super().__init__('waypoint_mux')

        self.declare_parameter('tare_waypoint_topic', '/tare/way_point')
        self.declare_parameter('far_waypoint_topic', '/far/way_point')
        self.declare_parameter('output_waypoint_topic', '/way_point')
        self.declare_parameter('waypoint_source_topic', '/mission/waypoint_source')
        self.declare_parameter('odom_topic', '/state_estimation')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('hold_publish_rate_hz', 5.0)
        self.declare_parameter('default_source', 'hold')

        tare_topic = self.get_parameter('tare_waypoint_topic').value
        far_topic = self.get_parameter('far_waypoint_topic').value
        output_topic = self.get_parameter('output_waypoint_topic').value
        source_topic = self.get_parameter('waypoint_source_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self._map_frame = self.get_parameter('map_frame').value

        default_source = self.get_parameter('default_source').value.lower()
        if default_source not in VALID_SOURCES:
            self.get_logger().warn(
                f"Invalid default_source '{default_source}', using 'hold'"
            )
            default_source = 'hold'

        self._source = default_source
        self._last_odom = None
        self._hold_waypoint = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._waypoint_pub = self.create_publisher(PointStamped, output_topic, 10)

        self.create_subscription(
            PointStamped, tare_topic, self._tare_callback, 10
        )
        self.create_subscription(
            PointStamped, far_topic, self._far_callback, 10
        )
        self.create_subscription(String, source_topic, self._source_callback, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, qos)

        hold_rate = self.get_parameter('hold_publish_rate_hz').value
        if hold_rate <= 0.0:
            hold_rate = 5.0
        self.create_timer(1.0 / hold_rate, self._hold_timer_callback)

        self.get_logger().info(
            f"Waypoint mux active: source={self._source}, output={output_topic}"
        )

    def _odom_callback(self, msg: Odometry):
        self._last_odom = msg
        if self._source != 'hold':
            return

        self._hold_waypoint = self._waypoint_from_odom(msg)

    def _source_callback(self, msg: String):
        requested = msg.data.strip().lower()
        if requested not in VALID_SOURCES:
            self.get_logger().warn(
                f"Ignoring unknown waypoint source '{msg.data}' "
                f"(expected hold, tare, or far)"
            )
            return
        if requested == self._source:
            return

        previous = self._source
        self._source = requested

        if requested == 'hold':
            if self._last_odom is not None:
                self._hold_waypoint = self._waypoint_from_odom(self._last_odom)
                self._publish_waypoint(self._hold_waypoint)

        self.get_logger().info(
            f"Waypoint source changed: {previous} -> {requested}"
        )

    def _tare_callback(self, msg: PointStamped):
        if self._source != 'tare':
            return
        self._publish_waypoint(msg)

    def _far_callback(self, msg: PointStamped):
        if self._source != 'far':
            return
        self._publish_waypoint(msg)

    def _hold_timer_callback(self):
        if self._source != 'hold':
            return
        if self._hold_waypoint is None:
            return
        self._hold_waypoint.header.stamp = self.get_clock().now().to_msg()
        self._publish_waypoint(self._hold_waypoint)

    def _waypoint_from_odom(self, odom: Odometry) -> PointStamped:
        waypoint = PointStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = self._map_frame
        waypoint.point = odom.pose.pose.position
        return waypoint

    def _publish_waypoint(self, msg: PointStamped):
        self._waypoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
