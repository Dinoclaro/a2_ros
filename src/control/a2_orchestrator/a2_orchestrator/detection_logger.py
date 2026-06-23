#!/usr/bin/env python3
"""Log object detections to CSV with positions transformed to map frame."""

import csv
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from object_detection_msgs.msg import ObjectDetectionInfoArray
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener, TransformException


class DetectionLogger(Node):
    def __init__(self):
        super().__init__('detection_logger')

        self.declare_parameter('output_csv', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('detection_info_topic', '/detection_info')

        self._output_csv = self.get_parameter('output_csv').value
        self._map_frame = self.get_parameter('map_frame').value
        self._detection_info_topic = self.get_parameter('detection_info_topic').value

        if not self._output_csv:
            raise ValueError('output_csv parameter is required')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._csv_header_written = (
            os.path.isfile(self._output_csv) and os.path.getsize(self._output_csv) > 0
        )

        self.create_subscription(
            ObjectDetectionInfoArray,
            self._detection_info_topic,
            self._detection_info_callback,
            10,
        )

        self.get_logger().info(
            f'Logging detections to {self._output_csv} '
            f'(map_frame={self._map_frame}, topic={self._detection_info_topic})'
        )

    def _detection_info_callback(self, msg: ObjectDetectionInfoArray):
        source_frame = msg.header.frame_id
        stamp_sec = (
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        )

        try:
            with open(self._output_csv, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                if not self._csv_header_written:
                    writer.writerow(
                        ['timestamp_sec', 'class_id', 'confidence', 'x', 'y', 'z']
                    )
                    self._csv_header_written = True

                for info in msg.info:
                    x, y, z = self._position_in_map(
                        info.position, source_frame, msg.header.stamp
                    )
                    writer.writerow(
                        [
                            stamp_sec,
                            info.class_id,
                            float(info.confidence),
                            x,
                            y,
                            z,
                        ]
                    )
                    csvfile.flush()
        except OSError as ex:
            self.get_logger().error(f'Failed to write CSV: {ex}')

    def _position_in_map(self, position, source_frame, stamp):
        x = float(position.x)
        y = float(position.y)
        z = float(position.z)

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                source_frame,
                rclpy.time.Time(),
            )
            ps = PointStamped()
            ps.header.frame_id = source_frame
            ps.header.stamp = stamp
            ps.point = position
            transformed = do_transform_point(ps, transform).point
            return transformed.x, transformed.y, transformed.z
        except TransformException as ex:
            self.get_logger().warn(
                f'TF {source_frame}->{self._map_frame} failed, using raw point: {ex}'
            )
            return x, y, z


def main(args=None):
    rclpy.init(args=args)
    node = DetectionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
