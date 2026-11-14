#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanResampler(Node):
    """Convert variable-length LaserScan messages to a fixed 450-bin scan."""

    def __init__(self) -> None:
        super().__init__('scan_resampler')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_fixed')
        self.declare_parameter('number_of_bins', 450)

        input_topic = (
            self.get_parameter('input_topic')
            .get_parameter_value()
            .string_value
        )
        output_topic = (
            self.get_parameter('output_topic')
            .get_parameter_value()
            .string_value
        )
        self.number_of_bins = (
            self.get_parameter('number_of_bins')
            .get_parameter_value()
            .integer_value
        )

        if self.number_of_bins < 10:
            raise ValueError('number_of_bins must be at least 10')

        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )

        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Resampling {input_topic} to {output_topic} '
            f'with {self.number_of_bins} fixed bins'
        )

    def scan_callback(self, input_scan: LaserScan) -> None:
        output_scan = LaserScan()

        output_scan.header = input_scan.header
        output_scan.range_min = input_scan.range_min
        output_scan.range_max = input_scan.range_max
        output_scan.scan_time = input_scan.scan_time

        number_of_bins = self.number_of_bins

        output_scan.angle_min = -math.pi
        output_scan.angle_increment = (
            2.0 * math.pi / number_of_bins
        )
        output_scan.angle_max = (
            output_scan.angle_min
            + (number_of_bins - 1)
            * output_scan.angle_increment
        )

        if output_scan.scan_time > 0.0:
            output_scan.time_increment = (
                output_scan.scan_time / number_of_bins
            )
        else:
            output_scan.time_increment = 0.0

        output_ranges = [math.inf] * number_of_bins

        for input_index, measured_range in enumerate(input_scan.ranges):
            if not math.isfinite(measured_range):
                continue

            if measured_range < input_scan.range_min:
                continue

            if measured_range > input_scan.range_max:
                continue

            angle = (
                input_scan.angle_min
                + input_index * input_scan.angle_increment
            )

            # Normalise to [-pi, pi).
            angle = (
                (angle + math.pi) % (2.0 * math.pi)
            ) - math.pi

            output_index = int(round(
                (angle - output_scan.angle_min)
                / output_scan.angle_increment
            )) % number_of_bins

            # Keep the nearest obstacle if several raw points
            # fall into the same output angular bin.
            if (
                not math.isfinite(output_ranges[output_index])
                or measured_range < output_ranges[output_index]
            ):
                output_ranges[output_index] = measured_range

        output_scan.ranges = output_ranges

        # Intensities are not required by SLAM Toolbox.
        output_scan.intensities = []

        self.publisher.publish(output_scan)


def main() -> None:
    rclpy.init()

    node = ScanResampler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
