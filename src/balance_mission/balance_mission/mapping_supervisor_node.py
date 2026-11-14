#!/usr/bin/env python3
"""Step 1 helper: save the map as soon as autonomous exploration finishes.

* Listens to /explore/status (explore_lite).  On EXPLORATION_COMPLETE the map
  is written to ``map_save_path`` (.yaml + .pgm) so Step 2 can load it.
* ``ros2 service call /mapping/save_map std_srvs/srv/Trigger`` saves on demand
  (use it if you drive the car by hand, or want an extra checkpoint).
* Every save also keeps a time-stamped copy, so a bad second save can never
  destroy a good map.
"""

import os
import subprocess
import time

import rclpy
from explore_lite_msgs.msg import ExploreStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger


class MappingSupervisor(Node):

    def __init__(self) -> None:
        super().__init__('mapping_supervisor')

        self.declare_parameter('map_save_path', '~/maps/sandbox_map')
        self.declare_parameter('auto_save_on_complete', True)
        self.declare_parameter('save_timeout_s', 8.0)
        self.declare_parameter('save_retries', 3)

        self.map_path = os.path.expanduser(
            self.get_parameter('map_save_path').value)
        self.auto_save = bool(
            self.get_parameter('auto_save_on_complete').value)
        self.save_timeout = float(self.get_parameter('save_timeout_s').value)
        self.retries = int(self.get_parameter('save_retries').value)

        self.saved_after_complete = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ExploreStatus, '/explore/status', self.status_callback, qos)
        self.create_service(Trigger, '/mapping/save_map', self.save_service)

        self.get_logger().info(
            f'Mapping supervisor ready. Map will be saved to {self.map_path}'
            f'.yaml (auto_save_on_complete={self.auto_save}).')

    def status_callback(self, msg: ExploreStatus) -> None:
        self.get_logger().info(f'Explore status: {msg.status}')
        if (msg.status == ExploreStatus.EXPLORATION_COMPLETE
                and self.auto_save and not self.saved_after_complete):
            self.saved_after_complete = True
            if self.save_map():
                self.get_logger().info(
                    'MAPPING DONE: map saved. The car returns to its start '
                    'pose. You can now stop this launch and start Step 2.')

    def save_service(self, _req, res):
        ok = self.save_map()
        res.success = ok
        res.message = (f'Saved {self.map_path}.yaml' if ok
                       else 'Map save failed (see log)')
        return res

    def save_map(self) -> bool:
        directory = os.path.dirname(self.map_path) or '.'
        os.makedirs(directory, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        targets = [self.map_path, f'{self.map_path}_{stamp}']

        for attempt in range(1, self.retries + 1):
            all_ok = True
            for target in targets:
                command = [
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '-f', target,
                    '--ros-args',
                    '-p', f'save_map_timeout:={self.save_timeout}',
                ]
                try:
                    result = subprocess.run(
                        command, capture_output=True, text=True,
                        timeout=self.save_timeout + 10.0, check=False)
                    ok = (result.returncode == 0
                          and os.path.exists(f'{target}.yaml'))
                except (subprocess.TimeoutExpired, OSError) as error:
                    self.get_logger().error(f'map_saver_cli failed: {error}')
                    ok = False
                all_ok = all_ok and ok
            if all_ok:
                self.get_logger().info(
                    f'Map saved: {self.map_path}.yaml (copy: '
                    f'{self.map_path}_{stamp}.yaml)')
                return True
            self.get_logger().warning(
                f'Map save attempt {attempt}/{self.retries} failed.')
        self.get_logger().error('Map could NOT be saved.')
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
