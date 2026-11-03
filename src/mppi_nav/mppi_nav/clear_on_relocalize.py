"""Clear the local costmap after every RViz 2D Pose Estimate.

Scans received before (re)localization are marked at the wrong map position
and the voxel layer never re-observes those cells, so they would persist.
"""

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.node import Node


class ClearOnRelocalize(Node):

    def __init__(self):
        super().__init__('clear_on_relocalize')
        # Give global_localization time to run ICP from the new initial pose.
        self.delay = float(self.declare_parameter('delay', 2.0).value)
        service = str(self.declare_parameter(
            'service', '/local_costmap/clear_entirely_local_costmap').value)
        self.client = self.create_client(ClearEntireCostmap, service)
        self.timer = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_initial_pose, 10)

    def _on_initial_pose(self, _msg):
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(self.delay, self._clear)

    def _clear(self):
        self.timer.cancel()
        self.timer = None
        if not self.client.service_is_ready():
            self.get_logger().warning(f'{self.client.srv_name} not available')
            return
        self.client.call_async(ClearEntireCostmap.Request())
        self.get_logger().info('initial pose received: local costmap cleared')


def main():
    rclpy.init()
    node = ClearOnRelocalize()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
