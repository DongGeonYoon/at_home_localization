"""Remove LiDAR returns that hit the robot itself before the local costmap.

The exclusion box is given in the robot frame (base_link). The output keeps
the input frame (FAST-LIO `body`, i.e. the sensor origin) so the costmap still
raytraces from the sensor. Points are selected with numpy on the raw buffer,
so every field of the input cloud is preserved.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf_transformations


class SelfFilter(Node):

    def __init__(self):
        super().__init__('self_filter')
        self.robot_frame = str(self.declare_parameter('robot_frame', 'base_link').value)
        # Defaults: measured 0.54 (L) x 0.50 (W) base, LiDAR front-center at
        # x = 0.31 (protrudes to ~0.35), top structure 0.93 m, plus ~5 cm margin.
        bounds = {
            'min_x': -0.32, 'max_x': 0.40,
            'min_y': -0.30, 'max_y': 0.30,
            'min_z': -0.05, 'max_z': 1.00,
        }
        self.box_min = np.array([
            float(self.declare_parameter(f'self_filter_{k}', bounds[k]).value)
            for k in ('min_x', 'min_y', 'min_z')])
        self.box_max = np.array([
            float(self.declare_parameter(f'self_filter_{k}', bounds[k]).value)
            for k in ('max_x', 'max_y', 'max_z')])
        input_topic = str(self.declare_parameter('input_topic', '/cloud_registered_body').value)
        output_topic = str(self.declare_parameter('output_topic', '/local_obstacle_cloud').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # cloud frame -> (R, t) mapping cloud points into the robot frame.
        # body -> base_link is a static mount, so it is looked up once.
        self.transforms = {}

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PointCloud2, output_topic, qos)
        self.create_subscription(PointCloud2, input_topic, self._on_cloud, qos)
        self.get_logger().info(
            f'{input_topic} -> {output_topic}, removing {self.robot_frame} box '
            f'{self.box_min.tolist()} .. {self.box_max.tolist()}')

    def _transform(self, frame):
        if frame not in self.transforms:
            try:
                tf = self.tf_buffer.lookup_transform(self.robot_frame, frame, Time())
            except tf2_ros.TransformException as error:
                self.get_logger().warning(
                    f'waiting for {self.robot_frame} <- {frame}: {error}',
                    throttle_duration_sec=2.0)
                return None
            q = tf.transform.rotation
            t = tf.transform.translation
            rotation = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            self.transforms[frame] = (rotation, np.array([t.x, t.y, t.z]))
        return self.transforms[frame]

    def _on_cloud(self, msg):
        transform = self._transform(msg.header.frame_id)
        if transform is None:
            return
        offsets = {field.name: field.offset for field in msg.fields}
        count = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(count, msg.point_step)
        dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
        xyz = np.column_stack([
            raw[:, offsets[axis]:offsets[axis] + 4].copy().view(dtype).ravel()
            for axis in ('x', 'y', 'z')])

        rotation, translation = transform
        robot = xyz @ rotation.T + translation
        inside = np.all((robot >= self.box_min) & (robot <= self.box_max), axis=1)
        kept = raw[~inside]

        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.height = 1
        out.width = len(kept)
        out.row_step = msg.point_step * len(kept)
        out.is_dense = msg.is_dense
        out.data = kept.tobytes()
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = SelfFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
