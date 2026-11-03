#!/usr/bin/env python3

import threading
from collections import deque

import open3d as o3d
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
# from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import numpy as np
import tf2_ros
import tf_transformations
import ros2_numpy


class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")
        self.global_map = None
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan_msg = None
        # Recent /cloud_registered scans; they share the odom frame, so they
        # stack into one denser cloud without any transform.
        self.scan_buffer = deque(maxlen=5)
        self.initialized = False
        # T_map_to_odom / cur_scan_msg are shared between the scan thread and
        # the ICP thread.
        self.lock = threading.Lock()

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.1),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 0.5),
                ("freq_global_map", 0.25),
                # Fraction of scan points within gate_distance of the map after
                # ICP. ICP's own fitness uses its (up to 1.0 m) correspondence
                # distance and reads ~1.0 even when misaligned.
                # Relocalization (a new initial pose) searches widely, so it
                # needs the strict gate; a wrong local minimum scored up to 0.87.
                ("localization_threshold", 0.85),
                # Tracking starts from the current estimate and is also bounded
                # by max_correction_*; areas changed since mapping read 0.83-0.86
                # while aligned, which the strict gate kept rejecting.
                ("tracking_threshold", 0.75),
                ("gate_distance", 0.25),
                # Tracking updates larger than this are rejected as ICP jumps.
                ("max_correction_m", 0.5),
                ("max_correction_deg", 10.0),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("pcd_map_topic", "/map"),
                ("pcd_map_path", ""),
            ],
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # self.pub_global_map = self.create_publisher(PointCloud2, self.get_parameter("pcd_map_topic").value, 10)
        self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 10)

        self.get_logger().info("Waiting for global map...")
        # global_map_msg = wait_for_message(msg_type = PointCloud2, node = self, topic = "/cloud_pcd")[1]
        # self.initialize_global_map(global_map_msg)

        self.initialize_global_map()
        self.get_logger().info("Global map received.")

        # Scans/odometry are handled in their own group so that ICP (0.3 s+)
        # never blocks them, and only the newest message is kept: a deeper
        # queue replays stale scans after every ICP run.
        sensor_group = MutuallyExclusiveCallbackGroup()
        icp_group = MutuallyExclusiveCallbackGroup()
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_cur_scan = self.create_subscription(
            PointCloud2, "/cloud_registered", self.cb_save_cur_scan, latest, callback_group=sensor_group)
        self.sub_cur_odom = self.create_subscription(
            Odometry, "/Odometry", self.cb_save_cur_odom, latest, callback_group=sensor_group)
        self.sub_initial_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.cb_initialize_pose,
            10,
            callback_group=icp_group,
        )

        self.timer_localisation = self.create_timer(
            1.0 / self.get_parameter("freq_localization").value, self.localisation_timer_callback,
            callback_group=icp_group)
        # self.timer_global_map = self.create_timer(1/ self.get_parameter("freq_global_map").value, self.global_map_callback)

    def global_map_callback(self):
        # self.get_logger().info(np.array(self.global_map.points).shape)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_global_map, header, np.array(self.global_map.points))

    def pose_to_mat(self, pose):
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans

    def msg_to_array(self, pc_msg):
        """xyz (N x 3, float64, finite rows only) straight from the PointCloud2 buffer."""
        offsets = {field.name: field.offset for field in pc_msg.fields}
        count = pc_msg.width * pc_msg.height
        raw = np.frombuffer(pc_msg.data, dtype=np.uint8).reshape(count, pc_msg.point_step)
        dtype = np.dtype('>f4' if pc_msg.is_bigendian else '<f4')
        xyz = np.column_stack([
            raw[:, offsets[axis]:offsets[axis] + 4].copy().view(dtype).ravel()
            for axis in ('x', 'y', 'z')]).astype(np.float64)
        return xyz[np.isfinite(xyz).all(axis=1)]

    def scan_to_o3d(self, pc_msg):
        scan = o3d.geometry.PointCloud()
        scan.points = o3d.utility.Vector3dVector(self.msg_to_array(pc_msg))
        return scan

    def registration_multiscale(self, scan, map, initial, distances, max_iteration):
        """Point-to-plane ICP with a shrinking correspondence distance.

        A single point-to-point pass with a wide (1-5 m) distance lets wrong
        correspondences pull the result off; the robust loss and the shrinking
        distance keep only the near-wall matches at the end.
        """
        reg = o3d.pipelines.registration
        transformation = initial
        for distance in distances:
            transformation = reg.registration_icp(
                scan, map, distance, transformation,
                reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=distance / 2.0)),
                reg.ICPConvergenceCriteria(max_iteration=max_iteration),
            ).transformation
        return transformation

    def gate_fitness(self, scan, map, transformation):
        return o3d.pipelines.registration.evaluate_registration(
            scan, map, self.get_parameter("gate_distance").value, transformation).fitness

    def yaw_offset(self, trans, yaw):
        """trans rotated by yaw about the body's current position in map."""
        T_map_to_body = np.matmul(trans, self.pose_to_mat(self.cur_odom.pose.pose))
        rotation = tf_transformations.rotation_matrix(yaw, (0, 0, 1), T_map_to_body[:3, 3])
        return np.matmul(rotation, trans)

    def inverse_se3(self, trans):
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    def publish_point_cloud(self, publisher, header, pc):
        pc = np.asarray(pc)
        if pc.size == 0:
            return

        if pc.ndim == 1:
            pc = pc.reshape(1, -1)

        data = dict()
        data["xyz"] = pc[:, :3]

        if pc.shape[1] == 4:
            data["intensity"] = pc[:, 3]
        # else:
            # data["rgb"] = np.ones_like(pc)
        msg = ros2_numpy.msgify(PointCloud2, data)
        msg.header = header
        if len(msg.fields) == 4:
            msg.point_step = 16
        else:
            msg.point_step = 12

        publisher.publish(msg)

    def transform_points(self, points, transform):
        if points.size == 0:
            return points

        points = np.asarray(points)
        if points.ndim == 1:
            points = points.reshape(1, -1)

        points_h = np.column_stack((points[:, :3], np.ones(points.shape[0])))
        points_in_target = np.matmul(transform, points_h.T).T[:, :3]

        if points.shape[1] > 3:
            return np.column_stack((points_in_target, points[:, 3:]))

        return points_in_target

    def publish_current_scan_in_map(self):
        with self.lock:
            scan_msg = self.cur_scan_msg
            T_map_to_odom = self.T_map_to_odom.copy()
        if scan_msg is None:
            return

        points = self.msg_to_array(scan_msg)
        if points.size == 0:
            return
        scan_in_map = (points @ T_map_to_odom[:3, :3].T + T_map_to_odom[:3, 3]).astype(np.float32)

        msg = PointCloud2()
        msg.header.stamp = scan_msg.header.stamp
        msg.header.frame_id = "map"
        msg.height = 1
        msg.width = len(scan_in_map)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(scan_in_map)
        msg.is_dense = True
        msg.data = scan_in_map.tobytes()
        self.pub_pc_in_map.publish(msg)

    def crop_global_map_in_FOV(self, pose_estimation):
        T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        fov = self.get_parameter("fov").value
        fov_far = self.get_parameter("fov_far").value
        planar_dist = np.linalg.norm(global_map_in_base_link[:, :2], axis=1)
        yaw = np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])

        if fov >= 2.0 * np.pi - 1e-3:
            indices = np.where(planar_dist < fov_far)
        else:
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0)
                & (planar_dist < fov_far)
                & (np.abs(yaw) < fov / 2.0)
            )
        # select_by_index keeps the normals point-to-plane ICP needs.
        global_map_in_FOV = self.global_map.select_by_index(indices[0])

        header = Header()
        header.stamp = self.cur_odom.header.stamp
        header.frame_id = "map"
        submap_points = np.array(global_map_in_FOV.points)
        if submap_points.size > 0:
            self.publish_point_cloud(self.pub_submap, header, submap_points[::10])

        return global_map_in_FOV


    def global_localization(self, pose_estimation, relocalize=False):
        """Register the recent scans to the map; True when the result was applied."""
        with self.lock:
            scan_msgs = list(self.scan_buffer)
        if self.cur_odom is None or not scan_msgs:
            return False

        if self.global_map is None or self.global_map.is_empty():
            self.get_logger().warn("Global map is empty. Cannot run localization.")
            return False

        scan_tobe_mapped = o3d.geometry.PointCloud()
        scan_tobe_mapped.points = o3d.utility.Vector3dVector(
            np.vstack([self.msg_to_array(msg) for msg in scan_msgs]))
        scan_tobe_mapped = self.voxel_down_sample(
            scan_tobe_mapped, self.get_parameter("scan_voxel_size").value)
        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)

        if global_map_in_FOV.is_empty():
            self.get_logger().warn("No map points in current FOV. Check the initial pose or fov_far.")
            return False

        if relocalize:
            # A clicked pose is often 10-20 deg off; one ICP from it lands in a
            # wrong local minimum. Try several headings, keep the best.
            best = None
            for yaw_deg in (0.0, -15.0, 15.0, -30.0, 30.0):
                candidate = self.registration_multiscale(
                    scan_tobe_mapped, global_map_in_FOV,
                    self.yaw_offset(pose_estimation, np.deg2rad(yaw_deg)), (1.5, 0.75), 15)
                score = o3d.pipelines.registration.evaluate_registration(
                    scan_tobe_mapped, global_map_in_FOV, 0.5, candidate).fitness
                if best is None or score > best[1]:
                    best = (candidate, score)
            transformation = self.registration_multiscale(
                scan_tobe_mapped, global_map_in_FOV, best[0], (0.5, 0.25), 30)
        else:
            transformation = self.registration_multiscale(
                scan_tobe_mapped, global_map_in_FOV, pose_estimation, (1.0, 0.5, 0.25), 30)

        fitness = self.gate_fitness(scan_tobe_mapped, global_map_in_FOV, transformation)
        correction = np.matmul(self.inverse_se3(pose_estimation), transformation)
        correction_m = np.linalg.norm(correction[:2, 3])
        correction_deg = abs(np.rad2deg(np.arctan2(correction[1, 0], correction[0, 0])))

        self.get_logger().info(
            f"ICP {'relocalization' if relocalize else 'tracking'}: "
            f"fitness@{self.get_parameter('gate_distance').value:.2f}m {fitness:.3f}, "
            f"correction {correction_m:.3f} m / {correction_deg:.2f} deg"
        )

        threshold = self.get_parameter(
            "localization_threshold" if relocalize else "tracking_threshold").value
        if fitness < threshold:
            self.get_logger().warn(
                f"Fitness score {fitness:.3f} less than threshold {threshold}. "
                "Keeping the previous estimate."
            )
            return False
        if not relocalize and (
            correction_m > self.get_parameter("max_correction_m").value
            or correction_deg > self.get_parameter("max_correction_deg").value
        ):
            self.get_logger().warn(
                f"Rejected ICP jump of {correction_m:.3f} m / {correction_deg:.2f} deg."
            )
            return False

        with self.lock:
            self.T_map_to_odom = transformation
        self.publish_odom(transformation)
        return True

    def voxel_down_sample(self, pcd, voxel_size):
        # print(pcd)

        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)

        except Exception as e:
            # for opend3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)

        return pcd_down

    def cb_save_cur_odom(self, msg):
        self.cur_odom = msg

    def cb_save_cur_scan(self, msg):
        if msg.width * msg.height == 0:
            self.get_logger().warn("Received an empty /cloud_registered scan.")
            return
        with self.lock:
            self.cur_scan_msg = msg
            self.scan_buffer.append(msg)
        self.publish_current_scan_in_map()

    def initialize_global_map(self): #, pc_msg):
        # self.global_map = o3d.geometry.PointCloud()
        # self.global_map.points = o3d.utility.Vector3dVector(self.msg_to_array(pc_msg)[:, :3])
        pcd_map_path = self.get_parameter("pcd_map_path").value
        if not pcd_map_path:
            self.get_logger().error("pcd_map_path is empty. Pass map:=/path/to/map.pcd in the launch command.")
            self.global_map = o3d.geometry.PointCloud()
            return

        self.global_map = o3d.io.read_point_cloud(pcd_map_path)
        if self.global_map.is_empty():
            self.get_logger().error(f"Failed to read PCD map or map is empty: {pcd_map_path}")
            return

        self.global_map = self.voxel_down_sample(self.global_map, self.get_parameter("map_voxel_size").value)
        self.global_map.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
        # o3d.io.write_point_cloud("/home/wheelchair2/laksh_ws/pcds/lab_map_with_outside_corridor (with ground pcd)_downsampled.pcd", self.global_map)
        self.get_logger().info(f"Global map loaded: {pcd_map_path}")

    def cb_initialize_pose(self, msg):

        if self.cur_odom is None:
            self.get_logger().warn(
                "Odometry not received yet. Cannot initialize pose."
            )
            return

        # RViz 2D Pose Estimate is the robot (base_link) pose in map, flat on
        # z = 0. FAST-LIO odometry is the body (IMU) pose, 0.32 m ahead of and
        # 0.40 m above base_link, so the click is moved to body first.
        T_map_to_base = self.pose_to_mat(msg.pose.pose)
        T_map_to_body = np.matmul(T_map_to_base, self.inverse_se3(self.lookup_body_to_base()))
        yaw = np.arctan2(T_map_to_body[1, 0], T_map_to_body[0, 0])

        # Current FAST-LIO pose: body pose in camera_init (odom) frame.
        T_odom_to_body = self.pose_to_mat(self.cur_odom.pose.pose)

        # Only x, y and yaw come from the click. Height, roll and pitch stay
        # with the current estimate: the map and camera_init both start at the
        # IMU on the same floor, so body z is ~0 in either, not the clicked 0.
        roll, pitch, _ = tf_transformations.euler_from_matrix(T_odom_to_body)
        with self.lock:
            T_map_to_odom_prev = self.T_map_to_odom.copy()
        body_z = np.matmul(T_map_to_odom_prev, T_odom_to_body)[2, 3]
        body_xy = T_map_to_body[:2, 3]
        T_map_to_body = tf_transformations.euler_matrix(roll, pitch, yaw)
        T_map_to_body[:3, 3] = [body_xy[0], body_xy[1], body_z]

        # T_map_body = T_map_odom * T_odom_body  =>  T_map_odom = T_map_body * inv(T_odom_body)
        T_map_to_odom_init = np.matmul(T_map_to_body, self.inverse_se3(T_odom_to_body))

        self.get_logger().info("Initial pose received. Searching around it...")

        # The click is applied only once ICP confirms it. Publishing it first
        # put map->base_link wherever a mis-click landed (15 m off in one test
        # run) until the next good click, and MPPI drives on that pose.
        if self.global_localization(T_map_to_odom_init, relocalize=True):
            self.initialized = True
            self.publish_current_scan_in_map()
        else:
            self.get_logger().warn(
                "Initial pose rejected; keeping the previous estimate. "
                "Set the initial pose again closer to the real pose."
            )

    def lookup_body_to_base(self):
        """base_link pose in body; identity when no body->base_link TF exists."""
        try:
            t = self.tf_buffer.lookup_transform("body", "base_link", Time()).transform
        except tf2_ros.TransformException:
            self.get_logger().warn("No body->base_link TF; using the clicked pose as body.", once=True)
            return np.eye(4)
        trans = tf_transformations.quaternion_matrix(
            [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        trans[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return trans

    def publish_odom(self, transform):
        odom_msg = Odometry()
        xyz = transform[:3, 3]
        quat = tf_transformations.quaternion_from_matrix(transform)
        odom_msg.pose.pose = Pose(
            position = Point(x = xyz[0], y = xyz[1], z = xyz[2]),
            orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
        )
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "camera_init"
        self.pub_map_to_odom.publish(odom_msg)

    def localisation_timer_callback(self):
        if not self.initialized:
            self.get_logger().info("Waiting for initial pose...")
            return

        if self.cur_odom is None:
            self.get_logger().info("Waiting for odometry...")
            return

        with self.lock:
            has_scan = self.cur_scan_msg is not None
            T_map_to_odom = self.T_map_to_odom.copy()
        if not has_scan:
            self.get_logger().info("Waiting for current scan...")
            return

        self.global_localization(T_map_to_odom)


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
