"""Print the TF/costmap interface values the MPPI team needs.

Reads the live grids and TF, the installed costmap configs, and hashes the
map files, so the printed block always matches what is actually running.
"""

import argparse
import hashlib
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros
import yaml


MAP_TOPIC = "/map"
STATIC_TOPIC = "/mppi_static_costmap/costmap"
LOCAL_TOPIC = "/local_costmap/costmap"


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def costmap_params(file_name, name):
    path = os.path.join(get_package_share_directory("mppi_nav"), "config", file_name)
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)[name][name]["ros__parameters"]


def grid_summary(msg, rate):
    if msg is None:
        return "  (not received)"
    info = msg.info
    return (
        f"  frame       {msg.header.frame_id}\n"
        f"  resolution  {info.resolution:.3f} m/cell\n"
        f"  width       {info.width} cells ({info.width * info.resolution:.2f} m)\n"
        f"  height      {info.height} cells ({info.height * info.resolution:.2f} m)\n"
        f"  origin.x    {info.origin.position.x:.6f}\n"
        f"  origin.y    {info.origin.position.y:.6f}\n"
        f"  rate        {rate:.2f} Hz (measured)"
    )


def repo_maps_dir():
    """<repo>/maps of the checkout this file was built from.

    Walks up from this file (src/ with --symlink-install, else install/ or
    build/), so the default works wherever the repository was cloned.
    """
    path = os.path.dirname(os.path.realpath(__file__))
    while path != os.path.dirname(path):
        if all(os.path.isdir(os.path.join(path, d)) for d in ("maps", "src")):
            return os.path.join(path, "maps")
        path = os.path.dirname(path)
    return os.path.expanduser("~/at_home/at_home_localization/maps")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    maps = repo_maps_dir()
    parser.add_argument("--pcd", default=os.path.join(maps, "406.pcd"))
    parser.add_argument(
        "--map-yaml", default=os.path.join(maps, "406_new.yaml")
    )
    parser.add_argument("--duration", type=float, default=6.0)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node("costmap_interface_report")
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    latched = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    received = {MAP_TOPIC: [], STATIC_TOPIC: [], LOCAL_TOPIC: []}
    for topic in received:
        node.create_subscription(
            OccupancyGrid,
            topic,
            lambda msg, topic=topic: received[topic].append((time.monotonic(), msg)),
            latched,
        )

    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    def latest(topic):
        return received[topic][-1][1] if received[topic] else None

    def rate(topic):
        stamps = [stamp for stamp, _ in received[topic]]
        return (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 2 else 0.0

    try:
        transform = buffer.lookup_transform("map", "base_link", Time())
        t = transform.transform
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp
        pose = (
            f"  OK  x={t.translation.x:.3f} m  y={t.translation.y:.3f} m  "
            f"yaw={yaw:.3f} rad  (stamp age {age:.3f} s)"
        )
    except tf2_ros.TransformException as error:
        pose = f"  UNAVAILABLE: {error}"

    static = costmap_params("static_costmap.yaml", "mppi_static_costmap")
    local = costmap_params("local_costmap.yaml", "local_costmap")
    with open(args.map_yaml, encoding="utf-8") as stream:
        image = os.path.join(os.path.dirname(args.map_yaml), yaml.safe_load(stream)["image"])
    hashes = "\n".join(
        f"  {sha256(path)}  {path}" for path in (args.pcd, image, args.map_yaml)
    )

    print(f"""
MAP FRAME                    map
ROBOT FRAME                  base_link

TF map -> base_link
{pose}

STATIC MAP TOPIC             {MAP_TOPIC}
{grid_summary(latest(MAP_TOPIC), rate(MAP_TOPIC))}

STATIC COSTMAP TOPIC         {STATIC_TOPIC}
{grid_summary(latest(STATIC_TOPIC), rate(STATIC_TOPIC))}

LOCAL COSTMAP TOPIC          {LOCAL_TOPIC}
{grid_summary(latest(LOCAL_TOPIC), rate(LOCAL_TOPIC))}

ROBOT FOOTPRINT (base_link)  {static['footprint']}
INSCRIBED / CIRCUMSCRIBED    0.25 / 0.368 m (data 99 = inscribed only)
STATIC INFLATION RADIUS      {static['inflation_layer']['inflation_radius']} m
STATIC COST SCALING FACTOR   {static['inflation_layer']['cost_scaling_factor']}
LOCAL COSTMAP WIDTH          {local['width']} m
LOCAL COSTMAP HEIGHT         {local['height']} m
LOCAL COSTMAP UPDATE RATE    {local['update_frequency']} Hz (publish {local['publish_frequency']} Hz)
LOCAL OBSTACLE HEIGHT (map z) {local['voxel_layer']['lidar']['min_obstacle_height']} .. {local['voxel_layer']['lidar']['max_obstacle_height']} m
TF timeout recommendation    0.5 s

MAP_VERSION (sha256)
{hashes}
""")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
