#!/usr/bin/env python3
import json
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


class ExplorationNode(Node):
    """Frontier detector and goal candidate publisher.

    The node follows the courseware data flow:
    /map -> frontier exploration -> /exploration/goal_pose.
    The state manager decides when to send the goal to Nav2.
    """

    UNKNOWN = -1

    def __init__(self):
        super().__init__("exploration_node")

        self.declare_parameter("enabled_on_start", False)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_topic", "/exploration/goal_pose")
        self.declare_parameter("status_topic", "/exploration/status")
        self.declare_parameter("direct_nav_goal_topic", "/goal_pose")
        self.declare_parameter("publish_direct_nav_goal", False)
        self.declare_parameter("timer_period_sec", 1.0)
        self.declare_parameter("goal_interval_sec", 4.0)
        self.declare_parameter("free_threshold", 10)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("min_frontier_cluster_size", 6)
        self.declare_parameter("min_goal_distance", 0.6)
        self.declare_parameter("info_radius_m", 0.8)
        self.declare_parameter("information_weight", 1.0)
        self.declare_parameter("distance_weight", 0.35)
        self.declare_parameter("status_log_period_sec", 3.0)

        self.enabled = bool(self.get_parameter("enabled_on_start").value)
        self.map_msg = None
        self.robot_pose = None
        self.last_goal_time = self.get_clock().now()
        self.last_status_log_time = self.get_clock().now()
        self.last_goal = None

        map_topic = self.get_parameter("map_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        goal_topic = self.get_parameter("goal_topic").value
        status_topic = self.get_parameter("status_topic").value
        direct_goal_topic = self.get_parameter("direct_nav_goal_topic").value
        timer_period = float(self.get_parameter("timer_period_sec").value)

        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        self.enable_sub = self.create_subscription(
            Bool, "/exploration/enable", self.enable_callback, 10
        )

        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.direct_goal_pub = self.create_publisher(PoseStamped, direct_goal_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Frontier exploration node ready")

    def map_callback(self, msg):
        self.map_msg = msg

    def odom_callback(self, msg):
        self.robot_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def enable_callback(self, msg):
        self.enabled = bool(msg.data)

    def timer_callback(self):
        if not self.enabled:
            self.publish_status({"event": "IDLE", "enabled": False}, log=False)
            return

        if self.map_msg is None:
            self.publish_status({"event": "WAITING_FOR_MAP", "enabled": True})
            return

        if self.robot_pose is None:
            self.publish_status({"event": "WAITING_FOR_ODOM", "enabled": True})
            return

        frontiers = self.detect_frontiers(self.map_msg)
        clusters = self.cluster_frontiers(frontiers)
        clusters = [
            cluster
            for cluster in clusters
            if len(cluster) >= int(self.get_parameter("min_frontier_cluster_size").value)
        ]

        if not clusters:
            self.publish_status(
                {
                    "event": "DONE",
                    "enabled": True,
                    "frontier_cells": len(frontiers),
                    "frontier_clusters": 0,
                }
            )
            return

        now = self.get_clock().now()
        goal_interval = float(self.get_parameter("goal_interval_sec").value)
        if (now - self.last_goal_time).nanoseconds * 1e-9 < goal_interval:
            self.publish_status(
                {
                    "event": "WAITING_FOR_INTERVAL",
                    "enabled": True,
                    "frontier_cells": len(frontiers),
                    "frontier_clusters": len(clusters),
                },
                log=False,
            )
            return

        candidate = self.select_goal(clusters)
        if candidate is None:
            self.publish_status(
                {
                    "event": "NO_REACHABLE_FRONTIER",
                    "enabled": True,
                    "frontier_cells": len(frontiers),
                    "frontier_clusters": len(clusters),
                }
            )
            return

        x, y, score, info_gain, cost, cluster_size = candidate
        msg = self.make_goal_msg(x, y)
        self.goal_pub.publish(msg)
        if bool(self.get_parameter("publish_direct_nav_goal").value):
            self.direct_goal_pub.publish(msg)

        self.last_goal = (x, y)
        self.last_goal_time = now
        self.publish_status(
            {
                "event": "GOAL_SENT",
                "enabled": True,
                "goal": {"x": x, "y": y},
                "score": score,
                "information_gain": info_gain,
                "cost": cost,
                "cluster_size": cluster_size,
                "frontier_cells": len(frontiers),
                "frontier_clusters": len(clusters),
            },
            force_log=True,
        )

    def detect_frontiers(self, map_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        data = map_msg.data
        free_threshold = int(self.get_parameter("free_threshold").value)
        frontiers = set()

        for y in range(1, height - 1):
            row_offset = y * width
            for x in range(1, width - 1):
                idx = row_offset + x
                if data[idx] < 0 or data[idx] > free_threshold:
                    continue
                if self.has_unknown_neighbor(data, width, x, y):
                    frontiers.add((x, y))

        return frontiers

    def has_unknown_neighbor(self, data, width, x, y):
        for ny in range(y - 1, y + 2):
            for nx in range(x - 1, x + 2):
                if nx == x and ny == y:
                    continue
                if data[ny * width + nx] == self.UNKNOWN:
                    return True
        return False

    def cluster_frontiers(self, frontiers):
        remaining = set(frontiers)
        clusters = []

        while remaining:
            start = remaining.pop()
            cluster = [start]
            queue = deque([start])
            while queue:
                cx, cy = queue.popleft()
                for ny in range(cy - 1, cy + 2):
                    for nx in range(cx - 1, cx + 2):
                        neighbor = (nx, ny)
                        if neighbor in remaining:
                            remaining.remove(neighbor)
                            queue.append(neighbor)
                            cluster.append(neighbor)
            clusters.append(cluster)

        return clusters

    def select_goal(self, clusters):
        rx, ry, _ = self.robot_pose
        min_goal_distance = float(self.get_parameter("min_goal_distance").value)
        alpha = float(self.get_parameter("information_weight").value)
        beta = float(self.get_parameter("distance_weight").value)

        best = None
        for cluster in clusters:
            cx = sum(cell[0] for cell in cluster) / len(cluster)
            cy = sum(cell[1] for cell in cluster) / len(cluster)
            wx, wy = self.map_to_world(self.map_msg, cx, cy)
            cost = math.hypot(wx - rx, wy - ry)
            if cost < min_goal_distance:
                continue
            info_gain = self.estimate_information_gain(cx, cy)
            score = alpha * info_gain - beta * cost
            candidate = (wx, wy, score, info_gain, cost, len(cluster))
            if best is None or score > best[2]:
                best = candidate

        return best

    def estimate_information_gain(self, mx, my):
        map_msg = self.map_msg
        resolution = map_msg.info.resolution
        radius_m = float(self.get_parameter("info_radius_m").value)
        radius_cells = max(1, int(radius_m / resolution))
        width = map_msg.info.width
        height = map_msg.info.height
        data = map_msg.data
        center_x = int(round(mx))
        center_y = int(round(my))
        count = 0

        for y in range(max(0, center_y - radius_cells), min(height, center_y + radius_cells + 1)):
            for x in range(max(0, center_x - radius_cells), min(width, center_x + radius_cells + 1)):
                if data[y * width + x] == self.UNKNOWN:
                    count += 1

        return float(count)

    def map_to_world(self, map_msg, mx, my):
        origin = map_msg.info.origin
        resolution = map_msg.info.resolution
        yaw = yaw_from_quaternion(origin.orientation)
        local_x = (mx + 0.5) * resolution
        local_y = (my + 0.5) * resolution
        world_x = origin.position.x + math.cos(yaw) * local_x - math.sin(yaw) * local_y
        world_y = origin.position.y + math.sin(yaw) * local_x + math.cos(yaw) * local_y
        return world_x, world_y

    def make_goal_msg(self, x, y):
        rx, ry, _ = self.robot_pose
        yaw = math.atan2(y - ry, x - rx)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def publish_status(self, payload, log=True, force_log=False):
        payload["stamp"] = self.get_clock().now().nanoseconds * 1e-9
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

        if not log and not force_log:
            return

        now = self.get_clock().now()
        period = float(self.get_parameter("status_log_period_sec").value)
        elapsed = (now - self.last_status_log_time).nanoseconds * 1e-9
        if force_log or elapsed >= period:
            self.last_status_log_time = now
            self.get_logger().info(f"[Exploration] {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
