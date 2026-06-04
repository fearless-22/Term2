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
        self.declare_parameter("failed_goal_topic", "/exploration/failed_goal")
        self.declare_parameter("direct_nav_goal_topic", "/goal_pose")
        self.declare_parameter("publish_direct_nav_goal", False)
        self.declare_parameter("timer_period_sec", 0.5)
        self.declare_parameter("goal_interval_sec", 1.0)
        self.declare_parameter("free_threshold", 10)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("min_frontier_cluster_size", 6)
        self.declare_parameter("min_goal_distance", 0.35)
        self.declare_parameter("bootstrap_min_goal_distance", 0.28)
        self.declare_parameter("goal_inset_m", 0.30)
        self.declare_parameter("goal_search_radius_m", 0.70)
        self.declare_parameter("obstacle_clearance_m", 0.20)
        self.declare_parameter("max_frontier_samples_per_cluster", 32)
        self.declare_parameter("max_frontier_clusters_to_score", 80)
        self.declare_parameter("failed_goal_blacklist_radius_m", 1.20)
        self.declare_parameter("failed_goal_blacklist_ttl_sec", 120.0)
        self.declare_parameter("failed_goal_blacklist_max_entries", 30)
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
        self.failed_goal_blacklist = []

        map_topic = self.get_parameter("map_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        goal_topic = self.get_parameter("goal_topic").value
        status_topic = self.get_parameter("status_topic").value
        failed_goal_topic = self.get_parameter("failed_goal_topic").value
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
        self.failed_goal_sub = self.create_subscription(
            PoseStamped, failed_goal_topic, self.failed_goal_callback, 10
        )

        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.direct_goal_pub = self.create_publisher(PoseStamped, direct_goal_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Frontier exploration node ready")

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

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

    def failed_goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        now = self.now_sec()
        radius = float(self.get_parameter("failed_goal_blacklist_radius_m").value)
        updated = False

        for index, (goal_x, goal_y, _) in enumerate(self.failed_goal_blacklist):
            if math.hypot(x - goal_x, y - goal_y) <= radius * 0.5:
                self.failed_goal_blacklist[index] = (x, y, now)
                updated = True
                break

        if not updated:
            self.failed_goal_blacklist.append((x, y, now))

        self.prune_failed_goal_blacklist()
        self.get_logger().warn(
            f"[Exploration] blacklisted failed goal near ({x:.2f}, {y:.2f})"
        )

    def timer_callback(self):
        self.prune_failed_goal_blacklist()

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
                    "blacklisted_goals": len(self.failed_goal_blacklist),
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
                    "blacklisted_goals": len(self.failed_goal_blacklist),
                },
                log=False,
            )
            return

        candidate, scored_clusters = self.select_goal(clusters)
        if candidate is None:
            self.publish_status(
                {
                    "event": "NO_REACHABLE_FRONTIER",
                    "enabled": True,
                    "frontier_cells": len(frontiers),
                    "frontier_clusters": len(clusters),
                    "scored_frontier_clusters": scored_clusters,
                    "blacklisted_goals": len(self.failed_goal_blacklist),
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
                "scored_frontier_clusters": scored_clusters,
                "blacklisted_goals": len(self.failed_goal_blacklist),
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
        robot_mx, robot_my = self.world_to_map(self.map_msg, rx, ry)
        if robot_mx is None:
            return None, 0

        clusters = self.prioritize_frontier_clusters(clusters, robot_mx, robot_my)

        min_goal_distance = float(self.get_parameter("min_goal_distance").value)
        bootstrap_min_goal_distance = float(
            self.get_parameter("bootstrap_min_goal_distance").value
        )
        alpha = float(self.get_parameter("information_weight").value)
        beta = float(self.get_parameter("distance_weight").value)

        best = None
        bootstrap_best = None
        for cluster in clusters:
            for fx, fy in self.sample_frontier_cells(cluster, robot_mx, robot_my):
                safe_cell = self.find_safe_goal_cell(fx, fy, robot_mx, robot_my)
                if safe_cell is None:
                    continue

                wx, wy = self.map_to_world(self.map_msg, safe_cell[0], safe_cell[1])
                if self.is_goal_blacklisted(wx, wy):
                    continue

                cost = math.hypot(wx - rx, wy - ry)
                info_gain = self.estimate_information_gain(fx, fy)
                score = alpha * info_gain - beta * cost
                candidate = (wx, wy, score, info_gain, cost, len(cluster))
                if cost < min_goal_distance:
                    if cost >= bootstrap_min_goal_distance and (
                        bootstrap_best is None or cost > bootstrap_best[4]
                    ):
                        bootstrap_best = candidate
                    continue
                if best is None or score > best[2]:
                    best = candidate

        return best or bootstrap_best, len(clusters)

    def prioritize_frontier_clusters(self, clusters, robot_mx, robot_my):
        max_clusters = int(self.get_parameter("max_frontier_clusters_to_score").value)
        if max_clusters <= 0 or len(clusters) <= max_clusters:
            return clusters

        return sorted(
            clusters,
            key=lambda cluster: self.frontier_cluster_distance(
                cluster, robot_mx, robot_my
            ),
        )[:max_clusters]

    def frontier_cluster_distance(self, cluster, robot_mx, robot_my):
        if not cluster:
            return float("inf")
        sum_x = 0
        sum_y = 0
        for x, y in cluster:
            sum_x += x
            sum_y += y
        cx = sum_x / len(cluster)
        cy = sum_y / len(cluster)
        return math.hypot(cx - robot_mx, cy - robot_my)

    def prune_failed_goal_blacklist(self):
        ttl = float(self.get_parameter("failed_goal_blacklist_ttl_sec").value)
        if ttl <= 0.0:
            self.failed_goal_blacklist = []
            return

        now = self.now_sec()
        self.failed_goal_blacklist = [
            entry for entry in self.failed_goal_blacklist if now - entry[2] <= ttl
        ]

        max_entries = int(self.get_parameter("failed_goal_blacklist_max_entries").value)
        if max_entries > 0 and len(self.failed_goal_blacklist) > max_entries:
            self.failed_goal_blacklist = self.failed_goal_blacklist[-max_entries:]

    def is_goal_blacklisted(self, x, y):
        radius = float(self.get_parameter("failed_goal_blacklist_radius_m").value)
        if radius <= 0.0:
            return False
        return any(
            math.hypot(x - goal_x, y - goal_y) <= radius
            for goal_x, goal_y, _ in self.failed_goal_blacklist
        )

    def sample_frontier_cells(self, cluster, robot_mx, robot_my):
        max_samples = max(
            1,
            int(self.get_parameter("max_frontier_samples_per_cluster").value),
        )
        if len(cluster) <= max_samples:
            return cluster

        by_angle = sorted(
            cluster,
            key=lambda cell: math.atan2(cell[1] - robot_my, cell[0] - robot_mx),
        )
        step = len(by_angle) / max_samples
        samples = [by_angle[min(len(by_angle) - 1, int(i * step))] for i in range(max_samples)]

        farthest = sorted(
            cluster,
            key=lambda cell: math.hypot(cell[0] - robot_mx, cell[1] - robot_my),
            reverse=True,
        )[: min(8, len(cluster))]

        unique_samples = []
        seen = set()
        for cell in samples + farthest:
            if cell in seen:
                continue
            seen.add(cell)
            unique_samples.append(cell)
        return unique_samples

    def find_safe_goal_cell(
        self,
        frontier_cx,
        frontier_cy,
        robot_mx=None,
        robot_my=None,
    ):
        if robot_mx is None or robot_my is None:
            rx, ry, _ = self.robot_pose
            robot_mx, robot_my = self.world_to_map(self.map_msg, rx, ry)
            if robot_mx is None:
                return None

        dx = robot_mx - frontier_cx
        dy = robot_my - frontier_cy
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return None

        resolution = self.map_msg.info.resolution
        inset_cells = max(1, int(float(self.get_parameter("goal_inset_m").value) / resolution))
        seed_x = int(round(frontier_cx + dx / length * inset_cells))
        seed_y = int(round(frontier_cy + dy / length * inset_cells))
        return self.find_nearest_safe_free_cell(seed_x, seed_y)

    def find_nearest_safe_free_cell(self, seed_x, seed_y):
        map_msg = self.map_msg
        resolution = map_msg.info.resolution
        search_radius = max(1, int(float(self.get_parameter("goal_search_radius_m").value) / resolution))
        width = map_msg.info.width
        height = map_msg.info.height

        best = None
        best_distance = None
        for y in range(max(0, seed_y - search_radius), min(height, seed_y + search_radius + 1)):
            for x in range(max(0, seed_x - search_radius), min(width, seed_x + search_radius + 1)):
                distance = math.hypot(x - seed_x, y - seed_y)
                if distance > search_radius:
                    continue
                if not self.is_safe_free_cell(x, y):
                    continue
                if best is None or distance < best_distance:
                    best = (x, y)
                    best_distance = distance

        return best

    def is_safe_free_cell(self, x, y):
        map_msg = self.map_msg
        data = map_msg.data
        width = map_msg.info.width
        height = map_msg.info.height
        free_threshold = int(self.get_parameter("free_threshold").value)
        occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        clearance_cells = max(
            1,
            int(float(self.get_parameter("obstacle_clearance_m").value) / map_msg.info.resolution),
        )

        if not (0 <= x < width and 0 <= y < height):
            return False
        center_value = data[y * width + x]
        if center_value < 0 or center_value > free_threshold:
            return False

        for ny in range(max(0, y - clearance_cells), min(height, y + clearance_cells + 1)):
            for nx in range(max(0, x - clearance_cells), min(width, x + clearance_cells + 1)):
                if math.hypot(nx - x, ny - y) > clearance_cells:
                    continue
                value = data[ny * width + nx]
                if value >= occupied_threshold:
                    return False

        return True

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

    def world_to_map(self, map_msg, wx, wy):
        origin = map_msg.info.origin
        resolution = map_msg.info.resolution
        yaw = yaw_from_quaternion(origin.orientation)
        dx = wx - origin.position.x
        dy = wy - origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        mx = int(math.floor(local_x / resolution))
        my = int(math.floor(local_y / resolution))
        if 0 <= mx < map_msg.info.width and 0 <= my < map_msg.info.height:
            return mx, my
        return None, None

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
