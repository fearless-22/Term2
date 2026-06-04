#!/usr/bin/env python3
import json
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String


class MapEvaluator(Node):
    """Publishes map quality metrics for the state manager and report."""

    UNKNOWN = -1

    def __init__(self):
        super().__init__("map_evaluator")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("evaluation_topic", "/map_evaluation")
        self.declare_parameter("timer_period_sec", 1.0)
        self.declare_parameter("free_threshold", 10)
        self.declare_parameter("min_frontier_cluster_size", 6)
        self.declare_parameter("pass_known_ratio", 0.72)
        self.declare_parameter("pass_frontier_clusters", 0)
        self.declare_parameter("pass_known_area_m2", 0.0)
        self.declare_parameter("log_period_sec", 5.0)

        self.map_msg = None
        self.last_log_time = self.get_clock().now()

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.map_callback,
            10,
        )
        self.eval_pub = self.create_publisher(
            String, self.get_parameter("evaluation_topic").value, 10
        )
        self.timer = self.create_timer(
            float(self.get_parameter("timer_period_sec").value), self.timer_callback
        )
        self.get_logger().info("Map evaluator ready")

    def map_callback(self, msg):
        self.map_msg = msg

    def timer_callback(self):
        if self.map_msg is None:
            self.publish_metrics({"event": "WAITING_FOR_MAP", "passed": False})
            return

        metrics = self.evaluate_map(self.map_msg)
        self.publish_metrics(metrics)

    def evaluate_map(self, map_msg):
        data = map_msg.data
        total = len(data)
        unknown = sum(1 for value in data if value == self.UNKNOWN)
        free = sum(1 for value in data if value == 0)
        occupied = sum(1 for value in data if value > 0)
        known = total - unknown
        known_ratio = known / total if total else 0.0
        resolution = map_msg.info.resolution
        known_area_m2 = known * resolution * resolution

        frontiers = self.detect_frontiers(map_msg)
        clusters = [
            cluster
            for cluster in self.cluster_frontiers(frontiers)
            if len(cluster) >= int(self.get_parameter("min_frontier_cluster_size").value)
        ]
        largest_cluster = max((len(cluster) for cluster in clusters), default=0)

        pass_known_ratio = float(self.get_parameter("pass_known_ratio").value)
        pass_frontier_clusters = int(self.get_parameter("pass_frontier_clusters").value)
        pass_known_area_m2 = float(self.get_parameter("pass_known_area_m2").value)
        known_area_ok = pass_known_area_m2 <= 0.0 or known_area_m2 >= pass_known_area_m2
        passed = (
            known_ratio >= pass_known_ratio
            and len(clusters) <= pass_frontier_clusters
            and known_area_ok
        )

        return {
            "event": "MAP_METRICS",
            "passed": passed,
            "width": map_msg.info.width,
            "height": map_msg.info.height,
            "resolution": resolution,
            "total_cells": total,
            "known_cells": known,
            "free_cells": free,
            "occupied_cells": occupied,
            "unknown_cells": unknown,
            "known_ratio": round(known_ratio, 4),
            "known_area_m2": round(known_area_m2, 3),
            "frontier_cells": len(frontiers),
            "frontier_clusters": len(clusters),
            "largest_frontier_cluster": largest_cluster,
        }

    def detect_frontiers(self, map_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        data = map_msg.data
        free_threshold = int(self.get_parameter("free_threshold").value)
        frontiers = set()

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x
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

    def publish_metrics(self, payload):
        payload["stamp"] = self.get_clock().now().nanoseconds * 1e-9
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.eval_pub.publish(msg)

        now = self.get_clock().now()
        period = float(self.get_parameter("log_period_sec").value)
        elapsed = (now - self.last_log_time).nanoseconds * 1e-9
        if elapsed >= period or payload.get("passed"):
            self.last_log_time = now
            self.get_logger().info(f"[MapEvaluation] {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = MapEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
