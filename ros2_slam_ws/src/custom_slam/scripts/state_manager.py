#!/usr/bin/env python3
import json
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


class State(str, Enum):
    IDLE = "IDLE"
    EXPLORATION = "EXPLORATION"
    NAVIGATION = "NAVIGATION"
    MAP_EVALUATION = "MAP_EVALUATION"
    ERROR = "ERROR"
    RECOVERY = "RECOVERY"
    FINISH = "FINISH"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class StateManager(Node):
    """Top-level FSM for autonomous exploration, navigation and recovery."""

    def __init__(self):
        super().__init__("state_manager")

        self.declare_parameter("auto_start", True)
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("heartbeat_hz", 10.0)
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("candidate_timeout_sec", 25.0)
        self.declare_parameter("recovery_cooldown_sec", 3.0)
        self.declare_parameter("max_recovery_attempts", 3)
        self.declare_parameter("goal_reached_tolerance", 0.35)
        self.declare_parameter("status_log_period_sec", 2.0)
        self.declare_parameter("navigate_action_name", "navigate_to_pose")
        self.declare_parameter("global_clear_service", "/global_costmap/clear_entirely_global_costmap")
        self.declare_parameter("local_clear_service", "/local_costmap/clear_entirely_local_costmap")

        self.state = State.IDLE
        self.state_enter_time = self.now_sec()
        self.start_time = self.now_sec()
        self.pending_goal = None
        self.active_goal = None
        self.goal_handle = None
        self.navigation_start_time = None
        self.navigation_result = None
        self.recovery_attempts = 0
        self.recovery_started = False
        self.last_candidate_time = self.now_sec()
        self.last_status_log_time = self.now_sec()
        self.last_error_reason = ""
        self.map_evaluation = None
        self.last_exploration_status = None
        self.robot_pose = None

        self.exploration_enable_pub = self.create_publisher(
            Bool, "/exploration/enable", 10
        )
        self.system_state_pub = self.create_publisher(String, "/system_state", 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, "/exploration/goal_pose", self.goal_callback, 10
        )
        self.exploration_status_sub = self.create_subscription(
            String, "/exploration/status", self.exploration_status_callback, 10
        )
        self.map_evaluation_sub = self.create_subscription(
            String, "/map_evaluation", self.map_evaluation_callback, 10
        )
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.command_sub = self.create_subscription(
            String, "/system_command", self.system_command_callback, 10
        )

        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter("navigate_action_name").value
        )
        self.global_clear_client = self.create_client(
            ClearEntireCostmap, self.get_parameter("global_clear_service").value
        )
        self.local_clear_client = self.create_client(
            ClearEntireCostmap, self.get_parameter("local_clear_service").value
        )

        heartbeat_hz = float(self.get_parameter("heartbeat_hz").value)
        self.timer = self.create_timer(1.0 / heartbeat_hz, self.timer_callback)
        self.get_logger().info("State manager ready")

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def goal_callback(self, msg):
        if self.state != State.EXPLORATION:
            return
        self.pending_goal = msg
        self.last_candidate_time = self.now_sec()

    def exploration_status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.last_exploration_status = payload
        event = payload.get("event")
        if event == "DONE" and self.state in (State.EXPLORATION, State.IDLE):
            self.transition(State.MAP_EVALUATION, "exploration reports done")

    def map_evaluation_callback(self, msg):
        try:
            self.map_evaluation = json.loads(msg.data)
        except json.JSONDecodeError:
            self.map_evaluation = None

    def odom_callback(self, msg):
        self.robot_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def system_command_callback(self, msg):
        command = msg.data.strip().upper()
        if command == "START":
            self.recovery_attempts = 0
            self.last_error_reason = ""
            self.transition(State.EXPLORATION, "operator command START")
        elif command == "STOP":
            self.pending_goal = None
            self.navigation_result = None
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.publish_zero_velocity()
            self.transition(State.IDLE, "operator command STOP")
        elif command == "INJECT_ERROR":
            self.last_error_reason = "operator injected recoverable fault"
            self.transition(State.ERROR, self.last_error_reason)
        elif command == "FINISH":
            self.pending_goal = None
            self.navigation_result = None
            self.publish_zero_velocity()
            self.transition(State.FINISH, "operator command FINISH")
        else:
            self.get_logger().warn(f"Unknown /system_command: {msg.data}")

    def timer_callback(self):
        self.publish_system_state()

        if self.state == State.IDLE:
            self.publish_exploration_enable(False)
            if bool(self.get_parameter("auto_start").value):
                delay = float(self.get_parameter("start_delay_sec").value)
                if self.now_sec() - self.start_time >= delay:
                    self.transition(State.EXPLORATION, "auto start")

        elif self.state == State.EXPLORATION:
            self.publish_exploration_enable(True)
            if self.map_passed():
                self.transition(State.MAP_EVALUATION, "map evaluator passed")
                return
            if self.pending_goal is not None:
                self.send_navigation_goal(self.pending_goal)
                self.pending_goal = None
                return
            candidate_timeout = float(self.get_parameter("candidate_timeout_sec").value)
            if (
                self.can_timeout_waiting_for_candidate()
                and self.now_sec() - self.last_candidate_time > candidate_timeout
            ):
                self.last_error_reason = "frontier candidate timeout"
                self.transition(State.ERROR, self.last_error_reason)

        elif self.state == State.NAVIGATION:
            self.publish_exploration_enable(False)
            self.check_navigation_timeout()
            self.check_navigation_result()

        elif self.state == State.MAP_EVALUATION:
            self.publish_exploration_enable(False)
            if self.map_passed():
                self.transition(State.FINISH, "map evaluation passed")
            else:
                self.transition(State.EXPLORATION, "map still has frontiers")

        elif self.state == State.ERROR:
            self.publish_exploration_enable(False)
            if self.recovery_attempts >= int(self.get_parameter("max_recovery_attempts").value):
                self.transition(State.EMERGENCY_STOP, "recovery attempts exhausted")
            else:
                self.transition(State.RECOVERY, self.last_error_reason or "recoverable error")

        elif self.state == State.RECOVERY:
            self.publish_exploration_enable(False)
            self.publish_zero_velocity()
            self.execute_recovery()

        elif self.state == State.FINISH:
            self.publish_exploration_enable(False)
            self.publish_zero_velocity()

        elif self.state == State.EMERGENCY_STOP:
            self.publish_exploration_enable(False)
            self.publish_zero_velocity()

    def send_navigation_goal(self, pose):
        if not self.nav_client.server_is_ready():
            self.last_error_reason = "NavigateToPose action server not ready"
            self.transition(State.ERROR, self.last_error_reason)
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav_goal_pub.publish(pose)
        self.active_goal = pose
        self.navigation_result = None
        self.navigation_start_time = self.now_sec()

        future = self.nav_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        self.transition(State.NAVIGATION, "frontier goal accepted for dispatch")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.last_error_reason = "NavigateToPose goal rejected"
            self.navigation_result = "FAILED"
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.navigation_result = "SUCCEEDED"
        else:
            self.last_error_reason = f"NavigateToPose failed with status {result.status}"
            self.navigation_result = "FAILED"

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback.distance_remaining <= float(self.get_parameter("goal_reached_tolerance").value):
            self.get_logger().debug("Goal is within reached tolerance")

    def check_navigation_result(self):
        if self.navigation_result == "SUCCEEDED":
            self.recovery_attempts = 0
            self.goal_handle = None
            self.active_goal = None
            self.transition(State.EXPLORATION, "goal reached")
        elif self.navigation_result == "FAILED":
            self.goal_handle = None
            self.active_goal = None
            self.transition(State.ERROR, self.last_error_reason)

    def check_navigation_timeout(self):
        if self.navigation_start_time is None:
            return
        timeout = float(self.get_parameter("goal_timeout_sec").value)
        if self.now_sec() - self.navigation_start_time <= timeout:
            return

        self.last_error_reason = "NavigateToPose timeout"
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.navigation_result = "FAILED"

    def execute_recovery(self):
        if not self.recovery_started:
            self.recovery_started = True
            self.recovery_attempts += 1
            self.clear_costmaps()
            self.state_enter_time = self.now_sec()
            self.get_logger().warn(
                f"[FSM Recovery] attempt={self.recovery_attempts} reason={self.last_error_reason}"
            )

        cooldown = float(self.get_parameter("recovery_cooldown_sec").value)
        if self.now_sec() - self.state_enter_time >= cooldown:
            self.recovery_started = False
            self.navigation_result = None
            self.pending_goal = None
            self.transition(State.EXPLORATION, "recovery complete")

    def clear_costmaps(self):
        request = ClearEntireCostmap.Request()
        if self.global_clear_client.service_is_ready():
            self.global_clear_client.call_async(request)
        else:
            self.get_logger().warn("Global costmap clear service is not ready")

        if self.local_clear_client.service_is_ready():
            self.local_clear_client.call_async(request)
        else:
            self.get_logger().warn("Local costmap clear service is not ready")

    def map_passed(self):
        return bool(self.map_evaluation and self.map_evaluation.get("passed"))

    def can_timeout_waiting_for_candidate(self):
        if self.map_evaluation is None or self.last_exploration_status is None:
            return False

        waiting_events = {"WAITING_FOR_MAP", "WAITING_FOR_ODOM", "IDLE"}
        if self.map_evaluation.get("event") == "WAITING_FOR_MAP":
            return False
        if self.last_exploration_status.get("event") in waiting_events:
            return False

        return True

    def publish_exploration_enable(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self.exploration_enable_pub.publish(msg)

    def publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def transition(self, new_state, reason):
        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state
        self.state_enter_time = self.now_sec()
        if new_state == State.EXPLORATION:
            self.last_candidate_time = self.now_sec()
            self.pending_goal = None
        if new_state != State.RECOVERY:
            self.recovery_started = False

        self.get_logger().info(
            f"[FSM State]: {old_state.value} -> {new_state.value} | reason={reason}"
        )
        self.publish_system_state(force_log=True)

    def publish_system_state(self, force_log=False):
        payload = {
            "state": self.state.value,
            "recovery_attempts": self.recovery_attempts,
            "last_error_reason": self.last_error_reason,
            "map_evaluation": self.map_evaluation,
        }
        if self.active_goal is not None:
            payload["active_goal"] = {
                "x": self.active_goal.pose.position.x,
                "y": self.active_goal.pose.position.y,
            }

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.system_state_pub.publish(msg)

        now = self.now_sec()
        period = float(self.get_parameter("status_log_period_sec").value)
        if force_log or now - self.last_status_log_time >= period:
            self.last_status_log_time = now
            self.get_logger().info(f"[FSM Heartbeat] {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
