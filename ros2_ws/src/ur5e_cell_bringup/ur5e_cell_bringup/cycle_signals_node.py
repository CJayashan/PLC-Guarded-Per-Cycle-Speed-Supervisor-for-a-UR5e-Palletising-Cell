#!/usr/bin/env python3

"""
cycle_signals_node.py

Listens to the FollowJointTrajectory action status from the
joint_trajectory_controller and publishes two simple cycle signals:

  - /zone_busy (std_msgs/Bool)
      True  while a trajectory is being executed
      False when no trajectory is active

  - /cycle_ok (std_msgs/Bool)
      Short True pulse when a trajectory completes with SUCCESS

This node is completely independent from Gazebo / MoveIt internals.
It just watches the action status topic.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from action_msgs.msg import GoalStatusArray, GoalStatus


# We treat these status values as "motion in progress"
ACTIVE_STATES = (
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
)


class CycleSignalsNode(Node):
    def __init__(self) -> None:
        super().__init__("cycle_signals_node")

        # --- Publishers for our simple boolean signals ---
        self.zone_busy_pub = self.create_publisher(Bool, "zone_busy", 10)
        self.cycle_ok_pub = self.create_publisher(Bool, "cycle_ok", 10)

        # Internal state: are we currently busy or not?
        self._zone_busy_state = False

        # Internal state: whether we saw SUCCESS during the current busy period
        self._had_success_in_busy = False

        # Timer handle for one-shot cycle_ok reset
        self._cycle_ok_reset_timer = None

        # Allow overriding the status topic via parameter if needed
        # Default should match the FollowJointTrajectory action for the JTC
        self.declare_parameter(
            "status_topic",
            "/joint_trajectory_controller/follow_joint_trajectory/_action/status",
        )
        status_topic = (
            self.get_parameter("status_topic").get_parameter_value().string_value
        )

        self.get_logger().info(
            f"Listening to action status on: {status_topic}"
        )

        # --- Subscription to FollowJointTrajectory status ---
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            status_topic,
            self.status_callback,
            10,
        )

    # ------------------------------------------------------
    # Callback: action status updates from the trajectory controller
    # ------------------------------------------------------
    def status_callback(self, msg: GoalStatusArray) -> None:
        """
        Each status message contains a list of goals with status codes.
        We derive:
          - busy_now: at least one goal is ACCEPTED or EXECUTING
          - success_present_now: at least one goal has SUCCEEDED
        """

        # Is there any active goal?
        busy_now = any(
            s.status in ACTIVE_STATES for s in msg.status_list
        )

        # Has any goal in THIS message succeeded?
        success_present_now = any(
            s.status == GoalStatus.STATUS_SUCCEEDED for s in msg.status_list
        )

        # --- Rising edge: we just became busy ---
        if busy_now and not self._zone_busy_state:
            # New busy period starts -> reset success memory
            self._zone_busy_state = True
            self._had_success_in_busy = False
            self._publish_zone_busy(True)
            self.get_logger().debug("zone_busy -> True")

        # --- While busy: remember if we ever see SUCCESS ---
        if busy_now and success_present_now:
            self._had_success_in_busy = True

        # --- Falling edge: we just became idle ---
        if (not busy_now) and self._zone_busy_state:
            # Decide if this motion was successful
            motion_had_success = self._had_success_in_busy or success_present_now

            # 1) If we saw SUCCESS at any point in this busy period, pulse cycle_ok
            if motion_had_success:
                self._publish_cycle_ok_pulse()

            # 2) Then drop zone_busy
            self._zone_busy_state = False
            self._publish_zone_busy(False)
            self.get_logger().debug("zone_busy -> False")

            # Reset memory for next motion
            self._had_success_in_busy = False

    # ------------------------------------------------------
    # Helper: publish zone_busy
    # ------------------------------------------------------
    def _publish_zone_busy(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.zone_busy_pub.publish(msg)

    # ------------------------------------------------------
    # Helper: publish a short cycle_ok pulse
    # ------------------------------------------------------
    def _publish_cycle_ok_pulse(self) -> None:
        """
        Publish cycle_ok = True, then automatically reset to False
        after a short delay (~100 ms). This makes it easier for the
        OPC UA bridge / PLC to catch the pulse.
        """
        self.get_logger().info("cycle_ok pulse")

        # Send rising edge
        msg_true = Bool()
        msg_true.data = True
        self.cycle_ok_pub.publish(msg_true)

        # Cancel any previous timer (if a second cycle finishes quickly)
        if self._cycle_ok_reset_timer is not None:
            self._cycle_ok_reset_timer.cancel()
            self._cycle_ok_reset_timer = None

        # One-shot timer to send the falling edge a bit later
        self._cycle_ok_reset_timer = self.create_timer(
            0.1,  # 100 ms pulse
            self._cycle_ok_reset_callback,
        )

    def _cycle_ok_reset_callback(self) -> None:
        """Timer callback to send cycle_ok = False once, then stop."""
        msg_false = Bool()
        msg_false.data = False
        self.cycle_ok_pub.publish(msg_false)
        self.get_logger().debug("cycle_ok reset to False")

        # Stop this timer so it only triggers once
        if self._cycle_ok_reset_timer is not None:
            self._cycle_ok_reset_timer.cancel()
            self._cycle_ok_reset_timer = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CycleSignalsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
