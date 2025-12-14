#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatusArray, GoalStatus

class CycleSignalsNode(Node):

    def __init__(self) -> None:
        super().__init__("cycle_signals_node_rewrite")

        self.declare_parameter(
            "status_topic",
            "/joint_trajectory_controller/follow_joint_trajectory/_action/status",
        )

        status_topic = (
            self.get_parameter("status_topic").get_parameter_value().string_value
        )

        # just logging that the node has started
        self.get_logger().info("CycleSignalsNode_rewrite has started.")

        self.zone_busy_pub = self.create_publisher(Bool, "zone_busy", 10)
        self.cycle_ok_pub = self.create_publisher(Bool, "cycle_ok", 10)

        self._zone_busy_state = False
        self._cycle_ok_reset_timer = None

        self.create_subscription(
            GoalStatusArray,
            status_topic,
            self.status_callback,
            10,
        )
    
    def _decode_status(self, status_code :int):
        if status_code == GoalStatus.STATUS_ACCEPTED:
            return "STATUS_ACCEPTED"
        elif status_code == GoalStatus.STATUS_EXECUTING:
            return "STATUS_EXECUTING"
        elif status_code == GoalStatus.STATUS_CANCELING:
            return "STATUS_CANCELING"
        elif status_code == GoalStatus.STATUS_SUCCEEDED:
            return "STATUS_SUCCEEDED"
        elif status_code == GoalStatus.STATUS_CANCELED:
            return "STATUS_CANCELED"
        elif status_code == GoalStatus.STATUS_ABORTED:
            return "STATUS_ABORTED"
        else:
            return "UNKNOWN_STATUS"
        
        
    



    def status_callback(self, msg: GoalStatusArray) -> None:
        
        # If there are no statuses, do nothing
        if not msg.status_list:
            return
        
        # Get the latest status from the status list (assuming it's the last one)
        latest_status = msg.status_list[len(msg.status_list) - 1]
        status_code = latest_status.status
        label = self._decode_status(status_code)

        # Log the latest status code with its label
        self.get_logger().info(f"Latest status code: {status_code} ({label}")

        # Determine if the robot is running based on the status code
        if status_code == GoalStatus.STATUS_ACCEPTED or status_code == GoalStatus.STATUS_EXECUTING:
            is_running = True
        else:
            is_running = False

        # Determine if the robot has successfully completed its task based on the status code
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            is_success = True
        else:
            is_success = False
        
        # Determine if the robot has aborted or canceled its task based on the status code
        if status_code == GoalStatus.STATUS_ABORTED or status_code == GoalStatus.STATUS_CANCELED:
            is_aborted = True
        else:
            is_aborted = False
        
        # ------------------------------------------------------
        # Edge detection for zone_busy
        # ------------------------------------------------------

        old_zone_busy_state = self._zone_busy_state
        new_zone_busy_state = is_running

        if new_zone_busy_state != old_zone_busy_state:
            # State has changed and new state is published
            self._zone_busy_state = new_zone_busy_state

            out_msg = Bool()
            out_msg.data = new_zone_busy_state
            self.zone_busy_pub.publish(out_msg)

            self.get_logger().info(
                "zone_busy "
                + str(new_zone_busy_state)
                + " (from status code "
                + label
                + ")"
            )

        if old_zone_busy_state and (not new_zone_busy_state) and is_success:
            self._publish_cycle_ok_pulse()

        if old_zone_busy_state and (not new_zone_busy_state) and is_aborted:
            self.get_logger().warn(
                "Trajectory execution aborted or canceled (status code "
                + label
                + "); no cycle_ok pulse emitted." 
            )

    def _publish_cycle_ok_pulse(self, duration_sec: float = 0.2):
        
        self.get_logger().info("Emmitting cycle_ok pulse.")
        # Publish cycle_ok = True
        msg_true = Bool()
        msg_true.data = True
        self.cycle_ok_pub.publish(msg_true)

        if self._cycle_ok_reset_timer is not None:
            self._cycle_ok_reset_timer.cancel()
            self._cycle_ok_reset_timer = None
        
        self._cycle_ok_reset_timer = self.create_timer(
            duration_sec,
            self._reset_cycle_ok_)
        
    def _reset_cycle_ok_(self):

        msg_false = Bool()
        msg_false.data = False
        self.cycle_ok_pub.publish(msg_false)
        self.get_logger().debug("cycle_ok reset to False.")

        if self._cycle_ok_reset_timer is not None:
            self._cycle_ok_reset_timer.cancel()
            self._cycle_ok_reset_timer = None

        
def main(args=None):
    rclpy.init(args=args)
    node = CycleSignalsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CycleSignalsNode_rewrite.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()