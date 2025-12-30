import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray


class RuleBasedSupervisor(Node):
    """
    Simple rule-based speed supervisor.

    Input:  /cycle_summary (Float64MultiArray)
            [cycle_index, payload_mass, speed, duration, success, had_stop, max_vel_ratio, max_torque_ratio]

    Output: /speed_set_external (Float64)
            (SpeedSupervisor in mode='external' listens to this)
    """

    def __init__(self):
        super().__init__("rule_based_supervisor")

        # Speed params
        self.declare_parameter("initial_speed", 0.30)
        self.declare_parameter("min_speed", 0.20)
        self.declare_parameter("max_speed", 0.90)
        self.declare_parameter("delta_up", 0.05)
        self.declare_parameter("delta_down", 0.10)

        # Safety thresholds (ratios, 0..1)
        # Tune later. Start loose, tighten for real robot.
        self.declare_parameter("torque_ratio_limit", 0.35)
        self.declare_parameter("vel_ratio_limit", 0.06)

        self.speed = float(self.get_parameter("initial_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.delta_up = float(self.get_parameter("delta_up").value)
        self.delta_down = float(self.get_parameter("delta_down").value)

        self.torque_lim = float(self.get_parameter("torque_ratio_limit").value)
        self.vel_lim = float(self.get_parameter("vel_ratio_limit").value)

        # We skip cycle_index 0 (executor's homing move current->A)
        self.seen_first_cycle = False

        self.pub = self.create_publisher(Float64, "speed_set_external", 10)
        self.sub = self.create_subscription(
            Float64MultiArray, "cycle_summary", self.on_cycle_summary, 10
        )

        self.publish_speed("startup")
        self.get_logger().info(
            f"RuleBasedSupervisor ready | "
            f"speed={self.speed:.3f} [{self.min_speed:.2f},{self.max_speed:.2f}] "
            f"du={self.delta_up:.2f} dd={self.delta_down:.2f} "
            f"torque_lim={self.torque_lim:.2f} vel_lim={self.vel_lim:.2f}"
        )

    def clamp(self, x: float) -> float:
        return max(self.min_speed, min(self.max_speed, x))

    def publish_speed(self, reason: str):
        msg = Float64()
        msg.data = float(self.speed)
        self.pub.publish(msg)
        self.get_logger().info(f"[Rule] publish speed_set_external={self.speed:.3f} ({reason})")

    def on_cycle_summary(self, msg: Float64MultiArray):
        d = list(msg.data) if msg.data is not None else []
        if len(d) < 8:
            return

        cycle_index = int(d[0])
        payload_mass = float(d[1])
        used_speed = float(d[2])
        duration = float(d[3])
        success = bool(int(d[4]))
        had_stop = bool(int(d[5]))
        max_vel_ratio = float(d[6])
        max_torque_ratio = float(d[7])

        # Skip first “homing” move (cycle 0)
        if not self.seen_first_cycle:
            self.seen_first_cycle = True
            self.get_logger().info(f"[Rule] skip cycle {cycle_index} (homing).")
            return

        old = self.speed

        # --- RULE ---
        if had_stop or (not success):
            self.speed = self.clamp(self.speed - self.delta_down)
            reason = "bad (stop or fail) -> down"
        elif (max_torque_ratio > self.torque_lim) or (max_vel_ratio > self.vel_lim):
            self.speed = self.clamp(self.speed - self.delta_down)
            reason = "near-limit (torque/vel high) -> down"
        else:
            self.speed = self.clamp(self.speed + self.delta_up)
            reason = "good -> up"

        self.get_logger().info(
            f"[Rule] cycle={cycle_index} mass={payload_mass:.2f} "
            f"used_speed={used_speed:.3f} dur={duration:.3f}s "
            f"ok={success} stop={had_stop} "
            f"velR={max_vel_ratio:.3f} torqueR={max_torque_ratio:.3f} | "
            f"{old:.3f}->{self.speed:.3f} ({reason})"
        )

        # Publish new speed for the NEXT motion
        self.publish_speed(reason)

def main(args=None):
    rclpy.init(args=args)
    node = RuleBasedSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
