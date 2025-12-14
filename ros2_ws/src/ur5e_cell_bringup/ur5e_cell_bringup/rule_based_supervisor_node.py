import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64


class RuleBasedSupervisor(Node):
    def __init__(self):
        super().__init__("rule_based_supervisor")

        # Parameters for the rule
        self.declare_parameter("initial_speed", 0.3)
        self.declare_parameter("min_speed", 0.2)
        self.declare_parameter("max_speed", 0.9)
        self.declare_parameter("delta_up", 0.05)
        self.declare_parameter("delta_down", 0.10)

        self.current_speed = (
            self.get_parameter("initial_speed")
            .get_parameter_value()
            .double_value
        )
        self.min_speed = (
            self.get_parameter("min_speed")
            .get_parameter_value()
            .double_value
        )
        self.max_speed = (
            self.get_parameter("max_speed")
            .get_parameter_value()
            .double_value
        )
        self.delta_up = (
            self.get_parameter("delta_up")
            .get_parameter_value()
            .double_value
        )
        self.delta_down = (
            self.get_parameter("delta_down")
            .get_parameter_value()
            .double_value
        )

        # State of the current motion
        self.zone_busy = False
        self.current_cycle_had_ok = False
        self.current_cycle_had_stop = False

        # Motion counter so we can skip the very first one (homing move)
        self.motion_index = 0   # 0 = first motion, 1 = second, ...

        # Publisher: this is what SpeedSupervisor (mode='external') listens to
        self.speed_pub = self.create_publisher(
            Float64, "speed_set_external", 10
        )

        # Subscriptions: we watch the cycle markers and stop_req
        self.create_subscription(Bool, "zone_busy", self.on_zone_busy, 10)
        self.create_subscription(Bool, "cycle_ok", self.on_cycle_ok, 10)
        self.create_subscription(Bool, "stop_req", self.on_stop_req, 10)

        # Publish the initial speed once at startup
        self.publish_speed()

        self.get_logger().info(
            f"RuleBasedSupervisor started with "
            f"initial_speed={self.current_speed:.3f}, "
            f"min_speed={self.min_speed:.3f}, "
            f"max_speed={self.max_speed:.3f}, "
            f"delta_up={self.delta_up:.3f}, "
            f"delta_down={self.delta_down:.3f}"
        )

            # ------------- helper: publish current speed -------------
    def publish_speed(self):
        msg = Float64()
        msg.data = float(self.current_speed)
        self.speed_pub.publish(msg)
        self.get_logger().info(
            f"[Rule] publishing speed_set_external = {self.current_speed:.3f}"
        )

    # ------------- callbacks -------------

    def on_zone_busy(self, msg: Bool):
        z = bool(msg.data)

        # Rising edge: new motion starts
        if not self.zone_busy and z:
            self.current_cycle_had_ok = False
            self.current_cycle_had_stop = False
            self.get_logger().info(
                f"[Rule] Motion {self.motion_index} started (zone_busy=True)"
            )

        # Falling edge: motion ends
        elif self.zone_busy and not z:
            self.get_logger().info(
                f"[Rule] Motion {self.motion_index} ended: "
                f"cycle_ok={self.current_cycle_had_ok}, "
                f"had_stop_req={self.current_cycle_had_stop}"
            )

            if self.motion_index == 0:
                # First motion is the homing move current->A, ignore for rule
                self.get_logger().info(
                    "[Rule] Skipping first motion (homing) for speed update."
                )
            else:
                # Apply the rule based on this motion
                self.update_speed_after_motion(
                    success=self.current_cycle_had_ok,
                    had_stop=self.current_cycle_had_stop,
                )

            # Move to next index for the next motion
            self.motion_index += 1

        # Remember latest zone_busy state
        self.zone_busy = z

    def on_cycle_ok(self, msg: Bool):
        if msg.data:
            self.current_cycle_had_ok = True

    def on_stop_req(self, msg: Bool):
        if msg.data:
            self.current_cycle_had_stop = True

        # ------------- core rule -------------

    def update_speed_after_motion(self, success: bool, had_stop: bool):
        """
        Simple rule:

        - If there was a stop or no success -> decrease speed by delta_down.
        - Else (success and no stop) -> increase speed by delta_up.
        """
        old_speed = self.current_speed

        if had_stop or (not success):
            # Bad motion -> go slower
            self.current_speed -= self.delta_down
            reason = "bad (stop or no cycle_ok), decreasing"
        else:
            # Good motion -> go faster
            self.current_speed += self.delta_up
            reason = "good (success, no stop), increasing"

        # Clamp between min and max
        if self.current_speed < self.min_speed:
            self.current_speed = self.min_speed
        if self.current_speed > self.max_speed:
            self.current_speed = self.max_speed

        self.get_logger().info(
            f"[Rule] Motion result {reason}: "
            f"{old_speed:.3f} -> {self.current_speed:.3f}"
        )

        # Publish new speed for next motion
        self.publish_speed()


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


