import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, String


class SpeedSupervisor(Node):
    def __init__(self):
        super().__init__('speed_supervisor')

        # 1) Parameters
        # 'mode' can now be: 'constant', 'manual', or 'external'
        self.declare_parameter('mode', 'constant')       # 'constant' | 'manual' | 'external' | 'ppo'
        self.declare_parameter('constant_speed', 0.5)    # 0.0 .. 1.0
        self.declare_parameter('manual_speed', 0.5)      # 0.0 .. 1.0

        # Fallback if external mode has no data yet
        self.declare_parameter('external_speed_fallback', 0.5)  # 0.0 .. 1.0

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.constant_speed = self.get_parameter('constant_speed').get_parameter_value().double_value
        self.manual_speed = self.get_parameter('manual_speed').get_parameter_value().double_value
        self.external_speed_fallback = self.get_parameter(
            'external_speed_fallback'
        ).get_parameter_value().double_value

        # External speed data (from other nodes, e.g., OPC UA / RL)
        self.external_speed = self.external_speed_fallback
        self.external_speed_valid = False
        self.warned_no_external = False  # so we don't spam logs

        # 2) Publisher for speed_set
        self.speed_pub = self.create_publisher(Float64, 'speed_set', 10)

        # 3) Subscriptions to cycle markers (future use)
        self.zone_busy = False
        self.cycle_ok = False

        self.create_subscription(Bool, 'zone_busy', self.zone_busy_callback, 10)
        self.create_subscription(Bool, 'cycle_ok', self.cycle_ok_callback, 10)

        # 3b) Subscription for external mode
        # Any "external supervisor" can publish Float64 on this topic.
        self.create_subscription(
            Float64,
            'speed_set_external',
            self.external_speed_callback,
            10
        )

        # 4) Timer to publish speed regularly (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 5) debug publisher (string with mode + speed + signals)
        self.debug_pub = self.create_publisher(String, 'speed_supervisor_debug', 10)


        # Last speed coming from OPC UA tag
        self.opcua_speed = 0.5
        self.opcua_speed_received = False

        # Manual / Auto mode from OPC UA (False = AUTO, True = MANUAL)
        self.opcua_manual_mode = False

        # Subscribe to bridge topics
        self.create_subscription(
            Float64,
            'opcua_speed_set',         # from bridge_node
            self._on_opcua_speed,
            10
        )
        self.create_subscription(
            Bool,
            'opcua_manual_mode',       # from bridge_node
            self._on_opcua_manual_mode,
            10
        )

        self.get_logger().info(
            f"SpeedSupervisor started in mode='{self.mode}', "
            f"constant_speed={self.constant_speed}, manual_speed={self.manual_speed}, "
            f"external_speed_fallback={self.external_speed_fallback}"
        )

    # --- Callbacks for OPC UA override ---

    def _on_opcua_speed(self, msg: Float64):
        """Remember latest OPC UA speed value."""
        self.opcua_speed = float(msg.data)
        self.opcua_speed_received = True

    def _on_opcua_manual_mode(self, msg: Bool):
        """Remember whether OPC UA selected MANUAL (True) or AUTO (False)."""
        self.opcua_manual_mode = bool(msg.data)
    
    
    # --- Callbacks for cycle markers ---

    def zone_busy_callback(self, msg: Bool):
        """Remember the latest zone_busy state (for future smarter logic)."""
        self.zone_busy = msg.data

    def cycle_ok_callback(self, msg: Bool):
        """Remember the latest cycle_ok state (for future smarter logic)."""
        self.cycle_ok = msg.data

    # --- Callback for external speed ---

    def external_speed_callback(self, msg: Float64):
        """
        Remember latest external speed.
        This is used when mode == 'external'.
        """
        self.external_speed = msg.data
        self.external_speed_valid = True
        # Reset the warning flag: now we HAVE external data.
        self.warned_no_external = False

    # --- Timer ---

    def timer_callback(self):
        """Periodic callback: compute current speed and publish it."""
        speed_value = self.compute_current_speed()

        # 3) Get internal speed from current mode
        internal_speed = self.compute_current_speed()

        # If PPO mode: wait until PPO has published at least one external speed
        # (unless OPC UA is in MANUAL and has a knob value)
        if self.mode == 'ppo':
            if not self.external_speed_valid and not (self.opcua_manual_mode and self.opcua_speed_received):
                if not getattr(self, 'warned_waiting_ppo', False):
                    self.get_logger().warn(
                        "Mode is 'ppo' but no data received on 'speed_set_external' yet. "
                        "Waiting (not publishing speed_set) until PPO starts."
                    )
                    self.warned_waiting_ppo = True
                return

        # 4) Apply OPC UA MANUAL / AUTO override
        if self.opcua_manual_mode and self.opcua_speed_received:
            # MANUAL mode ON → operator knob wins
            speed_value = self.opcua_speed
        else:
            # AUTO mode → use internal supervisor logic
            speed_value = internal_speed

        # (Optional) clamp between 0.0 and 1.0 just to be safe
        if speed_value < 0.0:
            speed_value = 0.0
        if speed_value > 1.0:
            speed_value = 1.0

        # 5) Publish final speed
        msg = Float64()
        msg.data = float(speed_value)
        self.speed_pub.publish(msg)

        # Publish debug info
        debug_msg = String()
        debug_msg.data = (
            f"mode='{self.mode}', speed={speed_value:.3f}, "
            f"opcua_manual={self.opcua_manual_mode}, "
            f"zone_busy={self.zone_busy}, cycle_ok={self.cycle_ok}"
        )
        self.debug_pub.publish(debug_msg)   

    # --- Core speed logic ---

    def compute_current_speed(self) -> float:
        """
        Decide what speed to output based on the current mode.

        - 'constant' -> always constant_speed
        - 'manual'   -> read manual_speed param (so we can change it with ros2 param set)
        - 'external' -> use speed from topic 'speed_set_external' if available,
                       otherwise use fallback parameter 'external_speed_fallback'
        """
        # Re-read mode each time so parameter updates are picked up
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        if self.mode == 'constant':
            self.constant_speed = self.get_parameter(
                'constant_speed'
            ).get_parameter_value().double_value
            return self.constant_speed

        if self.mode == 'manual':
            self.manual_speed = self.get_parameter(
                'manual_speed'
            ).get_parameter_value().double_value
            return self.manual_speed

        if self.mode in ('external', 'ppo'):
            # If we've received at least one external value, use that
            if self.external_speed_valid:
                return self.external_speed

            # Otherwise use the fallback parameter and warn once
            self.external_speed_fallback = self.get_parameter(
                'external_speed_fallback'
            ).get_parameter_value().double_value

            if not self.warned_no_external:
                self.get_logger().warn(
                    "Mode is 'external' but no data received on 'speed_set_external' yet. "
                    "Using 'external_speed_fallback' parameter."
                )
                self.warned_no_external = True

            return self.external_speed_fallback

        # Fallback if someone sets an unknown mode
        self.get_logger().warn(
            f"Unknown mode '{self.mode}', falling back to constant_speed."
        )
        return self.constant_speed


def main(args=None):
    rclpy.init(args=args)
    node = SpeedSupervisor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
