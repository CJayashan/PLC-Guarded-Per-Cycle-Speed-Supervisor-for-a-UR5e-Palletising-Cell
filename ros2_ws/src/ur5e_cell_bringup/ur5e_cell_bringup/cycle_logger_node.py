import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class CycleLoggerNode(Node):
    """
    Advanced cycle logger.

    - Watches /zone_busy rising/falling edges to detect each motion segment.
    - Uses /speed_set as the speed value.
    - Uses /cycle_ok and /stop_req to mark success / stop.
    - Subscribes to /joint_states to measure how "hard" the motion was.
    - Writes one row to CSV per motion, including per-joint torque/velocity ratios.
    """

    def __init__(self):
        super().__init__('cycle_logger')

        # Parameters
        self.declare_parameter('csv_path', 'cycle_log.csv')
        self.declare_parameter('run_id', 'test_run')
        # Payload mass used in this scenario (kg).
        self.declare_parameter('payload_mass', 5.0)

        csv_path_str = (
            self.get_parameter('csv_path')
            .get_parameter_value()
            .string_value
        )
        self.run_id = (
            self.get_parameter('run_id')
            .get_parameter_value()
            .string_value
        )
        self.payload_mass = (
            self.get_parameter('payload_mass')
            .get_parameter_value()
            .double_value
        )

        # --- Joint configuration for UR5e (6 joints) ---
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.joint_short_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_1",
            "wrist_2",
            "wrist_3",
        ]

        # Previous joint state for finite-difference velocity (SIM path)
        self.prev_js_time = None
        self.prev_js_positions = [None] * len(self.joint_names)

        # Per-cycle maxima
        self.max_abs_effort = [0.0] * len(self.joint_names)
        self.max_abs_velocity = [0.0] * len(self.joint_names)

        # Vendor-like max joint torques for UR5e (Nm)
        self.tau_max = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]

        # Max joint speed: 180 deg/s = pi rad/s for all joints
        self.vel_max = [math.radians(180.0)] * 6

        # Map from joint name -> index in JointState arrays
        self.joint_index_map = {}

        # Per-motion metrics
        self.metrics_active = False
        self.max_abs_effort = [0.0] * len(self.joint_names)
        self.max_abs_velocity = [0.0] * len(self.joint_names)

        # Prepare CSV file
        csv_path = Path(csv_path_str)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = open(csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # --- CSV Header ---
        header = [
            'run_id',
            'cycle_index',
            'payload_mass',
            'speed',
            't_start',
            't_end',
            'duration',
            'success',
            'had_stop_req',
            'max_torque_ratio',
            'max_velocity_ratio',
        ]

        for short_name in self.joint_short_names:
            header.extend([
                f'{short_name}_max_effort',
                f'{short_name}_torque_ratio',
                f'{short_name}_max_velocity',
                f'{short_name}_velocity_ratio',
            ])

        self.csv_writer.writerow(header)
        self.csv_file.flush()
        self.get_logger().info(f"CycleLogger writing to: {csv_path}")

        # Internal state for cycle detection
        self.last_zone_busy = False
        self.current_cycle_start = None
        self.current_cycle_had_ok = False
        self.current_cycle_had_stop = False
        self.current_speed = 0.0
        self.cycle_index = 0

        # Latched values for the CURRENT cycle
        self.cycle_speed_latched = 0.0
        self.cycle_payload_mass_latched = 0.0

        self.awaiting_cycle_end = False

        self.cycle_ok_grace_sec = 0.2
        self.pending_end_time = None
        self.pending_finalize_timer = None

        # Publishers
        self.summary_pub = self.create_publisher(Float64MultiArray, 'cycle_summary', 10)

        # Subscriptions
        self.create_subscription(Bool, 'zone_busy', self.on_zone_busy, 10)
        self.create_subscription(Bool, 'cycle_ok', self.on_cycle_ok, 10)
        self.create_subscription(Bool, 'stop_req', self.on_stop_req, 10)
        self.create_subscription(Float64, 'speed_set', self.on_speed_set, 10)
        self.create_subscription(Float64, 'payload_mass', self.on_payload_mass, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 50)

        # --- Torque estimator wiring (Gazebo joint_states effort is often zero) ---
        self.declare_parameter('estimated_effort_topic', 'estimated_joint_effort')
        self.declare_parameter('use_estimated_effort', True)
        self.declare_parameter('effort_zero_tol', 1e-6)

        # >>> NEW: Real mode switch
        # If real==True:
        #   - velocity uses msg.velocity directly
        #   - estimator effort used ONLY if all real efforts are ~zero
        self.declare_parameter('real', False)

        self.use_estimated_effort = bool(self.get_parameter('use_estimated_effort').value)
        self.effort_zero_tol = float(self.get_parameter('effort_zero_tol').value)
        self.real = bool(self.get_parameter('real').value)

        # Latest estimated torques (same order as self.joint_names)
        self.latest_est_effort = [None] * len(self.joint_names)
        self.latest_est_effort_time = None

        est_topic = str(self.get_parameter('estimated_effort_topic').value)
        self.create_subscription(Float64MultiArray, est_topic, self.on_estimated_joint_effort, 10)

    def get_now_sec(self) -> float:
        now = self.get_clock().now()
        return now.nanoseconds / 1e9

    # --- Callbacks ---

    def on_speed_set(self, msg: Float64):
        self.current_speed = float(msg.data)

    def on_payload_mass(self, msg: Float64):
        self.payload_mass = float(msg.data)

    def on_cycle_ok(self, msg: Bool):
        if not msg.data:
            return

        self.current_cycle_had_ok = True

        if self.pending_end_time is not None and self.current_cycle_start is not None and self.awaiting_cycle_end:
            if self.pending_finalize_timer is not None:
                self.pending_finalize_timer.cancel()
                self.pending_finalize_timer = None

            t_end = self.pending_end_time
            self.pending_end_time = None
            self._finalize_cycle(t_end, end_reason="zone_fall+cycle_ok")

    def on_stop_req(self, msg: Bool):
        if msg.data:
            self.current_cycle_had_stop = True

    def on_estimated_joint_effort(self, msg: Float64MultiArray):
        if msg.data is None or len(msg.data) < len(self.joint_names):
            return
        self.latest_est_effort = [float(msg.data[i]) for i in range(len(self.joint_names))]
        self.latest_est_effort_time = self.get_now_sec()

    def joint_states_callback(self, msg: JointState):
        now = self.get_clock().now().nanoseconds * 1e-9

        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        positions = [None] * len(self.joint_names)
        efforts   = [None] * len(self.joint_names)
        velocities = [None] * len(self.joint_names)

        for j, jname in enumerate(self.joint_names):
            idx = name_to_idx.get(jname)
            if idx is None:
                continue
            if idx >= len(msg.position):
                continue

            positions[j] = msg.position[idx]

            if idx < len(msg.effort):
                efforts[j] = msg.effort[idx]

            if idx < len(msg.velocity):
                velocities[j] = msg.velocity[idx]

        # --- Effort fallback to estimator ---
        # real==True: estimator is used ONLY if ALL real efforts are ~zero
        # real==False: keep old sim behavior (per-joint fallback allowed)
        if self.metrics_active and self.use_estimated_effort:
            have_est = all(v is not None for v in self.latest_est_effort)

            effort_valid_any = any(
                (e is not None and abs(e) > self.effort_zero_tol) for e in efforts
            )

            if have_est and (not effort_valid_any):
                # all efforts look invalid -> replace all with estimator
                efforts = list(self.latest_est_effort)
            elif (not self.real) and have_est:
                # SIM behavior: per-joint patching allowed
                for j in range(len(efforts)):
                    if efforts[j] is None or abs(efforts[j]) <= self.effort_zero_tol:
                        efforts[j] = self.latest_est_effort[j]

        # If not inside motion, do not accumulate metrics; still update prev state
        if not self.metrics_active:
            self.prev_js_time = now
            self.prev_js_positions = positions
            return

        # 1) Effort maxima
        for j, eff in enumerate(efforts):
            if eff is None:
                continue
            abs_eff = abs(float(eff))
            if abs_eff > self.max_abs_effort[j]:
                self.max_abs_effort[j] = abs_eff

        # 2) Velocity maxima
        if self.real:
            # REAL: take driver values directly (no checks)
            for j, v in enumerate(velocities):
                if v is None:
                    continue
                abs_v = abs(float(v))
                if abs_v > self.max_abs_velocity[j]:
                    self.max_abs_velocity[j] = abs_v
        else:
            # SIM: finite-difference on position
            if self.prev_js_time is not None and self.prev_js_positions is not None:
                dt = now - self.prev_js_time
                if dt > 1e-6:
                    for j in range(len(self.joint_names)):
                        prev_pos = self.prev_js_positions[j]
                        curr_pos = positions[j]
                        if prev_pos is None or curr_pos is None:
                            continue
                        v_est = (curr_pos - prev_pos) / dt
                        abs_v = abs(v_est)
                        if abs_v > self.max_abs_velocity[j]:
                            self.max_abs_velocity[j] = abs_v

        self.prev_js_time = now
        self.prev_js_positions = positions

    def _on_fallback_finalize_timer(self):
        if self.pending_finalize_timer is not None:
            self.pending_finalize_timer.cancel()
            self.pending_finalize_timer = None

        if self.current_cycle_start is None or not self.awaiting_cycle_end:
            self.pending_end_time = None
            return

        t_end = self.pending_end_time if self.pending_end_time is not None else self.get_now_sec()
        self.pending_end_time = None
        self._finalize_cycle(t_end, end_reason="zone_fall_grace_expired")

    def on_zone_busy(self, msg: Bool):
        z = bool(msg.data)
        now = self.get_now_sec()

        if not self.last_zone_busy and z:
            if self.pending_finalize_timer is not None:
                self.pending_finalize_timer.cancel()
                self.pending_finalize_timer = None
            self.pending_end_time = None
            self.current_cycle_start = now

            self.cycle_speed_latched = float(self.current_speed)
            self.cycle_payload_mass_latched = float(self.payload_mass)
            self.awaiting_cycle_end = True

            self.current_cycle_had_ok = False
            self.current_cycle_had_stop = False

            self.max_abs_effort = [0.0] * len(self.joint_names)
            self.max_abs_velocity = [0.0] * len(self.joint_names)

            self.prev_js_time = None
            self.prev_js_positions = [None] * len(self.joint_names)

            self.metrics_active = True

            self.get_logger().info(f"Cycle {self.cycle_index} started.")

        elif self.last_zone_busy and not z and self.current_cycle_start is not None and self.awaiting_cycle_end:
            self.metrics_active = False
            self.pending_end_time = now

            if self.current_cycle_had_ok:
                if self.pending_finalize_timer is not None:
                    self.pending_finalize_timer.cancel()
                    self.pending_finalize_timer = None

                t_end = self.pending_end_time
                self.pending_end_time = None
                self._finalize_cycle(t_end, end_reason="zone_fall_ok_already")
            else:
                if self.pending_finalize_timer is None:
                    self.pending_finalize_timer = self.create_timer(
                        self.cycle_ok_grace_sec,
                        self._on_fallback_finalize_timer
                    )

        self.last_zone_busy = z

    def _finalize_cycle(self, t_end: float, end_reason: str = ""):
        if self.current_cycle_start is None:
            return

        t_start = self.current_cycle_start
        duration = t_end - t_start

        self.metrics_active = False

        success = (self.current_cycle_had_ok and (not self.current_cycle_had_stop))

        torque_ratios = []
        velocity_ratios = []
        for j in range(len(self.joint_names)):
            tau_max = self.tau_max[j]
            vel_max = self.vel_max[j]

            tr = (self.max_abs_effort[j] / tau_max) if tau_max > 0.0 else 0.0
            vr = (self.max_abs_velocity[j] / vel_max) if vel_max > 0.0 else 0.0

            torque_ratios.append(tr)
            velocity_ratios.append(vr)

        max_torque_ratio = max(torque_ratios) if torque_ratios else 0.0
        max_velocity_ratio = max(velocity_ratios) if velocity_ratios else 0.0

        summary = Float64MultiArray()
        summary.data = [
            float(self.cycle_index),
            float(self.cycle_payload_mass_latched),
            float(self.cycle_speed_latched),
            float(duration),
            1.0 if success else 0.0,
            1.0 if self.current_cycle_had_stop else 0.0,
            float(max_velocity_ratio),
            float(max_torque_ratio),
        ]
        self.summary_pub.publish(summary)

        self.get_logger().info(
            f"Cycle {self.cycle_index} ended ({end_reason}): "
            f"speed={self.cycle_speed_latched:.3f}, "
            f"duration={duration:.3f}s, "
            f"success={success}, "
            f"had_stop={self.current_cycle_had_stop}, "
            f"max_torque_ratio={max_torque_ratio:.3f}, "
            f"max_velocity_ratio={max_velocity_ratio:.3f}"
        )

        row = [
            self.run_id,
            self.cycle_index,
            f"{self.cycle_payload_mass_latched:.3f}",
            f"{self.cycle_speed_latched:.3f}",
            f"{t_start:.6f}",
            f"{t_end:.6f}",
            f"{duration:.6f}",
            int(success),
            int(self.current_cycle_had_stop),
            f"{max_torque_ratio:.6f}",
            f"{max_velocity_ratio:.6f}",
        ]

        for j in range(len(self.joint_names)):
            row.extend([
                f"{self.max_abs_effort[j]:.6f}",
                f"{torque_ratios[j]:.6f}",
                f"{self.max_abs_velocity[j]:.6f}",
                f"{velocity_ratios[j]:.6f}",
            ])

        self.csv_writer.writerow(row)
        self.csv_file.flush()

        self.cycle_index += 1
        self.current_cycle_start = None
        self.awaiting_cycle_end = False

        self.current_cycle_had_ok = False
        self.current_cycle_had_stop = False

    def destroy_node(self):
        try:
            if hasattr(self, 'csv_file') and self.csv_file:
                self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CycleLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
