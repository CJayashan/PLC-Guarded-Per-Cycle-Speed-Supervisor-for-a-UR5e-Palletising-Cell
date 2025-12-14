import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState


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
        # Keep this in sync with the payload_mass xacro arg in ur5e_cell_gazebo.urdf.xacro.
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
        # Names as they appear in /joint_states
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        # Short labels for CSV columns
        self.joint_short_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_1",
            "wrist_2",
            "wrist_3",
        ]

        # Previous joint state for finite-difference velocity
        self.prev_js_time = None                       # float (sec)
        self.prev_js_positions = [None] * len(self.joint_names)

        # Per-cycle maxima (already there, but just to be clear)
        self.max_abs_effort = [0.0] * len(self.joint_names)
        self.max_abs_velocity = [0.0] * len(self.joint_names)
        
        
        # Vendor-like max joint torques for UR5e (Nm):
        # Base, Shoulder, Elbow: 150 Nm; Wrists: 28 Nm. :contentReference[oaicite:2]{index=2}
        self.tau_max = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]

        # Max joint speed: 180 deg/s = pi rad/s for all joints. :contentReference[oaicite:3]{index=3}
        self.vel_max = [math.radians(180.0)] * 6

        # Map from joint name -> index in JointState arrays
        self.joint_index_map = {}

        # Per-motion metrics (reset every time zone_busy rises)
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
            'payload_mass',          # kg
            'speed',                 # current speed_set during this motion
            't_start',
            't_end',
            'duration',
            'success',
            'had_stop_req',
            'max_torque_ratio',      # max over all joints
            'max_velocity_ratio',    # max over all joints
        ]

        # Per-joint columns: max effort/velocity and their ratios
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

        # Subscriptions
        self.create_subscription(Bool, 'zone_busy', self.on_zone_busy, 10)
        self.create_subscription(Bool, 'cycle_ok', self.on_cycle_ok, 10)
        self.create_subscription(Bool, 'stop_req', self.on_stop_req, 10)
        self.create_subscription(Float64, 'speed_set', self.on_speed_set, 10)
        self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            50,
        )

    def get_now_sec(self) -> float:
        """Return current ROS time in seconds (float)."""
        now = self.get_clock().now()
        return now.nanoseconds / 1e9

    # --- Callbacks ---

    def on_speed_set(self, msg: Float64):
        # Just remember latest speed value
        self.current_speed = float(msg.data)

    def on_cycle_ok(self, msg: Bool):
        # If any True pulse happens during a cycle, mark success
        if msg.data:
            self.current_cycle_had_ok = True

    def on_stop_req(self, msg: Bool):
        # If stop_req ever becomes True during a cycle, mark it
        if msg.data:
            self.current_cycle_had_stop = True

    def on_joint_state(self, msg: JointState):
        """
        Update per-motion max torque and velocity while metrics_active is True.
        """
        # Build index map on first message
        if not self.joint_index_map:
            for idx, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.joint_index_map[name] = idx
            # If we couldn't match anything, nothing to do
            if not self.joint_index_map:
                return

        if not self.metrics_active:
            # Not currently in a motion; ignore values
            return

        # Update metrics for each joint we care about
        for j, name in enumerate(self.joint_names):
            idx = self.joint_index_map.get(name)
            if idx is None:
                continue
            if idx >= len(msg.name):
                continue

            # Effort (Nm)
            if idx < len(msg.effort):
                effort = msg.effort[idx]
                self.max_abs_effort[j] = max(self.max_abs_effort[j], abs(effort))

            # Velocity (rad/s)
            if idx < len(msg.velocity):
                vel = msg.velocity[idx]
                self.max_abs_velocity[j] = max(self.max_abs_velocity[j], abs(vel))
    

    def joint_states_callback(self, msg: JointState):
        # 1) Current time in seconds (matches what you use for t_start/t_end)
        now = self.get_clock().now().nanoseconds * 1e-9

        # 2) Build a quick lookup: joint name -> index in this message
        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        # 3) Extract positions and efforts in our fixed joint order
        positions = [0.0] * len(self.joint_names)

        for j, jname in enumerate(self.joint_names):
            idx = name_to_idx.get(jname)
            if idx is None:
                # joint missing from message; skip
                continue

            pos = msg.position[idx]
            eff = msg.effort[idx] if idx < len(msg.effort) else 0.0

            positions[j] = pos

            # update max_abs_effort for this joint
            abs_eff = abs(eff)
            if abs_eff > self.max_abs_effort[j]:
                self.max_abs_effort[j] = abs_eff

        # 4) Estimate velocities from position differences
        if self.prev_js_time is not None:
            dt = now - self.prev_js_time
            if dt > 1e-6:  # avoid division by zero
                for j in range(len(self.joint_names)):
                    prev_pos = self.prev_js_positions[j]
                    curr_pos = positions[j]

                    # skip if we don't have a previous reading
                    if prev_pos is None:
                        continue

                    v_est = (curr_pos - prev_pos) / dt
                    abs_v = abs(v_est)

                    if abs_v > self.max_abs_velocity[j]:
                        self.max_abs_velocity[j] = abs_v

        # 5) Remember for next callback
        self.prev_js_time = now
        self.prev_js_positions = positions


    def on_zone_busy(self, msg: Bool):
        z = bool(msg.data)
        now = self.get_now_sec()

        # Rising edge: start of a motion segment
        if not self.last_zone_busy and z:
            self.current_cycle_start = now
            self.current_cycle_had_ok = False
            self.current_cycle_had_stop = False

            self.max_abs_effort = [0.0] * len(self.joint_names)
            self.max_abs_velocity = [0.0] * len(self.joint_names)

            # Reset per-motion metrics and arm them
            self.metrics_active = True
            self.max_abs_effort = [0.0] * len(self.joint_names)
            self.max_abs_velocity = [0.0] * len(self.joint_names)

            self.get_logger().info(f"Cycle {self.cycle_index} started.")

        # Falling edge: end of a motion segment -> log a row
        elif self.last_zone_busy and not z and self.current_cycle_start is not None:
            t_start = self.current_cycle_start
            t_end = now
            duration = t_end - t_start

            # Stop collecting metrics
            self.metrics_active = False

            # Define success purely as "no stop request during this motion".
            # This matches the safety story and avoids races with /cycle_ok.
            success = not self.current_cycle_had_stop

            # Compute per-joint torque/velocity ratios and global maxima
            torque_ratios = []
            velocity_ratios = []
            for j in range(len(self.joint_names)):
                tau_max = self.tau_max[j]
                vel_max = self.vel_max[j]

                tr = (
                    self.max_abs_effort[j] / tau_max
                    if tau_max > 0.0 else 0.0
                )
                vr = (
                    self.max_abs_velocity[j] / vel_max
                    if vel_max > 0.0 else 0.0
                )

                torque_ratios.append(tr)
                velocity_ratios.append(vr)

            max_torque_ratio = max(torque_ratios) if torque_ratios else 0.0
            max_velocity_ratio = max(velocity_ratios) if velocity_ratios else 0.0

            # Log a summary in the console
            self.get_logger().info(
                f"Cycle {self.cycle_index} ended: "
                f"speed={self.current_speed:.3f}, "
                f"duration={duration:.3f}s, "
                f"success={success}, "
                f"had_stop={self.current_cycle_had_stop}, "
                f"max_torque_ratio={max_torque_ratio:.3f}, "
                f"max_velocity_ratio={max_velocity_ratio:.3f}"
            )

            # Build CSV row
            row = [
                self.run_id,
                self.cycle_index,
                f"{self.payload_mass:.3f}",
                f"{self.current_speed:.3f}",
                f"{t_start:.6f}",
                f"{t_end:.6f}",
                f"{duration:.6f}",
                int(success),
                int(self.current_cycle_had_stop),
                f"{max_torque_ratio:.6f}",
                f"{max_velocity_ratio:.6f}",
            ]

            # Per-joint details (for later plotting)
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

        self.last_zone_busy = z

    def destroy_node(self):
        # Make sure CSV is closed cleanly
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
