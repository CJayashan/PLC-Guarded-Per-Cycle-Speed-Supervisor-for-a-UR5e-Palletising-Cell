import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState


# === Global joint + payload config for UR5e ===

# Joint names (same order as logger / UR5e)
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Vendor-like max joint torques [Nm] (UR5e datasheet-style)
VENDOR_TAU_MAX = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]

# Vendor max joint speeds [rad/s] ≈ 180 deg/s
VENDOR_VEL_MAX = [math.radians(180.0)] * 6

# Safety scaling: we treat these as our "1.0" safe limit for reward
SAFE_TORQUE_SCALE = 0.7   # 70% of vendor torque
SAFE_VEL_SCALE    = 0.5   # 50% of vendor velocity

SAFE_TAU_MAX = [SAFE_TORQUE_SCALE * t for t in VENDOR_TAU_MAX]
SAFE_VEL_MAX = [SAFE_VEL_SCALE * v for v in VENDOR_VEL_MAX]

# Payload safety: we encourage staying around/below this
SAFE_PAYLOAD_MAX = 5.0  # [kg]

# Reward weights (you can tune these later)
TIME_REWARD_WEIGHT   = 1.0
TORQUE_REWARD_WEIGHT = 1.0
VEL_REWARD_WEIGHT    = 1.0
MASS_REWARD_WEIGHT   = 0.5

# ============================================================
# Global tuning parameters 
# ============================================================

# Candidate speeds the bandit can choose between
DEFAULT_SPEED_CANDIDATES = [0.3, 0.35, 0.4, 0.45, 0.5,
                            0.55, 0.6, 0.65, 0.7, 0.8, 0.9]

# Initial speed for the very first motion
DEFAULT_INITIAL_SPEED = 0.3

# UCB exploration weight:
#  - bigger  => explores more
#  - smaller => exploits “best so far” more
DEFAULT_UCB_BETA = 1.0

# Extra “punishment” in reward if we see a stop_req in a motion
# (this is added in seconds-equivalent)
DEFAULT_STOP_PENALTY = 5.0

# How many initial motions to ignore for learning (warm-up)
DEFAULT_WARMUP_MOTIONS = 3
DEFAULT_PAYLOAD_MASS = 1.0  # [kg]




class BayesianSpeedSupervisor(Node):
    """
    Simple Bayesian-style multi-armed bandit over a discrete set of speeds.

    - Each motion (one zone_busy segment) is an experiment.
    - Reward = -duration  (shorter = better)
      If stop_req happened: reward -= stop_penalty
    - We keep running averages of reward for each speed, and choose
      the next speed using a UCB-style rule:
        score_i = mean_i + ucb_beta * sqrt( ln(total_plays) / count_i )
    """

    def __init__(self):
        super().__init__("bayesian_speed_supervisor")

        # Candidate speeds (discrete set)
        self.declare_parameter("speed_candidates", DEFAULT_SPEED_CANDIDATES )
        self.declare_parameter("initial_speed", DEFAULT_INITIAL_SPEED)

        # UCB / reward parameters
        self.declare_parameter("ucb_beta", DEFAULT_UCB_BETA)         # exploration weight
        self.declare_parameter("stop_penalty", DEFAULT_STOP_PENALTY)     # seconds of extra punishment
        self.declare_parameter("warmup_motions", DEFAULT_WARMUP_MOTIONS)

        # Payload for this scenario (kg) – used in the mass reward
        self.declare_parameter("payload_mass", DEFAULT_PAYLOAD_MASS)
        self.payload_mass = (
            self.get_parameter("payload_mass")
            .get_parameter_value()
            .double_value
        )

        # Load parameters
        speed_list_param = self.get_parameter("speed_candidates").get_parameter_value()
        # Depending on how ROS stores list params, this may be a list of doubles already
        self.speed_candidates = list(speed_list_param.double_array_value or []) \
            if hasattr(speed_list_param, "double_array_value") else list(speed_list_param)

        if not self.speed_candidates:
            # Fallback in case param loading behaves differently
            self.speed_candidates = [0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.8, 0.9]

        self.speed_candidates = sorted(self.speed_candidates)

        self.initial_speed = (
            self.get_parameter("initial_speed")
            .get_parameter_value()
            .double_value
        )
        self.ucb_beta = (
            self.get_parameter("ucb_beta")
            .get_parameter_value()
            .double_value
        )
        self.stop_penalty = (
            self.get_parameter("stop_penalty")
            .get_parameter_value()
            .double_value
        )

        self.warmup_motions = (
            self.get_parameter("warmup_motions")
            .get_parameter_value()
            .integer_value
        )

        # Bandit state: one entry per candidate speed
        n = len(self.speed_candidates)
        self.counts = [0] * n            # how many motions we ran at this speed
        self.sum_rewards = [0.0] * n     # sum of rewards
        self.total_plays = 0             # total motions processed

        # --- Joint-load metrics state (per motion) ---
        self.joint_names = JOINT_NAMES
        self.joint_index_map = {}

        # Max |torque| and |velocity| seen during the current motion
        self.max_abs_effort = [0.0] * len(self.joint_names)
        self.max_abs_velocity = [0.0] * len(self.joint_names)

        # For estimating velocities from joint positions if needed
        self.prev_js_time = None
        self.prev_js_positions = [None] * len(self.joint_names)

        # Flag: only log joint loads while zone_busy == True
        self.metrics_active = False


        # Motion counter so we can ignore the first warm-up motion
        self.motion_index = 0

        # Current chosen speed (for the motion that is running right now)
        self.current_speed = self._choose_initial_speed()

        # Motion tracking state
        self.zone_busy = False
        self.zone_busy = False
        self.current_cycle_start = None  # float seconds
        self.current_cycle_had_stop = False

        # Publisher: external speed command
        self.speed_pub = self.create_publisher(
            Float64, "speed_set_external", 10
        )

        # Subscriptions: cycle markers + stop_req
        self.create_subscription(Bool, "zone_busy", self.on_zone_busy, 10)
        self.create_subscription(Bool, "stop_req", self.on_stop_req, 10)
        self.create_subscription(
            JointState,
            "joint_states",
            self.on_joint_state,
            50,
        )

        # Publish initial speed once
        self._publish_speed()

        self.get_logger().info(
            "BayesianSpeedSupervisor started.\n"
            f"  speed_candidates = {self.speed_candidates}\n"
            f"  initial_speed    = {self.current_speed:.3f}\n"
            f"  ucb_beta         = {self.ucb_beta:.3f}\n"
            f"  stop_penalty     = {self.stop_penalty:.3f}"
        )

    # ----------------- helpers -----------------

    def _now_sec(self) -> float:
        """Return current time in seconds (ROS clock)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_speed(self):
        msg = Float64()
        msg.data = float(self.current_speed)
        self.speed_pub.publish(msg)
        self.get_logger().info(
            f"[Bayes] publishing speed_set_external = {self.current_speed:.3f}"
        )

    def _choose_initial_speed(self) -> float:
        """
        Choose initial speed: if initial_speed is in the candidate list,
        use that; otherwise use the middle candidate.
        """
        if self.initial_speed in self.speed_candidates:
            return self.initial_speed
        mid = len(self.speed_candidates) // 2
        return self.speed_candidates[mid]

    def _index_of_speed(self, speed: float) -> int:
        """Find index of speed in candidate list (exact match)."""
        try:
            return self.speed_candidates.index(speed)
        except ValueError:
            # If something weird happens, pick the closest candidate
            diffs = [abs(speed - s) for s in self.speed_candidates]
            return int(min(range(len(self.speed_candidates)), key=lambda i: diffs[i]))

    # ----------------- callbacks -----------------
    
    def on_joint_state(self, msg: JointState):
        """
        Track per-motion max |effort| and |velocity| for each UR5e joint.

        These are turned into ratios using SAFE_TAU_MAX and SAFE_VEL_MAX
        when the motion ends (zone_busy falling edge).
        """
        # Build the index map on the first message
        if not self.joint_index_map:
            for idx, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.joint_index_map[name] = idx

            if not self.joint_index_map:
                # No joints we care about – nothing to do
                return

        if not self.metrics_active:
            # We're currently not in a motion – ignore
            return

        # Current time (for velocity estimation)
        now = self._now_sec()

        # Positions in our fixed joint order
        positions = [None] * len(self.joint_names)

        for j, jname in enumerate(self.joint_names):
            idx = self.joint_index_map.get(jname)
            if idx is None:
                continue
            if idx >= len(msg.position):
                continue

            pos = msg.position[idx]
            positions[j] = pos

            # Effort: track max |effort|
            if idx < len(msg.effort):
                eff = msg.effort[idx]
                abs_eff = abs(eff)
                if abs_eff > self.max_abs_effort[j]:
                    self.max_abs_effort[j] = abs_eff

        # Estimate velocities from position differences
        if self.prev_js_time is not None and any(p is not None for p in positions):
            dt = now - self.prev_js_time
            if dt > 1e-6:
                for j in range(len(self.joint_names)):
                    prev = self.prev_js_positions[j]
                    curr = positions[j]
                    if prev is None or curr is None:
                        continue

                    v_est = (curr - prev) / dt
                    abs_v = abs(v_est)
                    if abs_v > self.max_abs_velocity[j]:
                        self.max_abs_velocity[j] = abs_v

        self.prev_js_time = now
        self.prev_js_positions = positions

    def on_stop_req(self, msg: Bool):
        """Handle stop requests from OPC UA / PLC.

        If a stop happens while we are in a motion (metrics_active == True),
        we mark this motion as 'had stop' so the bandit can punish it.
        """
        sr = bool(msg.data)
        self.stop_req_active = sr

        # Only care about stops while we are actually in a cycle
        if self.metrics_active and sr:
            self.current_cycle_had_stop = True
            self.get_logger().info(
                "[Bayes] stop_req received during motion -> marking cycle as had_stop_req"
            )

    
    
    def on_zone_busy(self, msg: Bool):
        z = bool(msg.data)
        now = self._now_sec()

        # Rising edge: motion starts
        if (not self.zone_busy) and z:
            self.zone_busy = True
            self.current_cycle_start = now
            self.current_cycle_had_stop = False

            # Reset joint-load metrics for this motion
            self.metrics_active = True
            self.max_abs_effort = [0.0] * len(self.joint_names)
            self.max_abs_velocity = [0.0] * len(self.joint_names)

            # Reset velocity estimation history so each motion is clean
            self.prev_js_time = None
            self.prev_js_positions = [None] * len(self.joint_names)

            self.get_logger().info(
                f"[Bayes] Motion {self.motion_index} started at speed={self.current_speed:.3f}"
            )

        # Falling edge: motion ends
        elif self.zone_busy and (not z) and (self.current_cycle_start is not None):
            self.zone_busy = False
            duration = now - self.current_cycle_start

            # Stop collecting joint metrics
            self.metrics_active = False

            success = not self.current_cycle_had_stop

            # --- Compute joint-load ratios using SAFE limits ---
            torque_ratios = []
            velocity_ratios = []

            for j in range(len(self.joint_names)):
                safe_tau = SAFE_TAU_MAX[j]
                safe_vel = SAFE_VEL_MAX[j]

                tr = self.max_abs_effort[j] / safe_tau if safe_tau > 0.0 else 0.0
                vr = self.max_abs_velocity[j] / safe_vel if safe_vel > 0.0 else 0.0

                torque_ratios.append(tr)
                velocity_ratios.append(vr)

            max_torque_ratio = max(torque_ratios) if torque_ratios else 0.0
            max_velocity_ratio = max(velocity_ratios) if velocity_ratios else 0.0

            # Payload ratio against a "comfortable" payload max
            mass_ratio = max(0.0, self.payload_mass / SAFE_PAYLOAD_MAX)

            # --- Separate reward components (all are "costs") ---
            reward_time = -duration
            reward_torque = -max_torque_ratio
            reward_velocity = -max_velocity_ratio
            reward_mass = -mass_ratio

            # Combined reward used by the bandit
            reward = (
                TIME_REWARD_WEIGHT * reward_time
                + TORQUE_REWARD_WEIGHT * reward_torque
                + VEL_REWARD_WEIGHT * reward_velocity
                + MASS_REWARD_WEIGHT * reward_mass
            )

            if not success:
                reward -= self.stop_penalty

            self.get_logger().info(
                f"[Bayes] Motion {self.motion_index} ended: "
                f"duration={duration:.3f}s, success={success}, "
                f"reward_time={reward_time:.3f}, "
                f"reward_torque={reward_torque:.3f}, "
                f"reward_velocity={reward_velocity:.3f}, "
                f"reward_mass={reward_mass:.3f}, "
                f"combined_reward={reward:.3f}, "
                f"max_torque_ratio={max_torque_ratio:.3f}, "
                f"max_velocity_ratio={max_velocity_ratio:.3f}"
            )

            # Ignore motion 0 for bandit learning (warm-up)
            if self.motion_index < self.warmup_motions:
                self.get_logger().info(
                    "[Bayes] Ignoring motion 0 for bandit learning (warm-up)."
                )
            else:
                self._update_bandit(self.current_speed, reward)

            # Choose speed for the NEXT motion
            self.current_speed = self._choose_next_speed()
            self._publish_speed()

            # Reset cycle start and increment motion index
            self.current_cycle_start = None
            self.motion_index += 1
    # ----------------- bandit logic -----------------

    def _update_bandit(self, speed: float, reward: float):
        idx = self._index_of_speed(speed)
        self.counts[idx] += 1
        self.sum_rewards[idx] += reward
        self.total_plays += 1

        mean = self.sum_rewards[idx] / max(1, self.counts[idx])
        self.get_logger().info(
            f"[Bayes] Updated speed={speed:.3f}: "
            f"count={self.counts[idx]}, mean_reward={mean:.3f}"
        )

    def _choose_next_speed(self) -> float:
        """
        UCB-style choice:
          - Try any never-tried speed first (pure exploration)
          - Otherwise, use:
              mean_i + ucb_beta * sqrt( ln(total_plays) / count_i )
        """
        # If any speeds have not been tried yet, choose the first one
        for i, c in enumerate(self.counts):
            if c == 0:
                self.get_logger().info(
                    f"[Bayes] Choosing untried speed {self.speed_candidates[i]:.3f}"
                )
                return self.speed_candidates[i]

        # If somehow no plays yet, return initial
        if self.total_plays <= 0:
            return self._choose_initial_speed()

        log_n = math.log(self.total_plays + 1.0)

        best_score = None
        best_speed = None

        for i, s in enumerate(self.speed_candidates):
            c = self.counts[i]
            mean = self.sum_rewards[i] / max(1, c)

            bonus = self.ucb_beta * math.sqrt(log_n / c)
            score = mean + bonus

            self.get_logger().debug(
                f"[Bayes] speed={s:.3f}, mean={mean:.3f}, "
                f"bonus={bonus:.3f}, score={score:.3f}"
            )

            if (best_score is None) or (score > best_score):
                best_score = score
                best_speed = s

        self.get_logger().info(
            f"[Bayes] Choosing next speed {best_speed:.3f} "
            f"(score={best_score:.3f})"
        )
        return best_speed


def main(args=None):
    rclpy.init(args=args)
    node = BayesianSpeedSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
