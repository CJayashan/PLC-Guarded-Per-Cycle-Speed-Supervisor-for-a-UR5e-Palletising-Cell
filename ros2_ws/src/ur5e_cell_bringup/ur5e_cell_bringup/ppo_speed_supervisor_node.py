#!/usr/bin/env python3
import os
import time
import csv
from dataclasses import dataclass
from typing import Optional, Callable
from rclpy.exceptions import ParameterAlreadyDeclaredException

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray

import gymnasium as gym
from gymnasium import spaces

from stable_baselines3 import PPO

# =========================
# Topics (keep same)
# =========================
SPEED_CMD_TOPIC = "/speed_set_external"
CYCLE_SUMMARY_TOPIC = "/cycle_summary"
PAYLOAD_MASS_TOPIC = "/payload_mass"   # purely for logging/telemetry (does NOT change Gazebo physics)

# =========================
# Defaults (can override via ROS params)
# =========================
DEFAULT_MODE = "train"      # train | finetune | eval
DEFAULT_RUN_ID = "run0"
DEFAULT_MODEL_DIR = "."
DEFAULT_MASS = 3.377

DEFAULT_TRAIN_STEPS = 256
DEFAULT_FINETUNE_STEPS = 128
DEFAULT_EVAL_STEPS = 256

# Speed bounds
MIN_SPEED = 0.30
MAX_SPEED = 0.90
SPEED_RESOLUTION = 0.01
DEFAULT_SPEED = 0.50

# Episode / warmup behavior
EPISODE_LEN = 256
IGNORE_FIRST_N_CYCLES = 4

# Safety clamp as function of mass (heavier -> lower allowed max speed)
MASS_SPEED_SLOPE = 0.02

# Reward weights
W_TIME = 1.0
W_VEL  = 1.0
W_TAU  = 0.0
W_MASS = 1.0            # keep 0 in sim unless you really want mass penalty
STOP_PENALTY = 8.0
FAIL_PENALTY = 8.0

SAFE_VEL_RATIO_LIMIT = 0.03
SAFE_TAU_RATIO_LIMIT = 0.60
HINGE_SLOPE = 2.0

SUMMARY_WAIT_TIMEOUT_SEC = 60.0

# PPO hyperparameters
PPO_N_STEPS = 64
PPO_BATCH_SIZE = 64
PPO_LEARNING_RATE = 2e-4
PPO_GAMMA = 0.95
PPO_ENT_COEF = 0.01

REQUIRE_SPEED_MATCH = True
SPEED_MATCH_TOL = 0.02
USE_SIM_TIME = True


# =========================
# Helpers
# =========================
def max_speed_allowed(mass: float) -> float:
    return max(MIN_SPEED, MAX_SPEED - MASS_SPEED_SLOPE * mass)

def round_speed(s: float) -> float:
    s = round(s / SPEED_RESOLUTION) * SPEED_RESOLUTION
    return float(s)

def action_to_speed(action_value: float, mass: float) -> float:
    # action_value in [-1, 1] -> speed in [MIN_SPEED, MAX_SPEED]
    speed = MIN_SPEED + (action_value + 1.0) * 0.5 * (MAX_SPEED - MIN_SPEED)
    speed = min(speed, max_speed_allowed(mass))
    speed = round_speed(speed)
    speed = float(max(MIN_SPEED, min(speed, max_speed_allowed(mass))))
    return speed

def hinge_penalty(x: float, limit: float) -> float:
    return max(0.0, x - limit) * HINGE_SLOPE

def compute_reward(duration: float,
                   max_vel_ratio: float,
                   max_tau_ratio: float,
                   mass: float,
                   success: bool,
                   had_stop: bool) -> float:
    r_time = -W_TIME * duration
    vel_pen = hinge_penalty(max_vel_ratio, SAFE_VEL_RATIO_LIMIT)
    tau_pen = hinge_penalty(max_tau_ratio, SAFE_TAU_RATIO_LIMIT)

    r_vel = -W_VEL * vel_pen
    r_tau = -W_TAU * tau_pen
    r_mass = -W_MASS * mass

    r_stop = -STOP_PENALTY if had_stop else 0.0
    r_fail = -FAIL_PENALTY if not success else 0.0

    return float(r_time + r_vel + r_tau + r_mass + r_stop + r_fail)


# =========================
# Cycle summary container
# =========================
@dataclass
class CycleSummary:
    cycle_index: int
    payload_mass: float
    speed_used: float
    duration: float
    success: bool
    had_stop: bool
    max_vel_ratio: float
    max_tau_ratio: float


# =========================
# ROS bridge node
# =========================
class PPOBridge(Node):
    def __init__(self):
        super().__init__("ppo_speed_trainer")

        # Parameters (override with --ros-args -p ...)
        try:
            self.declare_parameter("use_sim_time", USE_SIM_TIME)
        except ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter("mode", DEFAULT_MODE)
        self.declare_parameter("run_id", DEFAULT_RUN_ID)
        self.declare_parameter("model_dir", DEFAULT_MODEL_DIR)
        self.declare_parameter("payload_mass", float(DEFAULT_MASS))

        self.declare_parameter("train_steps", int(DEFAULT_TRAIN_STEPS))
        self.declare_parameter("finetune_steps", int(DEFAULT_FINETUNE_STEPS))
        self.declare_parameter("eval_steps", int(DEFAULT_EVAL_STEPS))

        # Derived files from run_id
        self.mode = str(self.get_parameter("mode").value)
        self.run_id = str(self.get_parameter("run_id").value)
        self.model_dir = str(self.get_parameter("model_dir").value)
        os.makedirs(self.model_dir, exist_ok=True)

        self.model_path = os.path.join(self.model_dir, f"ppo_speed_model_{self.run_id}.zip")
        self.log_csv_path = os.path.join(self.model_dir, f"ppo_debug_log_{self.run_id}.csv")

        # ROS I/O
        self.pub_speed = self.create_publisher(Float64, SPEED_CMD_TOPIC, 10)
        self.pub_mass  = self.create_publisher(Float64, PAYLOAD_MASS_TOPIC, 10)  # telemetry only
        self.sub_summary = self.create_subscription(
            Float64MultiArray, CYCLE_SUMMARY_TOPIC, self._on_summary, 10
        )

        self._last_summary: Optional[CycleSummary] = None

        self.get_logger().info(
            f"PPOBridge ready | mode={self.mode} run_id={self.run_id} "
            f"model_path={self.model_path} log_csv={self.log_csv_path}"
        )

    def publish_speed(self, speed: float):
        msg = Float64()
        msg.data = float(speed)
        self.pub_speed.publish(msg)

    def publish_mass(self, mass: float):
        msg = Float64()
        msg.data = float(mass)
        self.pub_mass.publish(msg)

    def _on_summary(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) < 8:
            self.get_logger().warn(f"cycle_summary length {len(data)} < 8, ignoring")
            return

        # [cycle_index, payload_mass, speed, duration, success, had_stop, max_vel_ratio, max_torque_ratio]
        s = CycleSummary(
            cycle_index=int(round(data[0])),
            payload_mass=float(data[1]),
            speed_used=float(data[2]),
            duration=float(data[3]),
            success=(float(data[4]) > 0.5),
            had_stop=(float(data[5]) > 0.5),
            max_vel_ratio=float(data[6]),
            max_tau_ratio=float(data[7]),
        )
        self._last_summary = s

    def wait_for_new_summary(self,
                             last_cycle_index: int,
                             expected_speed: Optional[float] = None) -> Optional[CycleSummary]:
        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self._last_summary is None:
                if time.time() - start > SUMMARY_WAIT_TIMEOUT_SEC:
                    self.get_logger().error("Timeout waiting for first cycle_summary.")
                    return None
                continue

            s = self._last_summary

            if s.cycle_index <= last_cycle_index:
                if time.time() - start > SUMMARY_WAIT_TIMEOUT_SEC:
                    self.get_logger().error(
                        f"Timeout waiting for new cycle_summary (last={last_cycle_index}, got={s.cycle_index})."
                    )
                    return None
                continue

            if REQUIRE_SPEED_MATCH and expected_speed is not None:
                if abs(s.speed_used - expected_speed) > SPEED_MATCH_TOL:
                    if time.time() - start > SUMMARY_WAIT_TIMEOUT_SEC:
                        self.get_logger().warn(
                            f"Timeout waiting for speed match. Returning summary anyway. "
                            f"expected={expected_speed:.3f}, got={s.speed_used:.3f}"
                        )
                        return s
                    continue

            return s

        return None


# =========================
# Gym environment: 1 step = 1 cycle
# =========================
class PPOCycleEnv(gym.Env):
    """
    Observation: [mass, last_speed, last_duration, last_success, last_had_stop, last_max_vel_ratio, last_max_tau_ratio]
    Action: one float in [-1, 1]
    """
    metadata = {}

    def __init__(self, bridge: PPOBridge, get_mass: Callable[[], float]):
        super().__init__()
        self.bridge = bridge
        self.get_mass = get_mass

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

        low = np.array([0.0,  0.0,  0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        high = np.array([50.0, 1.0, 60.0, 1.0, 1.0, 2.0, 2.0], dtype=np.float32)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        self.last_cycle_index = -1
        self.step_count_in_episode = 0

        self.last_speed = float(DEFAULT_SPEED)
        self.last_duration = 0.0
        self.last_success = 1.0
        self.last_had_stop = 0.0
        self.last_max_vel_ratio = 0.0
        self.last_max_tau_ratio = 0.0

        # CSV debug log (per run_id)
        self._csv_file = open(self.bridge.log_csv_path, "a", newline="")
        self._csv_writer = csv.writer(self._csv_file)

        if self._csv_file.tell() == 0:
            self._csv_writer.writerow([
                "run_id", "t_wall", "mass_param", "cycle_index",
                "cmd_speed", "summary_speed", "duration",
                "success", "had_stop", "max_vel_ratio", "max_tau_ratio", "reward"
            ])
            self._csv_file.flush()

    def _get_obs(self) -> np.ndarray:
        mass = float(self.get_mass())
        return np.array([
            mass,
            float(self.last_speed),
            float(self.last_duration),
            float(self.last_success),
            float(self.last_had_stop),
            float(self.last_max_vel_ratio),
            float(self.last_max_tau_ratio),
        ], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        self.step_count_in_episode = 0
        self.last_speed = float(DEFAULT_SPEED)
        self.last_duration = 0.0
        self.last_success = 1.0
        self.last_had_stop = 0.0
        self.last_max_vel_ratio = 0.0
        self.last_max_tau_ratio = 0.0
        return self._get_obs(), {}

    def step(self, action):
        self.step_count_in_episode += 1

        mass = float(self.get_mass())

        a = float(np.clip(action[0], -1.0, 1.0))
        cmd_speed = action_to_speed(a, mass)

        # Telemetry mass + speed for next cycle
        self.bridge.publish_mass(mass)
        self.bridge.publish_speed(cmd_speed)

        summary = self.bridge.wait_for_new_summary(
            last_cycle_index=self.last_cycle_index,
            expected_speed=cmd_speed
        )

        if summary is None:
            reward = -999.0
            terminated = True
            truncated = False
            info = {"reason": "summary_timeout"}
            return self._get_obs(), reward, terminated, truncated, info

        self.last_cycle_index = summary.cycle_index

        if summary.cycle_index < IGNORE_FIRST_N_CYCLES:
            self.last_speed = summary.speed_used
            self.last_duration = summary.duration
            self.last_success = 1.0 if summary.success else 0.0
            self.last_had_stop = 1.0 if summary.had_stop else 0.0
            self.last_max_vel_ratio = summary.max_vel_ratio
            self.last_max_tau_ratio = summary.max_tau_ratio
            return self._get_obs(), 0.0, False, False, {"warmup_ignored": True}

        reward = compute_reward(
            duration=summary.duration,
            max_vel_ratio=summary.max_vel_ratio,
            max_tau_ratio=summary.max_tau_ratio,
            mass=mass,
            success=summary.success,
            had_stop=summary.had_stop
        )

        self.last_speed = summary.speed_used
        self.last_duration = summary.duration
        self.last_success = 1.0 if summary.success else 0.0
        self.last_had_stop = 1.0 if summary.had_stop else 0.0
        self.last_max_vel_ratio = summary.max_vel_ratio
        self.last_max_tau_ratio = summary.max_tau_ratio

        self._csv_writer.writerow([
            self.bridge.run_id, time.time(), mass, summary.cycle_index,
            cmd_speed, summary.speed_used, summary.duration,
            int(summary.success), int(summary.had_stop),
            summary.max_vel_ratio, summary.max_tau_ratio,
            reward
        ])
        self._csv_file.flush()

        terminated = False
        truncated = (self.step_count_in_episode >= EPISODE_LEN)
        return self._get_obs(), reward, terminated, truncated, {}

    def close(self):
        try:
            self._csv_file.close()
        except Exception:
            pass


# =========================
# Main
# =========================
def _try_load_model(path: str, env: PPOCycleEnv, bridge: PPOBridge) -> Optional[PPO]:
    if os.path.isfile(path):
        try:
            m = PPO.load(path, env=env)
            bridge.get_logger().info(f"Loaded model: {path}")
            return m
        except Exception as e:
            bridge.get_logger().warn(f"Failed to load model {path}: {e}")
    return None

def main():
    rclpy.init()
    bridge = PPOBridge()

    mode = str(bridge.get_parameter("mode").value).lower().strip()
    mass_param = float(bridge.get_parameter("payload_mass").value)

    train_steps = int(bridge.get_parameter("train_steps").value)
    finetune_steps = int(bridge.get_parameter("finetune_steps").value)
    eval_steps = int(bridge.get_parameter("eval_steps").value)

    # mass is fixed per run (you will spawn Gazebo with matching URDF mass)
    def get_mass():
        return float(mass_param)

    env = PPOCycleEnv(bridge, get_mass)

    # Always try to load first (train also continues if file exists)
    model = _try_load_model(bridge.model_path, env, bridge)

    if model is None:
        model = PPO(
            policy="MlpPolicy",
            env=env,
            verbose=1,
            n_steps=PPO_N_STEPS,
            batch_size=PPO_BATCH_SIZE,
            learning_rate=PPO_LEARNING_RATE,
            gamma=PPO_GAMMA,
            ent_coef=PPO_ENT_COEF,
        )
        bridge.get_logger().warn("Created NEW PPO model (no existing model found)")

    bridge.get_logger().info(
        f"RUN CONFIG | mode={mode} run_id={bridge.run_id} payload_mass={mass_param:.3f} "
        f"train_steps={train_steps} finetune_steps={finetune_steps} eval_steps={eval_steps}"
    )

    if mode == "train":
        bridge.get_logger().info("TRAIN: continuing/learning on this mass scenario...")
        model.learn(total_timesteps=int(train_steps), reset_num_timesteps=False)
        model.save(bridge.model_path)
        bridge.get_logger().info(f"Saved model to {bridge.model_path}")

    elif mode == "finetune":
        bridge.get_logger().info("FINETUNE: continuing learning on this mass scenario (physical-friendly)...")
        model.learn(total_timesteps=int(finetune_steps), reset_num_timesteps=False)
        model.save(bridge.model_path)
        bridge.get_logger().info(f"Saved finetuned model to {bridge.model_path}")

    elif mode == "eval":
        bridge.get_logger().info("EVAL: no learning, deterministic actions...")
        obs, _ = env.reset()
        for k in range(int(eval_steps)):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action)
            bridge.get_logger().info(f"[EVAL] step={k} action={float(action[0]):.3f} reward={reward:.3f}")
            if terminated or truncated:
                obs, _ = env.reset()

    else:
        bridge.get_logger().error(f"Unknown mode: {mode} (use train|finetune|eval)")

    env.close()
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


"""
    # Mass 1.0 kg run
ros2 run ur5e_cell_bringup ppo_speed_supervisor_node \
  --ros-args -p mode:=train -p run_id:=main_v1 -p payload_mass:=1.0 -p train_steps:=1000 -p model_dir:=/home/withanage/logs

# Mass 2.0 kg run (continues main_v1 model)
ros2 run ur5e_cell_bringup ppo_speed_supervisor_node \
  --ros-args -p mode:=train -p run_id:=main_v1 -p payload_mass:=2.0 -p train_steps:=1000 -p model_dir:=/home/withanage/logs

 ros2 run ur5e_cell_bringup ppo_speed_supervisor_node \
  --ros-args -p mode:=finetune -p run_id:=main_v1 -p payload_mass:=3.4 -p finetune_steps:=300 -p use_sim_time:=false -p model_dir:=/home/withanage/logs
   

"""