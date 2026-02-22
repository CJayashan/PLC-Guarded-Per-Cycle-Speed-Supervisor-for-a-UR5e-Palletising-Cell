#!/usr/bin/env python3

import math
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from std_msgs.msg import Float64
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.msg import RobotTrajectory
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

import time

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

POSE_A = [
    1.5251,   # 87.42°
   -1.1297,   # -64.72°
    1.6778,   # 96.12°
   -2.1234,   # -121.64°
   -1.5713,   # -90.03°
    0.1253    # 7.18°
]
POSE_B = [
    3.1143,   # 178.43°
   -1.3546,   # -77.61°
    1.5607,   # 89.42°
   -1.8043,   # -103.37°
   -1.5439,   # -88.45°
    0.1253    # 7.18°
]

# ------------------------------------------------------------------
# Global switch: freeze the planned paths A->B and B->A
# True  -> plan A->B and B->A once, then only time-scale them
# False -> re-plan every cycle (current behavior)
# ------------------------------------------------------------------
FREEZE_PATH = True  # set to False when you want "dynamic" paths

class MoveItSpeedExecutorNode(Node):
    def __init__(self):
        super().__init__("moveit_speed_executor")

        # 1) Parameters
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("speed_topic", "/speed_set")
        self.speed_version = 0
        self.declare_parameter("plan_service", "/plan_kinematic_path")
        self.declare_parameter(
            "jtc_action_name",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("num_cycles", 5)
        self.declare_parameter("min_speed_scale", 0.1)
        self.declare_parameter("max_speed_scale", 1.0)

        self.group_name = self.get_parameter("group_name").get_parameter_value().string_value
        self.speed_topic = self.get_parameter("speed_topic").get_parameter_value().string_value
        self.plan_service = self.get_parameter("plan_service").get_parameter_value().string_value
        self.jtc_action_name = self.get_parameter("jtc_action_name").get_parameter_value().string_value
        self.num_cycles = self.get_parameter("num_cycles").get_parameter_value().integer_value
        self.min_speed_scale = self.get_parameter("min_speed_scale").get_parameter_value().double_value
        self.max_speed_scale = self.get_parameter("max_speed_scale").get_parameter_value().double_value
        
        # Path-freeze mode
        self.freeze_path = FREEZE_PATH
        self.ref_traj_A_to_B = None  # type: RobotTrajectory | None
        self.ref_traj_B_to_A = None  # type: RobotTrajectory | None

        # 2) Speed cache
        self.current_speed = 0.5  # default 50%

        # 3) Subscribers
        self.speed_sub = self.create_subscription(
            Float64, self.speed_topic, self._speed_callback, 10
        )

        self._last_js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # 4) MoveIt planning service client
        self.plan_client = self.create_client(GetMotionPlan, self.plan_service)

        # 5) JTC action client
        self.jtc_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.jtc_action_name,
        )

        self.get_logger().info("MoveItSpeedExecutorNode created.")

    # ---------------------------------------------------
    # Speed execution loop
    # ---------------------------------------------------

    def _speed_callback(self, msg: Float64):
        raw_speed = msg.data
        # Clamp speed to min/max
        if raw_speed < self.min_speed_scale:
            clamped_speed = self.min_speed_scale
        elif raw_speed > self.max_speed_scale:
            clamped_speed = self.max_speed_scale
        else:
            clamped_speed = raw_speed
        
        self.current_speed = clamped_speed
        self.speed_version += 1
        self.get_logger().debug(
            f"Received speed_set={raw_speed:.3f}, using clamped={clamped_speed:.3f}"
        )

    def get_speed_scale(self) -> float:
        """Get the current speed scale (0.0 .. 1.0)."""
        return float(self.current_speed)
    
    def _on_js(self, msg: JointState):
        # Accept only “real” joint states
        if msg.name and msg.position and len(msg.name) == len(msg.position):
            self._last_js = msg


    # ---------------------------------------------------
    # wait for clients to be ready
    # ---------------------------------------------------   

    def wait_for_backends(self):
        """Wait for MoveIt planning service and JTC action to be available."""
        self.get_logger().info(
            f"Waiting for MoveIt planning service '{self.plan_service}'..."
        )
        self.plan_client.wait_for_service()
        self.get_logger().info("MoveIt planning service available.")

        self.get_logger().info(
            f"Waiting for JTC action '{self.jtc_action_name}'..."
        )
        self.jtc_client.wait_for_server()
        self.get_logger().info("JTC action available.")

        if not self.wait_for_joint_states(timeout_sec=5.0):
            self.get_logger().warn("No /joint_states received after waiting. Planning may warn about empty start_state.")

    def wait_for_joint_states(self, timeout_sec: float = 5.0) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and self._last_js is None and (time.monotonic() - t0) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._last_js is not None

    # ---------------------------------------------------
    # Built joint goal constraints and motion plan request
    # ---------------------------------------------------

    def _build_joint_constarints(self, joint_names: List[str], joint_values: List[float], tolerance: float = 1e-3) -> Constraints:
        """Build joint constraints for given joint names and values so that MoveIt can plan to them."""

        c = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c
    
    def plan_to_joints(self, joint_values: List[float]) -> RobotTrajectory:
        """ask MoveIt to plan a motion to the given joint values.
        returns the planned RobotTrajectory. Empty trajectory on failure.
        """

        # Build motion plan request
        req = MotionPlanRequest()
        req.group_name = self.group_name

       
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0  # seconds
        req.goal_constraints.append(
            self._build_joint_constarints(JOINT_NAMES, joint_values)
        )

        # Try briefly to get joint states (in case of timing races)
        if self._last_js is None:
            self.wait_for_joint_states(timeout_sec=1.0)

        if self._last_js is not None:
            # Fill start_state (prevents “empty JointState” warning)
            req.start_state.joint_state.name = list(self._last_js.name)
            req.start_state.joint_state.position = list(self._last_js.position)
            req.start_state.joint_state.velocity = []
            req.start_state.joint_state.effort = []
            req.start_state.is_diff = False
        else:
            # Fallback: allow MoveIt to use "current state" (may print warning, but won’t abort)
            self.get_logger().warn("Still no /joint_states; planning without explicit start_state.")

        # Call planning service
        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = req

        self.get_logger().info(
            f"Requesting plan for the group [{self.group_name}]..."
        )

        future = self.plan_client.call_async(plan_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().info("GetMotionPlan service call failed")
            return RobotTrajectory()  # empty
        
        response = future.result()
        if not response.motion_plan_response.error_code.val == 1:
            self.get_logger().error(
                f"Motion planning failed with error code {response.motion_plan_response.error_code.val}"
            )
            return RobotTrajectory()  # empty
        
        trajectory = response.motion_plan_response.trajectory
        if not trajectory.joint_trajectory.points:
            self.get_logger().error("Received empty trajectory from MoveIt")
        else:
            self.get_logger().info(
                f"Received trajectory with {len(trajectory.joint_trajectory.points)} points."
            )
        return trajectory
    
    # ---------------------------------------------------
    # Scalling trajectory time
    # ---------------------------------------------------

    def scale_trajectory_timing(self, trajectory: RobotTrajectory, speed_scale: float) -> RobotTrajectory:
        """
        Take a RobotTrajectory from MoveIt and scale time_from_start
        in joint_trajectory according to speed_scale.
        speed_scale = 1.0  -> original time
        speed_scale = 0.5  -> twice as slow (double times)
        """

        joint_trajectory = trajectory.joint_trajectory

        if speed_scale <= 0.0:
            self.get_logger().warn(
                f"invalid speed scale = {speed_scale:.3f}, must be > 0.0. Using 0.1 instead.")
            speed_scale = 0.1

        scaled = JointTrajectory()
        scaled.joint_names = list(joint_trajectory.joint_names)

        for point in joint_trajectory.points:
            new_point = JointTrajectoryPoint()

            # 1) Keep positions (this defines the path)
            new_point.positions = list(point.positions)

            # 2) Let the controller figure out velocities/accels itself
            new_point.velocities = []
            new_point.accelerations = []
            new_point.effort = []

            # Scale time_from_start
            if point.time_from_start.sec == 0 and point.time_from_start.nanosec == 0 :
                new_point.time_from_start = point.time_from_start
            else:
                total = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                scaled_time = total / speed_scale
                sec = int(scaled_time)
                nsec = int((scaled_time - sec) * 1e9)
                new_point.time_from_start = Duration(sec=sec, nanosec=nsec)

            scaled.points.append(new_point)
        
        return scaled
    
    # ---------------------------------------------------
    # Execute trajectory via JTC
    # ---------------------------------------------------

    def execute_joint_trajectory(self, jt: JointTrajectory ) -> bool:
        """Send the given JointTrajectory to JTC and wait for completion.
        Returns True on success, False on failure.
        """

        if not jt.points:
            self.get_logger().error("Cannot execute empty JointTrajectory")
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = jt

        self.get_logger().info(
            f"Sending trajectory with {len(jt.points)} points to JTC..."
        )

        # Send goal
        send_goal_future = self.jtc_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("JTC goal rejected")
            return False
        
        self.get_logger().info("JTC goal accepted, waiting for result...")
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()

        if result is None:
            self.get_logger().error("JTC result is None")
            return False
        
        self.get_logger().info(
            f"JTC finished with status {result.status}JTC result error_code={result.result.error_code}"
        )
        
        ok_status = (result.status == GoalStatus.STATUS_SUCCEEDED)
        ok_code = (result.result.error_code == 0)  # for JTC, 0 is typically success
        return ok_status and ok_code
    
    def reverse_robot_traj(self, traj: RobotTrajectory) -> RobotTrajectory:
        out = RobotTrajectory()
        out.joint_trajectory.joint_names = list(traj.joint_trajectory.joint_names)

        pts = traj.joint_trajectory.points
        if not pts:
            return out

        total = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9

        for p in reversed(pts):
            np = JointTrajectoryPoint()
            np.positions = list(p.positions)
            np.velocities = []
            np.accelerations = []
            np.effort = []

            old_t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            new_t = total - old_t
            sec = int(new_t)
            nsec = int((new_t - sec) * 1e9)
            np.time_from_start = Duration(sec=sec, nanosec=nsec)

            out.joint_trajectory.points.append(np)

        return out

    
    # ----------------------
    # MAIN CYCLE LOOP
    # ----------------------

    def run_cycles(self):
        """
        Run num_cycles cycles: initial current->A (once) and then A -> B -> A.
        Each leg uses current speed scaling from /speed_set.
        """
        last_used_version = self.speed_version
        self.get_logger().info(f"Starting cycles: num_cycles={self.num_cycles}")

        for i in range(self.num_cycles):
            self.get_logger().info(f"=== Cycle {i+1}/{self.num_cycles} ===")

            # Wait a bit for a newer speed value (or timeout)
            timeout_secs = 5.0
            start = self.get_clock().now()
            while self.speed_version == last_used_version:
                rclpy.spin_once(self, timeout_sec=0.1)
                if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout_secs:
                    self.get_logger().warn(
                        "Timeout waiting for speed update, continuing with old speed."
                    )
                    break
            last_used_version = self.speed_version

            speed_scale = self.get_speed_scale()
            self.get_logger().info(f"Using speed_scale={speed_scale:.3f}")

            # A) current -> A (only on very first cycle, as a homing move)
            if i == 0:
                self.get_logger().info("Planning initial move to pose A (homing)...")
                traj_to_A = self.plan_to_joints(POSE_A)
                if traj_to_A.joint_trajectory.points:
                    scaled_A = self.scale_trajectory_timing(traj_to_A, speed_scale)
                    ok = self.execute_joint_trajectory(scaled_A)
                    if not ok:
                        self.get_logger().error("Execution to A failed, aborting.")
                        break
                else:
                    self.get_logger().error("No trajectory to A, aborting.")
                    break
            else:
                self.get_logger().info("Skipping current->A leg (already at A).")

            # B) A -> B
            if self.freeze_path and self.ref_traj_A_to_B is not None:
                # Use frozen trajectory
                self.get_logger().info("Using frozen A -> B trajectory.")
                traj_A_to_B = self.ref_traj_A_to_B
            else:
                # Plan a new trajectory (and optionally freeze it)
                self.get_logger().info("Planning A -> B...")
                traj_A_to_B = self.plan_to_joints(POSE_B)
                if self.freeze_path and traj_A_to_B.joint_trajectory.points:
                    self.get_logger().info(
                        f"Freezing A -> B trajectory "
                        f"({len(traj_A_to_B.joint_trajectory.points)} points) for reuse."
                    )
                    self.ref_traj_A_to_B = traj_A_to_B
                    # Derive B->A from frozen A->B (no separate planning)
                    self.ref_traj_B_to_A = self.reverse_robot_traj(traj_A_to_B)
                    self.get_logger().info("Derived frozen B -> A by reversing frozen A -> B.")

            # refresh speed right before A->B leg ---
            rclpy.spin_once(self, timeout_sec=0.0)
            
            if self.speed_version != last_used_version:
                last_used_version = self.speed_version
                speed_scale = self.get_speed_scale()
                self.get_logger().info(
                    f"Speed updated, using new speed_scale for A->B={speed_scale:.3f}"
                )
            # -----------------------------------------------------
            
            if traj_A_to_B.joint_trajectory.points:
                scaled_AB = self.scale_trajectory_timing(traj_A_to_B, speed_scale)
                ok = self.execute_joint_trajectory(scaled_AB)
                if not ok:
                    self.get_logger().error("Execution A -> B failed, aborting.")
                    break
            else:
                self.get_logger().error("No trajectory A -> B, skipping cycle.")
                break

            # C) B -> A
            if self.freeze_path:
                # If not built yet (e.g., starting mid-run), derive it from frozen A->B
                if self.ref_traj_B_to_A is None and self.ref_traj_A_to_B is not None:
                    self.ref_traj_B_to_A = self.reverse_robot_traj(self.ref_traj_A_to_B)
                    self.get_logger().info("Derived frozen B -> A by reversing frozen A -> B.")

                self.get_logger().info("Using frozen B -> A trajectory.")
                traj_B_to_A = self.ref_traj_B_to_A
            else:
                self.get_logger().info("Planning B -> A...")
                traj_B_to_A = self.plan_to_joints(POSE_A)


            # refresh speed between legs if a new speed_set arrived ---
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.speed_version != last_used_version:
                last_used_version = self.speed_version
                speed_scale = self.get_speed_scale()
                self.get_logger().info(
                    f"Speed updated mid-cycle, using new speed_scale for B->A={speed_scale:.3f}"
                )
            # ---------------------------------------------------------------

            
            if traj_B_to_A.joint_trajectory.points:
                scaled_BA = self.scale_trajectory_timing(traj_B_to_A, speed_scale)
                ok = self.execute_joint_trajectory(scaled_BA)
                if not ok:
                    self.get_logger().error("Execution B -> A failed, aborting.")
                    break
            else:
                self.get_logger().error("No trajectory B -> A, skipping cycle.")
                break




def main(args=None):
    rclpy.init(args=args)

    node = MoveItSpeedExecutorNode()
    
    try:
        node.wait_for_backends()
        node.run_cycles()
    finally:
        node.get_logger().info("Shutting down MoveItSpeedExecutorNode...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
