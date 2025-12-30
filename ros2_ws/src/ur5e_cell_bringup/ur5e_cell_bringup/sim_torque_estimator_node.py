#!/usr/bin/env python3
import math
import tempfile
import numpy as np
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.exceptions import ParameterAlreadyDeclaredException



# -------------------------
# Small math helpers
# -------------------------
def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])
    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])
    Rx = np.array([[1,  0,   0],
                   [0, cr, -sr],
                   [0, sr,  cr]])
    return Rz @ Ry @ Rx


def make_T(xyz, rpy):
    x, y, z = xyz
    rr, pp, yy = rpy
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(rr, pp, yy)
    T[:3, 3] = np.array([x, y, z])
    return T


def rot_axis(axis, q):
    ax = np.array(axis, dtype=float)
    n = np.linalg.norm(ax)
    if n < 1e-12:
        return np.eye(4)
    ax = ax / n
    x, y, z = ax
    c = math.cos(q)
    s = math.sin(q)
    C = 1.0 - c
    R = np.array([
        [c + x*x*C,   x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, c + z*z*C],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    return T


# -------------------------
# URDF chain parser + Jacobian (no extra libs)
# -------------------------
class URDFChain:
    def __init__(self, urdf_xml: str, base_link: str, tip_link: str):

        self.chain = self._build_chain_from_urdf(urdf_xml, base_link, tip_link)

    def _build_chain_from_urdf(self, urdf_xml: str, base_link: str, tip_link: str):
        root = ET.fromstring(urdf_xml)

        joints = []
        for j in root.findall("joint"):
            jname = j.attrib.get("name", "")
            jtype = j.attrib.get("type", "")

            parent = j.find("parent").attrib.get("link")
            child = j.find("child").attrib.get("link")

            origin = j.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                if "xyz" in origin.attrib:
                    xyz = [float(x) for x in origin.attrib["xyz"].split()]
                if "rpy" in origin.attrib:
                    rpy = [float(x) for x in origin.attrib["rpy"].split()]

            axis_el = j.find("axis")
            axis = [0.0, 0.0, 1.0]
            if axis_el is not None and "xyz" in axis_el.attrib:
                axis = [float(x) for x in axis_el.attrib["xyz"].split()]

            joints.append({
                "name": jname,
                "type": jtype,
                "parent": parent,
                "child": child,
                "xyz": xyz,
                "rpy": rpy,
                "axis": axis,
            })

        child_to_joint = {j["child"]: j for j in joints}

        chain_rev = []
        cur = tip_link
        while cur != base_link:
            if cur not in child_to_joint:
                raise RuntimeError(
                    f"Cannot find parent joint for link '{cur}'. "
                    f"Check base_link/tip_link. base='{base_link}' tip='{tip_link}'"
                )
            j = child_to_joint[cur]
            chain_rev.append(j)
            cur = j["parent"]

        return list(reversed(chain_rev))  # base->tip

    def jacobian(self, q_map: dict):
        # forward kinematics, storing joint axis and pos for revolute joints
        T = np.eye(4)

        joint_p = []
        joint_z = []
        active_names = []

        for j in self.chain:
            T = T @ make_T(j["xyz"], j["rpy"])

            if j["type"] in ("revolute", "continuous"):
                axis = np.array(j["axis"], dtype=float)
                z = T[:3, :3] @ axis
                p = T[:3, 3].copy()

                joint_z.append(z)
                joint_p.append(p)
                active_names.append(j["name"])

                q = float(q_map.get(j["name"], 0.0))
                T = T @ rot_axis(axis, q)

        p_end = T[:3, 3].copy()

        J = np.zeros((6, len(active_names)))
        for i in range(len(active_names)):
            z = joint_z[i]
            p = joint_p[i]
            Jv = np.cross(z, (p_end - p))
            Jw = z
            J[:3, i] = Jv
            J[3:, i] = Jw

        return J, active_names


# -------------------------
# Main node
# -------------------------
class TorqueEstimatorNode(Node):
    """
    Publishes estimated torque/effort for UR5e joints:
      tau_total = tau_robot_gravity(pinocchio) + J^T * payload_wrench

    - No rclpy.parameter_client used (Humble-safe)
    - Pinocchio is optional (use_pinocchio:=false if you can't/don't want it)
    """

    def __init__(self):
        super().__init__("torque_estimator")

        self.declare_parameter("robot_description_node", "robot_state_publisher")
        self.declare_parameter("robot_description_param", "robot_description")
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "payload_box")  # Gazebo URDF uses payload_box (not tcp_link)


        self.declare_parameter("joint_names", [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ])

        self.declare_parameter("payload_mass", 0.0)
        self.declare_parameter("use_pinocchio", False)  # enable if you install pinocchio
        self.declare_parameter("publish_topic", "estimated_joint_effort")
        self.declare_parameter("rate_hz", 50.0)

        self.robot_desc_node = self.get_parameter("robot_description_node").value
        self.robot_desc_param = self.get_parameter("robot_description_param").value

        # Declare use_sim_time safely (optional, but nice to have)
        try:
            self.declare_parameter("use_sim_time", True)
        except ParameterAlreadyDeclaredException:
            pass

        # Persistent client + future (no nested spinning)
        self._gp_srv = f"/{self.robot_desc_node}/get_parameters"
        self._gp_client = self.create_client(GetParameters, self._gp_srv)
        self._gp_future = None
        self._warned_no_service = False


        self.base_link = self.get_parameter("base_link").value
        self.tip_link = self.get_parameter("tip_link").value
        self.joint_names = list(self.get_parameter("joint_names").value)

        self.payload_mass = float(self.get_parameter("payload_mass").value)
        self.use_pin = bool(self.get_parameter("use_pinocchio").value)

        self.q_map = {}
        self.urdf_xml = ""
        self.chain = None

        self.pin = None
        self.pin_model = None
        self.pin_data = None
        self.pin_joint_vidx = {}  # joint_name -> v index

        topic = self.get_parameter("publish_topic").value
        self.pub = self.create_publisher(Float64MultiArray, topic, 10)

        self.create_subscription(JointState, "joint_states", self.on_joint_states, 50)
        self.create_subscription(Float64, "payload_mass", self.on_payload_mass, 10)

        self._loaded = False
        self.create_timer(0.5, self.try_load_everything)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(1.0, rate_hz), self.tick)

    def on_payload_mass(self, msg: Float64):
        self.payload_mass = float(msg.data)

    def on_joint_states(self, msg: JointState):
        for i, n in enumerate(msg.name):
            if i < len(msg.position):
                self.q_map[n] = float(msg.position[i])

    # ---- Parameter service call workaround (no rclpy.parameter_client) ----
    def request_robot_description(self):
        """Kick off an async GetParameters call (returns immediately)."""
        if not self._gp_client.service_is_ready():
            if not self._warned_no_service:
                self.get_logger().warn(f"Waiting for service {self._gp_srv} ...")
                self._warned_no_service = True
            return

        if self._gp_future is None:
            req = GetParameters.Request()
            req.names = [self.robot_desc_param]
            self._gp_future = self._gp_client.call_async(req)


    def try_import_pinocchio(self):
        if not self.use_pin:
            return False
        if self.pin is not None:
            return True

        try:
            import pinocchio as pin
            self.pin = pin
            return True
        except Exception as e:
            self.get_logger().warn(f"Pinocchio not available (falling back to payload-only). Error: {e}")
            self.pin = None
            return False

    def try_load_everything(self):
        if self._loaded:
            return

        # 1) Start request if not started
        self.request_robot_description()

        # 2) If request not done yet, just wait for next timer tick
        if self._gp_future is None or not self._gp_future.done():
            return

        # 3) Consume result (and clear future so we can retry if needed)
        res = self._gp_future.result()
        self._gp_future = None

        if res is None or len(res.values) == 0:
            self.get_logger().warn("GetParameters returned empty response (will retry).")
            return

        urdf_xml = res.values[0].string_value
        self.get_logger().info(f"Got robot_description length={len(urdf_xml)}")

        if not urdf_xml:
            self.get_logger().warn("robot_description string is empty (will retry).")
            return

        self.urdf_xml = urdf_xml

        # Build chain
        try:
            self.chain = URDFChain(self.urdf_xml, self.base_link, self.tip_link)
        except Exception as e:
            self.get_logger().error(f"Failed to build URDF chain: {e}")
            return

        self.get_logger().info("Built URDF chain successfully.")

        # Optional pinocchio init stays the same (your code below here unchanged)
        if self.try_import_pinocchio():
            try:
                with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=True) as f:
                    f.write(self.urdf_xml)
                    f.flush()
                    self.pin_model = self.pin.buildModelFromUrdf(f.name)
                self.pin_data = self.pin_model.createData()

                for jn in self.joint_names:
                    jid = self.pin_model.getJointId(jn)
                    if jid > 0:
                        self.pin_joint_vidx[jn] = int(self.pin_model.idx_vs[jid])

                self.get_logger().info("Pinocchio model ready (robot gravity torques enabled).")
            except Exception as e:
                self.get_logger().warn(f"Pinocchio init failed (payload-only). Error: {e}")
                self.pin = None
                self.pin_model = None
                self.pin_data = None
                self.pin_joint_vidx = {}

        self._loaded = True
        self.get_logger().info(
            f"TorqueEstimator ready | base={self.base_link} tip={self.tip_link} "
            f"use_pinocchio={self.pin is not None}"
        )


    def compute_robot_gravity_tau(self):
        """
        Returns dict joint_name -> tau_g (static gravity torque) if pinocchio is enabled.
        Otherwise returns zeros.
        """
        out = {jn: 0.0 for jn in self.joint_names}
        if self.pin is None or self.pin_model is None:
            return out

        # Build q in pinocchio order
        q = self.pin.neutral(self.pin_model)

        for jn in self.joint_names:
            jid = self.pin_model.getJointId(jn)
            if jid <= 0:
                continue
            qidx = int(self.pin_model.idx_qs[jid])
            q[qidx] = float(self.q_map.get(jn, 0.0))

        # g(q)
        tau_g = self.pin.computeGeneralizedGravity(self.pin_model, self.pin_data, q)

        for jn in self.joint_names:
            if jn in self.pin_joint_vidx:
                out[jn] = float(tau_g[self.pin_joint_vidx[jn]])

        return out

    def compute_payload_tau(self):
        """
        Payload-only torque from Jacobian transpose:
          tau = J^T * [0,0,-m g,0,0,0]
        """
        if self.chain is None:
            return {jn: 0.0 for jn in self.joint_names}

        J, active_names = self.chain.jacobian(self.q_map)
        m = float(self.payload_mass)
        g = 9.81
        wrench = np.array([0.0, 0.0, -m * g, 0.0, 0.0, 0.0])
        tau = J.T @ wrench

        name_to_tau = {active_names[i]: float(tau[i]) for i in range(len(active_names))}
        return {jn: float(name_to_tau.get(jn, 0.0)) for jn in self.joint_names}

    def tick(self):
        msg = Float64MultiArray()

        # Publish zeros until ready (so echo always shows something)
        if not self._loaded:
            msg.data = [0.0] * len(self.joint_names)
            self.pub.publish(msg)
            return

        # normal compute
        tau_g = self.compute_robot_gravity_tau()
        tau_p = self.compute_payload_tau()

        out = []
        for jn in self.joint_names:
            out.append(tau_g.get(jn, 0.0) + tau_p.get(jn, 0.0))

        msg.data = out
        self.pub.publish(msg)



def main():
    rclpy.init()
    node = TorqueEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
