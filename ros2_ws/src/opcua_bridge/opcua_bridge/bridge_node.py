#!/usr/bin/env python3
"""
ROS 2 ↔ OPC UA Bridge (simple & robust)

What it does:
- Acts as an OPC UA *client* (connects to your mock PLC server).
- Polls 4 tags: zone_busy, cycle_ok, stop_req, speed_set.
- Publishes a ROS 2 topic /opcua/status (JSON) **only when connected**.
- Exposes a ROS 2 parameter 'speed_set' — when you set it, the UA tag is written.
- Has a watchdog: on repeated read errors, it reconnects automatically.
- Logs a CSV for proof (/logs/opcua_bridge.csv).

"""

import json
import csv
import time
from datetime import datetime
from pathlib import Path
import os

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

from opcua import Client, ua
from std_msgs.msg import String, Float64, Bool


def iso_now() -> str:
    """Return an ISO timestamp (seconds precision)."""
    return datetime.now().isoformat(timespec="seconds")


class OpcUaBridge(Node):
    def __init__(self) -> None:
        super().__init__('opcua_bridge')

        # ---------------- Parameters (with sensible defaults) ----------------
        # You can override any of these at launch with --ros-args -p name:=value
        self.declare_parameter('endpoint', 'opc.tcp://localhost:4840')  # UA server URL
        self.declare_parameter('ns_idx', 2)                              # your namespace index
        self.declare_parameter('poll_hz', 5.0)                           # read rate
        #self.declare_parameter('speed_set', 0.5)                         # initial dial
        self.declare_parameter('clamp_min', 0.2)                         # safety clamp (soft)
        self.declare_parameter('clamp_max', 1.0)

        # Default CSV path: project_root/logs/opcua_bridge.csv (two dirs up from this file)
        default_csv = Path.home() / "projects/project_root/logs/opcua_bridge.csv"
        self.declare_parameter('csv_path', str(default_csv))

        # Read parameter values into simple attributes
        self.endpoint = self.get_parameter('endpoint').get_parameter_value().string_value
        self.ns_idx = self.get_parameter('ns_idx').get_parameter_value().integer_value
        poll_hz = self.get_parameter('poll_hz').get_parameter_value().double_value
        self.poll_dt = 1.0 / max(0.1, poll_hz)  # avoid divide-by-zero, the loop rate
        self.csv_path = Path(self.get_parameter('csv_path').get_parameter_value().string_value)

        # ---------------- ROS I/O ----------------
        # Publish JSON status here. We publish ONLY when connected and reads succeed.
        self.pub_status = self.create_publisher(String, 'opcua/status', 10)
        self.pub_opcua_manual_mode = self.create_publisher(Bool, 'opcua_manual_mode', 10)
        self.pub_stop_req = self.create_publisher(Bool, 'stop_req', 10)

        # Last values from ROS (for writing into OPC UA)
        self.last_zone_busy = False
        self.last_cycle_ok = False
        self.last_speed_actual = 0.0

        # Subscribe to ROS topics that we want to mirror into OPC UA
        self.create_subscription(Bool, 'zone_busy', self._on_zone_busy, 10)
        self.create_subscription(Bool, 'cycle_ok', self._on_cycle_ok, 10)
        self.create_subscription(Float64, 'speed_set', self._on_speed_actual, 10)

        # Publish raw UA tags as ROS topics 
        self.pub_opcua_speed = self.create_publisher(Float64, 'opcua_speed_set', 10)


        # CSV evidence (create file + header if new)
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open('a', newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)
        if self.csv_path.stat().st_size == 0:
            self.writer.writerow(['ts', 'zone_busy', 'cycle_ok', 'stop_req', 'speed_set', 'manual_mode'])

        # ---------------- OPC UA session state ----------------
        self.client: Client | None = None
        self.n_speed = self.n_speed_actual = self.n_mode = self.n_zbusy = self.n_cycleok = self.n_stopreq = None  # UA node handles
        self.connected = False              # gate publishing
        self.consec_fail = 0                # read failure counter
        self.fail_limit = 5                 # how many bad reads trigger reconnect
        self.reconnect_backoff_s = 2.0      # sleep between reconnect attempts

        # Pre-build NodeIds (string identifiers in your namespace)
        self.nid_speed   = ua.NodeId('speed_set', self.ns_idx)
        self.nid_speed_actual = ua.NodeId('speed_actual', self.ns_idx)
        self.nid_mode    = ua.NodeId('speed_mode_manual', self.ns_idx)
        self.nid_zbusy   = ua.NodeId('zone_busy', self.ns_idx)
        self.nid_cycleok = ua.NodeId('cycle_ok',  self.ns_idx)
        self.nid_stopreq = ua.NodeId('stop_req',  self.ns_idx)

        # Try to connect now (will loop until success)
        self._connect_and_bind()

        # React to parameter changes (we care about 'speed_set')
        #self.add_on_set_parameters_callback(self._on_param_set)

        # Poll timer (fires every poll_dt seconds)
        self.timer = self.create_timer(self.poll_dt, self._poll_once)

    # ---------------- Connection helpers ----------------

    def _connect_and_bind(self) -> None:
        """Create a new UA client session and resolve node handles."""
        while rclpy.ok():
            try:
                cli = Client(self.endpoint)
                # Optional: shorter socket timeout helps detect dead connections faster (milliseconds)
                try:
                    cli.set_timeout(3000)
                except Exception:
                    pass

                cli.connect()  # open Session
                # Resolve handles once (faster than get_node every loop)
                self.n_speed   = cli.get_node(self.nid_speed)
                self.n_speed_actual = cli.get_node(self.nid_speed_actual)
                self.n_mode    = cli.get_node(self.nid_mode)
                self.n_zbusy   = cli.get_node(self.nid_zbusy)
                self.n_cycleok = cli.get_node(self.nid_cycleok)
                self.n_stopreq = cli.get_node(self.nid_stopreq)

                self.client = cli
                self.connected = True
                self.consec_fail = 0
                self.get_logger().info(f'OPC UA connected: {self.endpoint}')

                # Push current parameter to UA on (re)connect
                #sp = self.get_parameter('speed_set').get_parameter_value().double_value
                #self._write_speed(sp, log_ok=True)
                return

            except Exception as e:
                self.connected = False
                self.get_logger().warn(f'Connect failed: {e}  (retrying in {self.reconnect_backoff_s}s)')
                time.sleep(self.reconnect_backoff_s)

    def _disconnect_silent(self) -> None:
        """Close the session if it exists; ignore errors."""
        try:
            if self.client is not None:
                self.client.disconnect()
        except Exception:
            pass
        finally:
            self.client = None
            self.connected = False

    # ---------------- Parameter handling ----------------

    def _on_param_set(self, params) -> SetParametersResult:
        """
        Called when any parameter changes.
        We only *act* on 'speed_set'. Others are accepted as-is.
        """
        ok = True
        for p in params:
            if p.name == 'speed_set':
                try:
                    # Grab the numeric value (rclpy gives a wrapper)
                    new_val = p.value.double_value if hasattr(p.value, 'double_value') else float(p.value)
                    #self._write_speed(new_val, log_ok=True)
                except Exception as e:
                    ok = False
                    self.get_logger().error(f'Failed to write speed_set to OPC UA: {e}')
        return SetParametersResult(successful=ok)

    def _write_speed(self, value: float, log_ok: bool = False) -> None:
        """Clamp and write speed_set to the UA server (Float)."""
        if not self.connected or self.client is None:
            raise RuntimeError('No OPC UA session')

        lo = self.get_parameter('clamp_min').get_parameter_value().double_value
        hi = self.get_parameter('clamp_max').get_parameter_value().double_value
        v  = max(lo, min(hi, float(value)))
        if v != value:
            self.get_logger().warn(f'speed_set clamped {value} → {v}')

        self.n_speed.set_value(ua.Variant(v, ua.VariantType.Float))
        if log_ok:
            self.get_logger().info(f'Wrote speed_set={v:.3f} to OPC UA')

    # ---------------- Polling loop ----------------

    def _poll_once(self) -> None:
        """
        Called by the ROS timer. Reads UA values and publishes status.
        IMPORTANT: Publish ONLY when connected and reads succeed.
        """
        # If not connected yet, try reconnecting and skip publishing this tick.
        if not self.connected or self.client is None:
            self.get_logger().warn_once('Not connected yet — waiting to publish.')
            self._connect_and_bind()
            return

        t0 = time.time()
        try:
            # Typed reads from UA
            z = bool(self.n_zbusy.get_value())
            c = bool(self.n_cycleok.get_value())
            s = bool(self.n_stopreq.get_value())
            v = float(self.n_speed.get_value())
            m = bool(self.n_mode.get_value())  # read manual mode

            # If we had failures previously, log that we recovered.
            if self.consec_fail > 0:
                self.get_logger().info(f'Reads recovered after {self.consec_fail} failures.')
            self.consec_fail = 0

            # Build and publish JSON status
            msg_obj = {'ts': iso_now(), 'zone_busy': int(z), 'cycle_ok': int(c), 'stop_req': int(s), 'speed_set': v, 'manual_mode': int(m)}
            self.pub_status.publish(String(data=json.dumps(msg_obj)))
            self.pub_opcua_speed.publish(Float64(data=v))
            self.pub_opcua_manual_mode.publish(Bool(data=m))
            self.pub_stop_req.publish(Bool(data=s))
            
            # CSV evidence
            self.writer.writerow([msg_obj['ts'], msg_obj['zone_busy'], msg_obj['cycle_ok'], msg_obj['stop_req'], msg_obj['speed_set'], msg_obj['manual_mode']])
            self.csv_file.flush()

        except Exception as e:
            # Count failures; after N, reconnect.
            self.consec_fail += 1
            self.get_logger().warn(f'READ ERROR {self.consec_fail}/{self.fail_limit}: {e}')
            if self.consec_fail >= self.fail_limit:
                self.get_logger().error('WATCHDOG: repeated read failures — reconnecting…')
                self._disconnect_silent()
                self._connect_and_bind()

        # Keep loop rate (best-effort; timer already enforces it, this smooths work inside callback)
        dt = time.time() - t0
        if self.poll_dt - dt > 0:
            time.sleep(self.poll_dt - dt)

    # ---------------- Tear-down ----------------

    def destroy_node(self):
        # Make sure files and sessions close cleanly
        try:
            self.csv_file.close()
        except Exception:
            pass
        self._disconnect_silent()
        super().destroy_node()

        # --- ROS → UA callbacks ---

    def _on_zone_busy(self, msg: Bool):
        """Write zone_busy from ROS into OPC UA."""
        self.last_zone_busy = bool(msg.data)
        if self.connected and self.n_zbusy is not None:
            try:
                self.n_zbusy.set_value(
                    ua.Variant(self.last_zone_busy, ua.VariantType.Boolean)
                )
            except Exception as e:
                self.get_logger().warn(f'Write zone_busy failed: {e}')

    def _on_cycle_ok(self, msg: Bool):
        """Write cycle_ok from ROS into OPC UA."""
        self.last_cycle_ok = bool(msg.data)
        if self.connected and self.n_cycleok is not None:
            try:
                self.n_cycleok.set_value(
                    ua.Variant(self.last_cycle_ok, ua.VariantType.Boolean)
                )
            except Exception as e:
                self.get_logger().warn(f'Write cycle_ok failed: {e}')

    def _on_speed_actual(self, msg: Float64):
        """Write actual speed from ROS (/speed_set) into OPC UA."""
        self.last_speed_actual = float(msg.data)
        if self.connected and self.n_speed_actual is not None:
            try:
                self.n_speed_actual.set_value(
                    ua.Variant(self.last_speed_actual, ua.VariantType.Float)
                )
            except Exception as e:
                self.get_logger().warn(f'Write speed_actual failed: {e}')



def main() -> None:
    rclpy.init()
    node = OpcUaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
