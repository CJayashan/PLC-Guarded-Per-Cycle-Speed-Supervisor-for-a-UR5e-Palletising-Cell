#!/usr/bin/env python3
"""
OPC UA Mock PLC Server (4-tag model)

Endpoint:
    opc.tcp://0.0.0.0:4840
Address Space (browse tree):
    Objects
      └── MockPLC (Object)
            ├── speed_set  (Float,  writable by clients)
            ├── zone_busy  (Boolean, read-only)
            ├── cycle_ok   (Boolean, read-only)
            └── stop_req   (Boolean, read-only)

Why:
- Matches our "one dial + three status" contract.
- Types and access match the story we'll use in the bridge and twin.
"""

import time                    # for simple sleeping/heartbeat
from opcua import ua, Server   # ua = types/enums; Server = OPC UA server class


def main():
    # (1) Create the server object and define where it will listen for clients.
    server = Server()
    server.set_endpoint("opc.tcp://0.0.0.0:4840")
    # "0.0.0.0" binds on all interfaces. Clients will connect to "opc.tcp://localhost:4840" on this PC.

    # (2) Register a custom namespace to avoid NodeId clashes with standard nodes.
    uri = "urn:mockplc:chathushka_withanage:opcua"  # a unique-ish URI string for our namespace
    ns_idx = server.register_namespace(uri)
    # ns_idx is a small integer (often 2 or 3). Clients must use THIS exact ns index when addressing nodes.

    # (3) Get the standard 'Objects' folder (a built-in root for placing our own objects/variables).
    objects = server.get_objects_node()

    # (4) Create a logical device/object to group our variables (like a PLC device folder).
    mockplc_obj = objects.add_object(ns_idx, "MockPLC")
    # Browsename = "MockPLC"; Namespace = our custom ns. It will appear under Objects in UAExpert.

    # (5) Add variables (tags) with correct DataTypes and access.
    # 5a) speed_set: Float, writable (this is our single control knob)
    speed_set_var = mockplc_obj.add_variable(
        ua.NodeId("speed_set", ns_idx),              # NodeId: string identifier "speed_set" in our namespace
        "speed_set",                                 # Display/Browse name
        ua.Variant(0.5, ua.VariantType.Float)        # Initial value 0.5, typed as Float
    )
    speed_set_var.set_writable(True)                 # Allow clients to write this variable

    # 5b) zone_busy: Boolean, read-only (we omit set_writable → remains read-only to clients)
    zone_busy_var = mockplc_obj.add_variable(
        ua.NodeId("zone_busy", ns_idx),
        "zone_busy",
        ua.Variant(False, ua.VariantType.Boolean)    # Initial value False, typed as Boolean
    )

    # 5c) cycle_ok: Boolean, read-only
    cycle_ok_var = mockplc_obj.add_variable(
        ua.NodeId("cycle_ok", ns_idx),
        "cycle_ok",
        ua.Variant(False, ua.VariantType.Boolean)
    )

    # 5d) stop_req: Boolean, read-only
    stop_req_var = mockplc_obj.add_variable(
        ua.NodeId("stop_req", ns_idx),
        "stop_req",
        ua.Variant(False, ua.VariantType.Boolean)
    )

    # (6) Start listening for clients. From now on, UAExpert/clients can connect and browse/write.
    server.start()
    print("OPC UA server started at: opc.tcp://localhost:4840")
    print(f"   Namespace URI: {uri}")
    print(f"   Namespace Index (ns): {ns_idx}")
    print("   NodeIds you will use:")
    print(f"     speed_set → ns={ns_idx};s=speed_set  [Float, writable]")
    print(f"     zone_busy → ns={ns_idx};s=zone_busy  [Boolean, read-only]")
    print(f"     cycle_ok  → ns={ns_idx};s=cycle_ok   [Boolean, read-only]")
    print(f"     stop_req  → ns={ns_idx};s=stop_req   [Boolean, read-only]")

    try:
        # (7) Keep the server alive. Put any mock behavior here later if you want.
        while True:
            time.sleep(5)  # heartbeat every ~5s (no prints to keep terminal clean)
    finally:
        # (8) Clean shutdown if you Ctrl+C or the process exits.
        server.stop()
        print("OPC UA server stopped.")


if __name__ == "__main__":
    main()
