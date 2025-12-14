#! /usr/bin/env python3

import time
from opcua import ua, Server
import logging

def main():
    
    for name in [
        "opcua",
        "opcua.server",
        "opcua.server.address_space",
        "opcua.ua",  
    ]:
        logging.getLogger(name).setLevel(logging.ERROR)
    
    srv = Server()

    # listen to all interfaces on default OPCUA port 4840(localhost, LAN, wifi, etc)
    srv.set_endpoint("opc.tcp://0.0.0.0:4840") 

    #custom workspace to avoid clashing with other nodes
    uri = "urn:mockplc:chathushka_withanage:opcua"
    ns_idx = srv.register_namespace(uri)                    # returns small integer

    """
    The server's address space is organized as a tree, starting from a single Root node.
    Standard nodes are organized under the 'Objects' folder where we usually mess.
    We will create a custom object 'MockPLC' under 'Objects' to hold our variables.

    Root
 ├── Objects
 │    └── MockPLC
 │         ├── SpeedSet     (VariableNode)
 │         ├── ZoneBusy     (VariableNode)
 │         ├── CycleOK      (VariableNode)
 │         └── Reset()      (MethodNode)
 ├── Types
 │    ├── ObjectTypes
 │    │     ├── BaseObjectType
 │    │     └── MyCustomPLCType
 │    ├── VariableTypes
 │    │     ├── BaseVariableType
 │    │     └── AnalogItemType
 │    └── DataTypes
 │          ├── Double
 │          └── Boolean
 └── Views

    """

    objects = srv.get_objects_node()                   # gets the handle to the 'Objects' folder inside the tree
    object_mockplc = objects.add_object(ns_idx, "MockPLC")        # add a custom object 'MockPLC' under 'Objects'

    # Add variables (tags) with correct DataTypes and access.

    speed_set = object_mockplc.add_variable(
        ua.NodeId("speed_set", ns_idx),              # NodeId: string identifier "speed_set" in our namespace
        "speed_set",                                 # Display/Browse name
        ua.Variant(0.5, ua.VariantType.Float)        # Value is wrapped in a Variant with type Float
    )
    speed_set.set_writable(True)                     # Allow clients to write this variable

    # NEW: Speed actually used by robot (Float) – ROS writes this
    speed_actual = object_mockplc.add_variable(
        ua.NodeId("speed_actual", ns_idx),
        "speed_actual",
        ua.Variant(0.0, ua.VariantType.Float)
    )
    speed_actual.set_writable(True)

    # Manual / Auto selector - Bool - Operator can write
    speed_mode_manual = object_mockplc.add_variable(
        ua.NodeId("speed_mode_manual", ns_idx),
        "speed_mode_manual",
        False,
        ua.VariantType.Boolean
    )
    speed_mode_manual.set_writable(True)             # Operator can write this variable

    # Status Bits written by ROS
    zone_busy = object_mockplc.add_variable(
        ua.NodeId("zone_busy", ns_idx),
        "zone_busy",
        False,
        ua.VariantType.Boolean
    )
    zone_busy.set_writable(True)                    # Read-only for clients
    
    cycle_ok = object_mockplc.add_variable(
        ua.NodeId("cycle_ok", ns_idx),
        "cycle_ok",
        False,
        ua.VariantType.Boolean
    )
    cycle_ok.set_writable(True)                     # Read-only for clients

    stop_req = object_mockplc.add_variable(
        ua.NodeId("stop_req", ns_idx),
        "stop_req",
        False,
        ua.VariantType.Boolean
    )
    stop_req.set_writable(True)                     # Read-only for clients

    # Add common variable attributes

    def set_defaults(v_node):
        v_node.set_attribute(ua.AttributeIds.ValueRank, 
                             ua.DataValue(ua.Variant(-1, ua.VariantType.Int32)))  # says the variable is a scalar
        v_node.set_attribute(ua.AttributeIds.ArrayDimensions,
                             ua.DataValue(ua.Variant([], ua.VariantType.UInt32)))  # no array dimensions
        v_node.set_attribute(ua.AttributeIds.MinimumSamplingInterval,
                             ua.DataValue(ua.Variant(0.0, ua.VariantType.Double)))  # no restriction. So, clients may poll as fast as they want
        v_node.set_attribute(ua.AttributeIds.Historizing,
                             ua.DataValue(ua.Variant(False, ua.VariantType.Boolean)))  # No history logging for the varible node
        
    for var_node in [speed_set, speed_actual, speed_mode_manual, zone_busy, cycle_ok, stop_req]:
        set_defaults(var_node)
    

    # Start the server

    srv.start()
    print("OPC UA server started at: opc.tcp://localhost:4840")
    print(f"   Namespace URI: {uri}")
    print(f"   Namespace Index (ns): {ns_idx}")
    print("   NodeIds:")
    print(f"     speed_set → ns={ns_idx};s=speed_set  [Float, writable]")
    print(f"     speed_actual → ns={ns_idx};s=speed_actual  [Float, writable]")
    print(f"     speed_mode_manual → ns={ns_idx};s=speed_mode_manual  [Boolean, writable]")
    print(f"     zone_busy → ns={ns_idx};s=zone_busy  [Boolean, writable]")
    print(f"     cycle_ok  → ns={ns_idx};s=cycle_ok   [Boolean, writable]")
    print(f"     stop_req  → ns={ns_idx};s=stop_req   [Boolean, writable]")

    try:
        while True:
            time.sleep(5)  # make the server in idle
    finally:
        srv.stop()
        print("OPC UA server stopped.")

if __name__ == "__main__":
    main()


