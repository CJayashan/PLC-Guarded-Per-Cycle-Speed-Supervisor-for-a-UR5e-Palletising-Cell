#!/usr/bin/env python3
"""
Standalone OPC UA polling client with auto-reconnect + CSV + watchdog.

Behavior:
- Polls ~5 Hz: reads zone_busy, cycle_ok, stop_req, speed_set.
- Logs CSV each poll.
- On repeated read errors: prints WARNING, disconnects, and retries connect
  until success, then prints "reconnected" and continues.
"""

import csv
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Tuple

from opcua import Client, ua

# ---------- CONFIG ----------
ENDPOINT = "opc.tcp://localhost:4840"
NS_IDX = 2   # <-- set to the ns index shown by your server/UAExpert

NODEID_SPEED   = ua.NodeId("speed_set", NS_IDX)
NODEID_ZBUSY   = ua.NodeId("zone_busy", NS_IDX)
NODEID_CYCLEOK = ua.NodeId("cycle_ok", NS_IDX)
NODEID_STOPREQ = ua.NodeId("stop_req", NS_IDX)

POLL_HZ = 5.0
POLL_DT = 1.0 / POLL_HZ

CSV_PATH = Path("../logs/opcua_poll.csv")

WATCHDOG_FAIL_LIMIT = 5         # after N consecutive read errors → reconnect
RECONNECT_BACKOFF_S = 2.0       # wait between reconnect attempts
# ----------------------------


def iso_now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def connect_and_bind() -> Tuple[Client, object, object, object, object]:
    """
    Connects a fresh Client() to ENDPOINT and returns:
      (client, n_speed, n_zbusy, n_cycleok, n_stopreq)
    """
    cli = Client(ENDPOINT)
    # (optional) shorter socket timeout helps the loop fail fast when the server is down
    try:
        cli.set_timeout(3000)  # ms; if not available in your version, it's safe to ignore
    except Exception:
        pass

    cli.connect()  # establishes a Session
    # Resolve node handles once
    n_speed   = cli.get_node(NODEID_SPEED)
    n_zbusy   = cli.get_node(NODEID_ZBUSY)
    n_cycleok = cli.get_node(NODEID_CYCLEOK)
    n_stopreq = cli.get_node(NODEID_STOPREQ)
    return cli, n_speed, n_zbusy, n_cycleok, n_stopreq


def main():
    # CSV setup
    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    new_file = not CSV_PATH.exists() or CSV_PATH.stat().st_size == 0
    csv_file = CSV_PATH.open("a", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    if new_file:
        writer.writerow(["ts", "zone_busy", "cycle_ok", "stop_req", "speed_set"])

    # Initial connect
    print(f"[{iso_now()}] Connecting to {ENDPOINT} ...")
    client, n_speed, n_zbusy, n_cycleok, n_stopreq = connect_and_bind()
    print(f"[{iso_now()}] Connected. Starting poll loop… (Ctrl+C to stop)")

    consec_fail = 0

    try:
        while True:
            t0 = time.time()
            try:
                z = bool(n_zbusy.get_value())
                c = bool(n_cycleok.get_value())
                s = bool(n_stopreq.get_value())
                v = float(n_speed.get_value())

                if consec_fail > 0:
                    print(f"[{iso_now()}] INFO: reads recovered after {consec_fail} failures.")
                consec_fail = 0

                writer.writerow([iso_now(), int(z), int(c), int(s), v])
                csv_file.flush()

            except Exception as e:
                consec_fail += 1
                print(f"[{iso_now()}] READ ERROR {consec_fail}/{WATCHDOG_FAIL_LIMIT}: {e}", file=sys.stderr)

                if consec_fail >= WATCHDOG_FAIL_LIMIT:
                    # watchdog event → do a controlled reconnect
                    print(f"[{iso_now()}] WATCHDOG WARNING: repeated read failures — attempting reconnect…",
                          file=sys.stderr)
                    # Try to tear down any half-dead session
                    try:
                        client.disconnect()
                    except Exception:
                        pass

                    # Keep trying until we reconnect
                    while True:
                        try:
                            time.sleep(RECONNECT_BACKOFF_S)
                            client, n_speed, n_zbusy, n_cycleok, n_stopreq = connect_and_bind()
                            print(f"[{iso_now()}] Reconnected successfully.")
                            consec_fail = 0
                            break
                        except Exception as ee:
                            print(f"[{iso_now()}] Reconnect failed: {ee}  (retrying in {RECONNECT_BACKOFF_S}s)",
                                  file=sys.stderr)

            # Keep loop rate
            dt = time.time() - t0
            time.sleep(max(0.0, POLL_DT - dt))

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        try:
            client.disconnect()
        except Exception:
            pass
        csv_file.close()
        print("Client stopped. CSV closed.")


if __name__ == "__main__":
    main()
