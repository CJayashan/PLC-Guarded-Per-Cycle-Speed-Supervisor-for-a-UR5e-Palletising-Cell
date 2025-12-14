#!/usr/bin/env python3

"""
Small helper script.

Reads a cycle log CSV and prints:
- all cycle_index values (space-separated)
- all speed values (space-separated)
- all duration values (space-separated)

Usage:
  python3 extract_for_plot.py path/to/cycle_log.csv              # all rows
  python3 extract_for_plot.py path/to/cycle_log.csv testBayes    # only this run_id
"""

import csv
import sys

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 extract_for_plot.py <csv_path> [run_id_filter]")
        sys.exit(1)

    csv_path = sys.argv[1]
    run_id_filter = sys.argv[2] if len(sys.argv) >= 3 else None

    indices = []
    speeds = []
    durations = []

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # If a run_id filter is given, skip rows that don't match
            if run_id_filter is not None and row.get("run_id") != run_id_filter:
                continue

            # Take values as strings, they are already fine for printing
            indices.append(row["cycle_index"])
            speeds.append(row["speed"])
            durations.append(row["duration"])

    # Build the 3 space-separated strings
    idx_str = " ".join(indices)
    speed_str = " ".join(speeds)
    dur_str = " ".join(durations)

    # Print them in a copy-paste friendly way
    print("cycle_index:")
    print(idx_str)
    print()
    print("speed:")
    print(speed_str)
    print()
    print("duration:")
    print(dur_str)

if __name__ == "__main__":
    main()
