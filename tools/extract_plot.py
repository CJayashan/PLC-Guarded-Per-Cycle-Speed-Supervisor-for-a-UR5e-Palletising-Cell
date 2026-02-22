#!/usr/bin/env python3
"""
Plot two key Bayes physical-run graphs from cycle_logger CSVs:
1) duration vs cycle_index
2) speed vs cycle_index

Edit the global variables CSV_DIR and CSV_GLOB to point to your logs.
Then run:
  python3 plot_bayes_physical.py
"""

import glob
import os
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt


# =========================
# EDIT THESE TWO LINES ONLY
# =========================
CSV_DIR  = "/home/withanage/projects/project_root/logs"   # <-- change this
CSV_GLOB = "ppo_sim_final.csv"                                  # <-- change this
# =========================


REQUIRED_COLS = ["run_id", "cycle_index", "payload_mass", "speed", "duration"]


def load_csvs(csv_paths):
    frames = []
    for p in csv_paths:
        df = pd.read_csv(p)
        missing = [c for c in REQUIRED_COLS if c not in df.columns]
        if missing:
            raise ValueError(f"{p} missing columns: {missing}. Found: {list(df.columns)}")

        # ensure numeric
        df["cycle_index"] = pd.to_numeric(df["cycle_index"], errors="coerce")
        df["speed"] = pd.to_numeric(df["speed"], errors="coerce")
        df["duration"] = pd.to_numeric(df["duration"], errors="coerce")
        df["payload_mass"] = pd.to_numeric(df["payload_mass"], errors="coerce")

        df = df.dropna(subset=["cycle_index", "speed", "duration", "payload_mass"])
        frames.append(df)

    if not frames:
        raise ValueError("No CSV rows loaded.")
    return pd.concat(frames, ignore_index=True)


def make_outdir():
    outdir = Path("out_plots")
    outdir.mkdir(exist_ok=True)
    return outdir


def plot_duration_vs_index(df, outdir: Path):
    plt.figure()
    for run_id, g in df.groupby("run_id"):
        g2 = g.sort_values("cycle_index")

        x = g2["cycle_index"].to_numpy()
        y = g2["duration"].to_numpy()

        plt.plot(x, y, marker="o", linewidth=1, label=str(run_id))

    plt.xlabel("Cycle index")
    plt.ylabel("Duration (s)")
    plt.title("PPO supervisor (sim): duration vs cycle index")
    plt.legend(fontsize=8)
    plt.tight_layout()
    outpath = outdir / "ppo_3.377simkg_duration_vs_cycle_index.png"
    plt.savefig(outpath, dpi=200)
    plt.close()
    return outpath


def plot_speed_vs_index(df, outdir: Path):
    plt.figure()
    for run_id, g in df.groupby("run_id"):
        g2 = g.sort_values("cycle_index")

        x = g2["cycle_index"].to_numpy()
        y = g2["speed"].to_numpy()

        plt.step(x, y, where="post", linewidth=1.5, label=str(run_id))

    plt.xlabel("Cycle index")
    plt.ylabel("Speed setpoint s")
    plt.title("PPO supervisor (sim): chosen speed vs cycle index")
    plt.legend(fontsize=8)
    plt.tight_layout()
    outpath = outdir / "ppo_3.377simkg_speed_vs_cycle_index.png"
    plt.savefig(outpath, dpi=200)
    plt.close()
    return outpath


def export_min_table(df, outdir: Path):
    cols = ["run_id", "cycle_index", "payload_mass", "speed", "duration",
            "success", "had_stop_req", "max_velocity_ratio", "max_torque_ratio"]
    cols = [c for c in cols if c in df.columns]
    mini = df[cols].sort_values(["run_id", "cycle_index"])
    outpath = outdir / "bayes_extracted_minimal.csv"
    mini.to_csv(outpath, index=False)
    return outpath


def main():
    pattern = os.path.join(CSV_DIR, CSV_GLOB)
    csv_paths = sorted(glob.glob(pattern))

    if not csv_paths:
        raise SystemExit(f"No CSV files found.\nChecked: {pattern}")

    df = load_csvs(csv_paths)
    outdir = make_outdir()

    p1 = plot_duration_vs_index(df, outdir)
    p2 = plot_speed_vs_index(df, outdir)
    p3 = export_min_table(df, outdir)

    print("Loaded files:")
    for p in csv_paths:
        print(" ", p)

    print("\nSaved plots:")
    print(" ", p1)
    print(" ", p2)
    print("Saved extracted table:")
    print(" ", p3)


if __name__ == "__main__":
    main()
