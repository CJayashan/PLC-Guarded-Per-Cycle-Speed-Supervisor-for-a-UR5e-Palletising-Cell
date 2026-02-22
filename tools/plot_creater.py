#!/usr/bin/env python3
"""
Make 2 plots per CSV (saved as JPG):
  1) Safety ratios vs cycle index (velocity + torque on same plot, togglable)
  2) Speed vs duration scatter

Fix: convert pandas Series to numpy arrays before plotting to avoid
"Multi-dimensional indexing is no longer supported" error.
"""

import os
import pandas as pd
import matplotlib.pyplot as plt


# =========================
# USER SETTINGS (EDIT HERE)
# =========================

CSV_FILES = [
    "/home/withanage/projects/project_root/logs/bayes_0.377simkg.csv",
    "/home/withanage/projects/project_root/logs/bayes0.377.csv",
    # "/full/path/to/rule1.csv",
]

OUT_DIR = "./plots"
IGNORE_FIRST_N_CYCLES = 0  # ignore first N rows after sorting by cycle_index

# Safety ratio plot toggles
PLOT_VEL_RATIO = True
PLOT_TORQUE_RATIO = True   # set False for sim if torque ratios are meaningless/zero

# Optional threshold lines (set None to disable)
VEL_RATIO_LIMIT = 0.55    # e.g. 0.55
TORQUE_RATIO_LIMIT = 0.35  # e.g. 0.35

# JPG quality
DPI = 200
FIGSIZE = (10, 5)


# =========================
# INTERNALS
# =========================

REQ_COLS = ["cycle_index", "duration", "speed", "max_velocity_ratio", "max_torque_ratio"]


def ensure_out_dir(path: str):
    os.makedirs(path, exist_ok=True)


def safe_stem(path: str) -> str:
    base = os.path.basename(path)
    stem, _ = os.path.splitext(base)
    return stem


def load_and_prepare(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)

    missing = [c for c in REQ_COLS if c not in df.columns]
    if missing:
        raise ValueError(f"{csv_path} is missing columns: {missing}")

    # keep required + a few extras if present
    keep = [c for c in REQ_COLS]
    extra = [c for c in ["payload_mass", "run_id", "method"] if c in df.columns]
    df = df[keep + extra].copy()

    # numeric coercion
    for c in REQ_COLS:
        df[c] = pd.to_numeric(df[c], errors="coerce")

    # sort and optionally ignore warmup cycles
    df = df.sort_values("cycle_index").reset_index(drop=True)
    if IGNORE_FIRST_N_CYCLES > 0 and len(df) > IGNORE_FIRST_N_CYCLES:
        df = df.iloc[IGNORE_FIRST_N_CYCLES:].reset_index(drop=True)

    return df


def plot_safety_ratios(df: pd.DataFrame, title: str, out_path: str):
    # Drop rows missing the stuff we will plot
    cols_needed = ["cycle_index"]
    if PLOT_VEL_RATIO:
        cols_needed.append("max_velocity_ratio")
    if PLOT_TORQUE_RATIO:
        cols_needed.append("max_torque_ratio")

    d = df.dropna(subset=cols_needed).copy()
    if d.empty:
        print(f"[WARN] No data after NaN drop for safety plot: {out_path}")
        return

    # IMPORTANT: convert to numpy arrays (fixes your error)
    x = d["cycle_index"].to_numpy()

    plt.figure(figsize=FIGSIZE)
    plotted_any = False

    if PLOT_VEL_RATIO:
        yv = d["max_velocity_ratio"].to_numpy()
        plt.plot(x, yv, label="max_velocity_ratio")
        plotted_any = True
        if VEL_RATIO_LIMIT is not None:
            plt.axhline(VEL_RATIO_LIMIT, linestyle="--", linewidth=1, label=f"vel_limit={VEL_RATIO_LIMIT}")

    if PLOT_TORQUE_RATIO:
        yt = d["max_torque_ratio"].to_numpy()
        plt.plot(x, yt, label="max_torque_ratio")
        plotted_any = True
        if TORQUE_RATIO_LIMIT is not None:
            plt.axhline(TORQUE_RATIO_LIMIT, linestyle="--", linewidth=1, label=f"torque_limit={TORQUE_RATIO_LIMIT}")

    if not plotted_any:
        plt.close()
        print(f"[WARN] Safety ratios plot skipped (both toggles off): {out_path}")
        return

    plt.xlabel("cycle_index")
    plt.ylabel("ratio")
    plt.title(title)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=DPI)
    plt.close()


def plot_speed_vs_duration(df: pd.DataFrame, title: str, out_path: str):
    d = df.dropna(subset=["speed", "duration"]).copy()
    if d.empty:
        print(f"[WARN] No data after NaN drop for scatter plot: {out_path}")
        return

    # numpy arrays not strictly required for scatter, but keeps it consistent
    xs = d["speed"].to_numpy()
    ys = d["duration"].to_numpy()

    plt.figure(figsize=FIGSIZE)
    plt.scatter(xs, ys)
    plt.xlabel("speed")
    plt.ylabel("duration (s)")
    plt.title(title)
    plt.tight_layout()
    plt.savefig(out_path, dpi=DPI)
    plt.close()


def main():
    ensure_out_dir(OUT_DIR)

    for csv_path in CSV_FILES:
        stem = safe_stem(csv_path)
        df = load_and_prepare(csv_path)

        mass_str = ""
        if "payload_mass" in df.columns:
            unique_m = df["payload_mass"].dropna().unique()
            if len(unique_m) == 1:
                mass_str = f" | mass={unique_m[0]:g}kg"
            elif len(unique_m) > 1:
                mass_str = " | mass=mixed"

        safety_out = os.path.join(OUT_DIR, f"{stem}_safety_ratios.jpg")
        plot_safety_ratios(df, f"Safety ratios vs cycle index: {stem}{mass_str}", safety_out)

        scatter_out = os.path.join(OUT_DIR, f"{stem}_speed_vs_duration.jpg")
        plot_speed_vs_duration(df, f"Speed vs duration: {stem}{mass_str}", scatter_out)

        print(f"[OK] {csv_path}")
        print(f"     -> {safety_out}")
        print(f"     -> {scatter_out}")


if __name__ == "__main__":
    main()
