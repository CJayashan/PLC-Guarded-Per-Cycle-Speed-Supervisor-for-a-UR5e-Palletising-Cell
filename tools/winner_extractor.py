#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd

# =========================
# GLOBAL CONFIG (edit here)
# =========================

RUNS = [
    # IMPORTANT: keep the label SAME for all Bayes files
    {"label": "rule", "path": "logs/ppo_sim_final.csv"},

    # Add rule-based runs like this:
    # {"label": "rule", "path": "/full/path/to/rule1.csv"},
]

IGNORE_FIRST_N_CYCLES = 3   # per (method, file, payload_mass)

# Fairness speed-cap filter (drop only > cap, keep == cap)
ENABLE_SPEED_CAP_FILTER = True
CAP_METHOD_LABEL = "bayes"
FAIR_SPEED_CAP = 1

BOOT_ITERS = 5000
BOOT_SEED = 42

OUT_CSV = "winner_tablePPOsim_3.377simkg.csv"
OUT_TEX = "winner_tablePPOsim_3.377simkg.tex"


# =========================
# Helpers
# =========================

def bootstrap_ci_mean(x, iters=5000, alpha=0.05, seed=0):
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n == 0:
        return (np.nan, np.nan)
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, n, size=(iters, n))
    means = x[idx].mean(axis=1)
    lo = np.quantile(means, alpha / 2)
    hi = np.quantile(means, 1 - alpha / 2)
    return float(lo), float(hi)


def summarize_one_group(df: pd.DataFrame) -> dict:
    dur = df["duration"].astype(float).to_numpy()
    n = len(dur)

    mean = float(np.mean(dur)) if n else np.nan
    std = float(np.std(dur, ddof=1)) if n > 1 else (0.0 if n == 1 else np.nan)
    ci_lo, ci_hi = bootstrap_ci_mean(dur, iters=BOOT_ITERS, seed=BOOT_SEED)

    stop_rate = float(np.mean(df["had_stop_req"].astype(int))) if n else np.nan
    success_rate = float(np.mean(df["success"].astype(int))) if n else np.nan

    med = float(np.median(dur)) if n else np.nan
    p95 = float(np.quantile(dur, 0.95)) if n else np.nan

    vel = df["max_velocity_ratio"].astype(float).to_numpy()
    tau = df["max_torque_ratio"].astype(float).to_numpy()

    vel_p95 = float(np.quantile(vel, 0.95)) if n else np.nan
    vel_max = float(np.max(vel)) if n else np.nan
    tau_p95 = float(np.quantile(tau, 0.95)) if n else np.nan
    tau_max = float(np.max(tau)) if n else np.nan

    return {
        "n_cycles": n,
        "mean_duration_s": mean,
        "std_duration_s": std,
        "mean_95ci_low_s": ci_lo,
        "mean_95ci_high_s": ci_hi,
        "median_duration_s": med,
        "p95_duration_s": p95,
        "stop_rate": stop_rate,
        "success_rate": success_rate,
        "vel_ratio_p95": vel_p95,
        "vel_ratio_max": vel_max,
        "torque_ratio_p95": tau_p95,
        "torque_ratio_max": tau_max,
    }


def latex_escape(s: str) -> str:
    # minimal escape for LaTeX special chars
    return (s.replace("\\", "\\textbackslash{}")
             .replace("_", "\\_")
             .replace("%", "\\%")
             .replace("&", "\\&")
             .replace("#", "\\#"))


def write_simple_latex_table(df: pd.DataFrame, path: str):
    cols = list(df.columns)
    col_spec = "l" * len(cols)
    lines = []
    lines.append("\\begin{tabular}{" + col_spec + "}")
    lines.append("\\hline")
    lines.append(" & ".join(latex_escape(c) for c in cols) + " \\\\")
    lines.append("\\hline")
    for _, row in df.iterrows():
        vals = []
        for c in cols:
            v = row[c]
            if isinstance(v, float):
                vals.append(f"{v:.3f}")
            else:
                vals.append(latex_escape(str(v)))
        lines.append(" & ".join(vals) + " \\\\")
    lines.append("\\hline")
    lines.append("\\end{tabular}")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main():
    # Load
    dfs = []
    for r in RUNS:
        p = r["path"]
        label = r["label"]
        df = pd.read_csv(p)
        df["method"] = label
        df["source_path"] = p
        df["source_file"] = os.path.basename(p)
        dfs.append(df)

    if not dfs:
        raise SystemExit("RUNS is empty. Add at least one file.")

    data = pd.concat(dfs, ignore_index=True)

    # Fairness speed cap: drop only speed > cap
    if ENABLE_SPEED_CAP_FILTER:
        before = len(data)
        mask_drop = (data["method"] == CAP_METHOD_LABEL) & (data["speed"].astype(float) > FAIR_SPEED_CAP)
        dropped = int(mask_drop.sum())
        data = data[~mask_drop].copy()
        after = len(data)
        print(f"[INFO] Speed-cap filter: dropped {dropped} rows where {CAP_METHOD_LABEL}.speed > {FAIR_SPEED_CAP} "
              f"(kept <= {FAIR_SPEED_CAP}). Rows: {before} -> {after}")

    # Ignore first N cycles per (method, file, payload_mass) WITHOUT groupby.apply
    if IGNORE_FIRST_N_CYCLES > 0:
        data = data.sort_values(["method", "source_path", "payload_mass", "cycle_index"]).copy()
        data["_rank"] = data.groupby(["method", "source_path", "payload_mass"]).cumcount()
        data = data[data["_rank"] >= IGNORE_FIRST_N_CYCLES].drop(columns=["_rank"])

    # Build winner table by iterating groups (no apply warning)
    rows = []
    for (method, mass), g in data.groupby(["method", "payload_mass"], sort=True):
        stats = summarize_one_group(g)
        stats["method"] = method
        stats["payload_mass"] = mass
        rows.append(stats)

    table = pd.DataFrame(rows)

    # Friendly columns
    table["mean±ci (s)"] = table.apply(
        lambda r: f'{r["mean_duration_s"]:.3f} [{r["mean_95ci_low_s"]:.3f}, {r["mean_95ci_high_s"]:.3f}]'
        if pd.notna(r["mean_duration_s"]) else "",
        axis=1,
    )
    table["stop_rate_%"] = (table["stop_rate"] * 100).round(1)
    table["success_rate_%"] = (table["success_rate"] * 100).round(1)

    display = table[[
        "method", "payload_mass", "n_cycles",
        "mean±ci (s)", "median_duration_s", "p95_duration_s",
        "stop_rate_%", "success_rate_%",
        "vel_ratio_p95", "vel_ratio_max",
        "torque_ratio_p95", "torque_ratio_max",
    ]].copy()

    # Save CSV
    display.to_csv(OUT_CSV, index=False)

    # Save LaTeX (try pandas first; fallback to simple tabular)
    try:
        display.to_latex(OUT_TEX, index=False, escape=True)
    except Exception as e:
        print(f"[WARN] Pandas LaTeX export failed ({e}). Writing simple LaTeX tabular instead.")
        write_simple_latex_table(display, OUT_TEX)

    print(f"\n[INFO] Ignore first N cycles per run & mass: {IGNORE_FIRST_N_CYCLES}")
    print(f"[INFO] Saved: {OUT_CSV}")
    print(f"[INFO] Saved: {OUT_TEX}")

    # Useful pivot if you have multiple methods (bayes vs rule)
    if display["method"].nunique() > 1:
        pivot = table.pivot_table(index="payload_mass", columns="method", values="mean_duration_s", aggfunc="first")
        print("\nMean duration (s) by payload_mass:")
        print(pivot.round(3))


if __name__ == "__main__":
    main()
