import csv
import math
import sys
from pathlib import Path

# Durations shorter than this (in seconds) are ignored (set to 0.0 to disable)
MIN_DURATION = 0.0

def analyze_csv(csv_path: Path):
    # Read all rows first so we can skip the first one (homing)
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print(f"No data rows found in {csv_path}")
        return

    # Skip the very first motion (homing move)
    data_rows = rows[1:]

    durations = []
    successes = []
    stops = []

    for row in data_rows:
        try:
            dur = float(row["duration"])
        except (KeyError, ValueError):
            # Bad row, skip
            continue

        # Optional filter (currently disabled by MIN_DURATION = 0.0)
        if dur < MIN_DURATION:
            continue

        durations.append(dur)

        # success / had_stop_req are 0 or 1
        try:
            successes.append(int(row.get("success", 0)))
        except ValueError:
            successes.append(0)

        try:
            stops.append(int(row.get("had_stop_req", 0)))
        except ValueError:
            stops.append(0)

    if not durations:
        print(
            f"No valid rows after skipping first entry "
            f"and applying MIN_DURATION={MIN_DURATION}s in {csv_path}"
        )
        return

    n = len(durations)
    mean = sum(durations) / n

    # Sample standard deviation
    if n > 1:
        var = sum((d - mean) ** 2 for d in durations) / (n - 1)
        std = math.sqrt(var)
    else:
        std = 0.0

    # 95% confidence interval for the mean (normal approximation)
    if n > 1 and std > 0.0:
        se = std / math.sqrt(n)
        ci_low = mean - 1.96 * se
        ci_high = mean + 1.96 * se
    else:
        ci_low = ci_high = mean

    success_rate = sum(successes) / n
    stop_rate = sum(stops) / n

    print(f"=== Analysis for {csv_path.name} ===")
    print(f"Skipped first entry (homing move).")
    print(f"Used {n} motions (duration >= {MIN_DURATION:.1f}s)")
    print(f"Mean duration:      {mean:.3f} s")
    print(f"Std deviation:      {std:.3f} s")
    print(f"95% CI for mean:    [{ci_low:.3f}, {ci_high:.3f}] s")
    print(f"Success rate:       {success_rate * 100:.1f} %")
    print(f"Stop_req rate:      {stop_rate * 100:.1f} %")
    print()


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 offline_analysis.py <cycle_log.csv> [<cycle_log2.csv> ...]")
        sys.exit(1)

    for arg in sys.argv[1:]:
        csv_path = Path(arg)
        if not csv_path.exists():
            print(f"File not found: {csv_path}")
            continue
        analyze_csv(csv_path)


if __name__ == "__main__":
    main()

