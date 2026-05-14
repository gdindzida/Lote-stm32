"""
analyze_timing.py
-----------------
Diagnostic script to analyse:
  1. Nav-cam timestamp frequency
  2. Mocap sensor timestamp frequency
  3. Last-before matching between the two (latest sensor frame <= t_cam), with time-difference stats

Usage:
    python analyze_timing.py <data_root> <sensors_root>

    data_root    – folder containing nav_cam_timestamps.csv
    sensors_root – folder containing mocap_vehicle_data.csv
"""

import sys
import os
import csv
import numpy as np

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _last_before(arr: np.ndarray, val: float) -> int:
    """Return the index of the latest entry in *arr* that is <= *val*.

    *arr* must be sorted ascending.
    Returns -1 if all entries are strictly after *val* (no valid match).
    """
    idx = int(np.searchsorted(arr, val, side="right")) - 1
    return idx  # -1 when val < arr[0]


def load_cam_timestamps(data_root: str) -> np.ndarray:
    cam_csv = os.path.join(data_root, "nav_cam_timestamps.csv")
    if not os.path.isfile(cam_csv):
        raise FileNotFoundError(f"nav_cam_timestamps.csv not found in: {data_root}")

    times = []
    with open(cam_csv, newline="") as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 2:
                continue
            try:
                times.append(float(parts[1]))
            except ValueError:
                continue

    return np.array(times, dtype=np.float64)


def load_sensor_timestamps(sensors_root: str) -> np.ndarray:
    mocap_csv = os.path.join(sensors_root, "mocap_vehicle_data.csv")
    if not os.path.isfile(mocap_csv):
        raise FileNotFoundError(f"mocap_vehicle_data.csv not found in: {sensors_root}")

    times = []
    with open(mocap_csv, newline="") as fh:
        reader = csv.reader(fh)
        header = [h.strip() for h in next(reader)]
        idx_t = header.index("t")
        for row in reader:
            if not row:
                continue
            try:
                times.append(float(row[idx_t]))
            except (ValueError, IndexError):
                continue

    return np.array(times, dtype=np.float64)


# ---------------------------------------------------------------------------
# Analysis helpers
# ---------------------------------------------------------------------------


def frequency_stats(timestamps: np.ndarray, label: str) -> None:
    """Print frequency statistics derived from *timestamps*."""
    if len(timestamps) < 2:
        print(f"[{label}] Not enough timestamps to compute frequency.")
        return

    diffs = np.diff(timestamps)  # inter-frame intervals (seconds)
    freqs = 1.0 / diffs  # instantaneous frequencies (Hz)

    print(f"\n{'=' * 60}")
    print(f"  {label}")
    print(f"{'=' * 60}")
    print(f"  Total frames        : {len(timestamps)}")
    print(f"  Time span           : {timestamps[0]:.6f} → {timestamps[-1]:.6f} s")
    print(f"  Total duration      : {timestamps[-1] - timestamps[0]:.4f} s")
    print(f"\n  --- Inter-frame interval (s) ---")
    print(f"  Mean                : {diffs.mean():.6f}")
    print(f"  Std                 : {diffs.std():.6f}")
    print(f"  Min                 : {diffs.min():.6f}")
    print(f"  Max                 : {diffs.max():.6f}")
    print(f"\n  --- Frequency (Hz) ---")
    print(f"  Mean                : {freqs.mean():.4f}")
    print(f"  Std                 : {freqs.std():.4f}")
    print(f"  Min                 : {freqs.min():.4f}")
    print(f"  Max                 : {freqs.max():.4f}")
    print(f"  Median              : {np.median(freqs):.4f}")

    # Flag outlier gaps (> 3× median interval)
    median_dt = np.median(diffs)
    outlier_mask = diffs > 3 * median_dt
    n_outliers = outlier_mask.sum()
    if n_outliers:
        print(f"\n  ⚠  Outlier gaps (>3× median interval): {n_outliers}")
        for i in np.where(outlier_mask)[0][:10]:  # show first 10
            print(f"     frame {i}→{i+1}  Δt={diffs[i]:.6f} s")
        if n_outliers > 10:
            print(f"     … and {n_outliers - 10} more.")
    else:
        print(f"\n  ✓  No outlier gaps detected.")


def nearest_neighbour_analysis(t_cam: np.ndarray, t_sensor: np.ndarray) -> None:
    """For every camera frame find the latest sensor timestamp that is <= t_cam."""
    print(f"\n{'=' * 60}")
    print("  Last-Before Matching  (latest sensor frame <= t_cam)")
    print(f"{'=' * 60}")
    print(f"  Camera frames       : {len(t_cam)}")
    print(f"  Sensor frames       : {len(t_sensor)}")

    matched_sensor_t = np.full(len(t_cam), np.nan, dtype=np.float64)
    matched_sensor_idx = np.full(len(t_cam), -1, dtype=np.int64)

    for i, tc in enumerate(t_cam):
        si = _last_before(t_sensor, tc)
        if si >= 0:
            matched_sensor_t[i] = t_sensor[si]
            matched_sensor_idx[i] = si

    # Frames with no valid match (camera started before sensor stream)
    unmatched_mask = matched_sensor_idx < 0
    n_unmatched = unmatched_mask.sum()
    if n_unmatched:
        print(f"\n  ⚠  Camera frames with no prior sensor reading: {n_unmatched}")
        print(f"     (these are excluded from time-difference stats)")

    # Stats only on matched frames; diff is always >= 0 (sensor is before cam)
    valid_mask = ~unmatched_mask
    time_diffs = t_cam[valid_mask] - matched_sensor_t[valid_mask]

    print(f"\n  --- t_cam − t_sensor_matched (s)  [sensor always <= cam] ---")
    print(f"  Matched frames      : {valid_mask.sum()} / {len(t_cam)}")
    print(f"  Mean                : {time_diffs.mean():.6f}")
    print(f"  Std                 : {time_diffs.std():.6f}")
    print(f"  Median              : {np.median(time_diffs):.6f}")
    print(f"  Min                 : {time_diffs.min():.6f}")
    print(f"  Max                 : {time_diffs.max():.6f}")

    # Percentiles
    for p in (50, 75, 90, 95, 99):
        print(f"  p{p:<2}                 : {np.percentile(time_diffs, p):.6f}")

    # Threshold breakdown
    print(f"\n  --- Fraction of matches within threshold ---")
    for thresh_ms in (1, 2, 5, 10, 20, 50):
        thresh_s = thresh_ms / 1000.0
        frac = (time_diffs <= thresh_s).mean() * 100
        print(f"  ≤ {thresh_ms:>2} ms            : {frac:6.2f} %")

    # Worst matches (largest lag = sensor data most stale)
    worst_n = min(10, int(valid_mask.sum()))
    valid_cam_indices = np.where(valid_mask)[0]
    worst_idx = valid_cam_indices[np.argsort(time_diffs)[-worst_n:][::-1]]
    print(f"\n  --- Top-{worst_n} worst matches (largest sensor lag) ---")
    print(f"  {'cam_idx':>8}  {'t_cam':>14}  {'t_sensor':>14}  {'Δt (s)':>10}")
    for wi in worst_idx:
        print(
            f"  {wi:>8d}  {t_cam[wi]:>14.6f}  {matched_sensor_t[wi]:>14.6f}  "
            f"{t_cam[wi] - matched_sensor_t[wi]:>10.6f}"
        )

    # Sensor re-use check
    valid_sensor_indices = matched_sensor_idx[valid_mask]
    unique, counts = np.unique(valid_sensor_indices, return_counts=True)
    reused = (counts > 1).sum()
    print(f"\n  Distinct sensor frames used : {len(unique)} / {len(t_sensor)}")
    print(f"  Sensor frames reused (>1×)  : {reused}")
    if reused:
        top_reused = np.argsort(counts)[-5:][::-1]
        print(f"  Most-reused sensor frames:")
        for ri in top_reused:
            if counts[ri] > 1:
                print(
                    f"    sensor_idx={unique[ri]}  t={t_sensor[unique[ri]]:.6f}  "
                    f"matched {counts[ri]}× by camera"
                )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    if len(sys.argv) != 3:
        print("Usage: python analyze_timing.py <data_root> <sensors_root>")
        sys.exit(1)

    data_root = sys.argv[1]
    sensors_root = sys.argv[2]

    print(f"\nData root    : {data_root}")
    print(f"Sensors root : {sensors_root}")

    t_cam = load_cam_timestamps(data_root)
    t_sensor = load_sensor_timestamps(sensors_root)

    frequency_stats(t_cam, label="Nav-Cam Timestamps")
    frequency_stats(t_sensor, label="Mocap Sensor Timestamps")
    nearest_neighbour_analysis(t_cam, t_sensor)

    print(f"\n{'=' * 60}\n")


if __name__ == "__main__":
    main()
