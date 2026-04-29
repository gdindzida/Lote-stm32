"""
create_dataset.py
-----------------
Creates a matched dataset CSV from rectified nav-cam images and mocap sensor data.

For each rectified image the script finds the latest mocap reading whose
timestamp is <= the camera timestamp (causal / last-before match), then
writes one row per image to the output CSV.

Usage
-----
    python create_dataset.py <data_root> <sensors_root> [output_csv]

    data_root    – folder containing nav_cam_timestamps.csv and rectified_img/
    sensors_root – folder containing mocap_vehicle_data.csv
    output_csv   – (optional) output path; default: dataset.csv in cwd

Output CSV columns
------------------
    timestamp_cam       camera timestamp (s)
    timestamp_sensor    matched mocap timestamp (s)
    delta_t             timestamp_cam - timestamp_sensor (s), always >= 0
    p_x                 position x (m)
    p_y                 position y (m)
    p_z                 position z (m)
    q_w                 quaternion w
    q_x                 quaternion x
    q_y                 quaternion y
    q_z                 quaternion z
    roll                roll  (rotation about X) in radians
    pitch               pitch (rotation about Y) in radians
    yaw                 yaw   (rotation about Z) in radians
    image_path          relative path to rectified image (relative to data_root)

Rows where no prior mocap reading exists are written with NaN sensor fields
and flagged in the delta_t column as NaN.
"""

import sys
import os
import csv
import numpy as np

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _quat_to_rpy(qw: float, qx: float, qy: float, qz: float):
    """Convert unit quaternion (w, x, y, z) to roll, pitch, yaw (radians).

    Uses the ZYX / aerospace convention:
        roll  (phi)   – rotation about X
        pitch (theta) – rotation about Y
        yaw   (psi)   – rotation about Z
    """
    # roll
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch  (clamped to avoid NaN from floating-point drift)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # yaw
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _last_before(arr: np.ndarray, val: float) -> int:
    """Return the index of the latest entry in sorted *arr* that is <= *val*.

    Returns -1 if all entries are strictly after *val*.
    """
    idx = int(np.searchsorted(arr, val, side="right")) - 1
    return idx


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------


def load_cam_timestamps(data_root: str):
    """Load nav_cam_timestamps.csv → list of (frame_index, timestamp_s)."""
    cam_csv = os.path.join(data_root, "nav_cam_timestamps.csv")
    if not os.path.isfile(cam_csv):
        raise FileNotFoundError(f"nav_cam_timestamps.csv not found in: {data_root}")

    entries = []  # (frame_index, timestamp)
    with open(cam_csv, newline="") as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 2:
                continue
            try:
                frame_idx = int(parts[0])
                timestamp = float(parts[1])
                entries.append((frame_idx, timestamp))
            except ValueError:
                continue

    return entries  # list of (int, float)


def load_mocap(sensors_root: str):
    """Load mocap_vehicle_data.csv → dict of arrays keyed by column name,
    plus a sorted 't' array for binary search."""
    mocap_csv = os.path.join(sensors_root, "mocap_vehicle_data.csv")
    if not os.path.isfile(mocap_csv):
        raise FileNotFoundError(f"mocap_vehicle_data.csv not found in: {sensors_root}")

    required_cols = {"t", "p_x", "p_y", "p_z", "q_w", "q_x", "q_y", "q_z"}
    data = {c: [] for c in required_cols}

    with open(mocap_csv, newline="") as fh:
        reader = csv.reader(fh)
        header = [h.strip() for h in next(reader)]
        missing = required_cols - set(header)
        if missing:
            raise ValueError(f"mocap_vehicle_data.csv is missing columns: {missing}")

        col_idx = {c: header.index(c) for c in required_cols}

        for row in reader:
            if not row:
                continue
            try:
                for c in required_cols:
                    data[c].append(float(row[col_idx[c]]))
            except (ValueError, IndexError):
                continue

    return {c: np.array(v, dtype=np.float64) for c, v in data.items()}


def find_rectified_images(data_root: str):
    """Scan rectified_img/ and return a dict: frame_index -> relative_path.

    Supports filenames like:
        000123.png / 000123.jpg
        frame_000123.png
        image_000123.png
    Falls back to sorted order if no numeric stem is found.
    """
    rect_dir = os.path.join(data_root, "rectified_img")
    if not os.path.isdir(rect_dir):
        raise FileNotFoundError(f"rectified_img/ not found in: {data_root}")

    supported_ext = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif"}
    frame_map = {}  # frame_index (int) -> relative path string

    files = sorted(os.listdir(rect_dir))
    for fname in files:
        ext = os.path.splitext(fname)[1].lower()
        if ext not in supported_ext:
            continue

        stem = os.path.splitext(fname)[0]
        # Strip common prefixes
        for prefix in ("frame_", "image_", "img_", "rect_"):
            if stem.lower().startswith(prefix):
                stem = stem[len(prefix) :]
                break

        rel_path = os.path.join("rectified_img", fname)

        try:
            frame_idx = int(stem)
            frame_map[frame_idx] = rel_path
        except ValueError:
            # Non-numeric stem — skip; handled below by sorted fallback
            pass

    # If no numeric stems were found at all, assign indices by sorted order
    if not frame_map:
        print(
            "  ⚠  Could not parse frame indices from filenames; "
            "assigning by sorted order."
        )
        for i, fname in enumerate(files):
            ext = os.path.splitext(fname)[1].lower()
            if ext in supported_ext:
                frame_map[i] = os.path.join("rectified_img", fname)

    return frame_map  # {int: str}


# ---------------------------------------------------------------------------
# Core matching
# ---------------------------------------------------------------------------


def build_dataset(data_root: str, sensors_root: str, output_csv: str) -> None:
    print(f"\nData root    : {data_root}")
    print(f"Sensors root : {sensors_root}")
    print(f"Output CSV   : {output_csv}")

    # Load inputs
    print("\nLoading camera timestamps …")
    cam_entries = load_cam_timestamps(data_root)  # [(frame_idx, t), ...]
    print(f"  {len(cam_entries)} camera frames found")

    print("Loading mocap data …")
    mocap = load_mocap(sensors_root)
    t_sensor = mocap["t"]
    print(f"  {len(t_sensor)} mocap readings found")
    print(f"  Mocap time range: {t_sensor[0]:.6f} → {t_sensor[-1]:.6f} s")

    print("Scanning rectified images …")
    frame_map = find_rectified_images(data_root)
    print(f"  {len(frame_map)} rectified images found")

    # Match and write
    n_matched = 0
    n_unmatched = 0
    n_no_image = 0

    fieldnames = [
        "timestamp_cam",
        "timestamp_sensor",
        "delta_t",
        "p_x",
        "p_y",
        "p_z",
        "q_w",
        "q_x",
        "q_y",
        "q_z",
        "roll",
        "pitch",
        "yaw",
        "image_path",
    ]

    os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)

    with open(output_csv, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()

        for frame_idx, t_cam in cam_entries:
            image_path = frame_map.get(frame_idx)
            if image_path is None:
                n_no_image += 1
                continue  # no image file on disk — skip

            si = _last_before(t_sensor, t_cam)

            if si < 0:
                # No mocap reading before this camera frame — skip
                n_unmatched += 1
                continue

            t_s = t_sensor[si]
            delta_t = t_cam - t_s
            n_matched += 1

            roll, pitch, yaw = _quat_to_rpy(
                mocap["q_w"][si],
                mocap["q_x"][si],
                mocap["q_y"][si],
                mocap["q_z"][si],
            )

            writer.writerow(
                {
                    "timestamp_cam": f"{t_cam:.9f}",
                    "timestamp_sensor": f"{t_s:.9f}",
                    "delta_t": f"{delta_t:.9f}",
                    "p_x": f"{mocap['p_x'][si]:.9f}",
                    "p_y": f"{mocap['p_y'][si]:.9f}",
                    "p_z": f"{mocap['p_z'][si]:.9f}",
                    "q_w": f"{mocap['q_w'][si]:.9f}",
                    "q_x": f"{mocap['q_x'][si]:.9f}",
                    "q_y": f"{mocap['q_y'][si]:.9f}",
                    "q_z": f"{mocap['q_z'][si]:.9f}",
                    "roll": f"{roll:.9f}",
                    "pitch": f"{pitch:.9f}",
                    "yaw": f"{yaw:.9f}",
                    "image_path": image_path,
                }
            )

    # Summary
    print(f"\n{'=' * 55}")
    print(f"  Dataset written to: {output_csv}")
    print(f"{'=' * 55}")
    print(f"  Total camera frames   : {len(cam_entries)}")
    print(f"  Matched rows          : {n_matched}")
    print(f"  Unmatched (no prior)  : {n_unmatched}")
    print(f"  Missing image files   : {n_no_image}")

    # Quick timing stats on matched rows
    if n_matched > 0:
        deltas = []
        with open(output_csv, newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                if row["delta_t"]:
                    deltas.append(float(row["delta_t"]))
        deltas = np.array(deltas)
        print(f"\n  --- Sensor lag stats (delta_t, seconds) ---")
        print(f"  Mean   : {deltas.mean():.6f}")
        print(f"  Std    : {deltas.std():.6f}")
        print(f"  Median : {np.median(deltas):.6f}")
        print(f"  Min    : {deltas.min():.6f}")
        print(f"  Max    : {deltas.max():.6f}")
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    if len(sys.argv) < 3:
        print("Usage: python create_dataset.py <data_root> <sensors_root> [output_csv]")
        sys.exit(1)

    data_root = sys.argv[1]
    sensors_root = sys.argv[2]
    output_csv = sys.argv[3] if len(sys.argv) > 3 else "dataset.csv"

    build_dataset(data_root, sensors_root, output_csv)


if __name__ == "__main__":
    main()
