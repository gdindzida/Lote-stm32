"""KPI: accuracy metrics for HIL tx / ty / theta estimates vs. ground truth.

Algorithm context
-----------------
The STM32 firmware receives one 96×96 grayscale query image per frame and
fits a 2-D rigid-body motion model (translation tx/ty in pixels, rotation
theta in radians) to the block-matching displacement vectors found inside
that image.

Ground-truth derivation
-----------------------
Because the STM32 output represents frame-to-frame ego-motion, the ground
truth for frame *i* is the **inter-frame motion** between query frame *i* and
query frame *i + 1* as measured by the on-board GPS/IMU recorded in
``query.csv``:

tx, ty  – UTM inter-frame displacement (East / North) converted to
          96 × 96 pixel units.  The GSD is either supplied explicitly
          via *gsd_m_per_px* or derived automatically from the per-frame
          ``altitude`` column in ``query.csv`` and the camera HFOV
          (see :func:`compute_gsd` and :func:`load_ground_truth`).
          Positive tx → East,  positive ty → North (UTM convention).

theta   – Change in camera yaw (radians) between consecutive frames,
          derived from the scalar-last quaternion in ``query.csv`` and
          wrapped to (−π, π].

Metrics reported (per axis and 2-D combined)
--------------------------------------------
MAE   Mean Absolute Error
RMSE  Root Mean Square Error
Max   Maximum absolute error
Bias  Mean signed error  (pred − gt)
Std   Standard deviation of the (pred − gt) error
R²    Coefficient of determination
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Sequence, Tuple

try:
    import numpy as np
except ImportError as exc:
    raise ImportError(
        "numpy is required for KPI computation. " "Install it with: pip install numpy"
    ) from exc


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Return yaw (rotation around Z-axis) from a scalar-last quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle: float) -> float:
    """Wrap *angle* to (−π, π]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Result containers
# ---------------------------------------------------------------------------


@dataclass
class AxisMetrics:
    """Accuracy metrics for a single scalar axis (tx, ty, or theta)."""

    mae: float  # Mean Absolute Error
    rmse: float  # Root Mean Square Error
    max_err: float  # Maximum absolute error
    bias: float  # Mean signed error (pred − gt)
    std_err: float  # Standard deviation of (pred − gt)
    r2: float  # Coefficient of determination (NaN if gt has zero variance)
    n: int  # Number of samples used


@dataclass
class KPIResult:
    """Full KPI result for a HIL run."""

    tx: AxisMetrics
    ty: AxisMetrics
    theta: AxisMetrics

    # Combined 2-D translation error: magnitude = sqrt(tx_err² + ty_err²)
    translation_mae: float  # Mean of combined error magnitudes
    translation_rmse: float  # RMSE of combined error magnitudes
    translation_max: float  # Maximum combined error magnitude


# ---------------------------------------------------------------------------
# Indoor ground-truth loading (INSANE dataset)
# ---------------------------------------------------------------------------


def _nearest_index(sorted_times: "np.ndarray", query_time: float) -> int:
    """Return the index in *sorted_times* whose value is closest to *query_time*."""
    idx = int(np.searchsorted(sorted_times, query_time))
    if idx == 0:
        return 0
    if idx >= len(sorted_times):
        return len(sorted_times) - 1
    # Choose the closer of the two neighbours.
    if abs(sorted_times[idx] - query_time) < abs(sorted_times[idx - 1] - query_time):
        return idx
    return idx - 1


# ---------------------------------------------------------------------------
# Velocity ground truth computation (indoor dataset)
# ---------------------------------------------------------------------------


def compute_and_print_kpi(
    frame_meta_list: "Sequence[tuple[int, float, float, float]]",
    data_root: str,
    sensors_root: str,
    plot_kpi: bool = False,
) -> None:
    """Compute and print velocity KPI metrics comparing STM32 output to ground truth.

    Parameters
    ----------
    frame_meta_list : Sequence[tuple[int, float, float, float]]
        List of (frame_number, vx_pred, vy_pred, omega_pred) tuples from STM32.
    data_root : str
        Path to the nav-cam folder (e.g. ``…/indoor_1_nav_cam``).
    sensors_root : str
        Path to the sensors folder (e.g. ``…/indoor_1_sensors``).
    plot_kpi : bool, optional
        If True, display timeseries plots of predicted vs ground truth velocities.
        Requires matplotlib. Default is False.
    """
    print("")
    print("Computing velocity KPI…")
    try:
        # Extract predicted velocities from STM32
        frame_numbers = [fm[0] for fm in frame_meta_list]
        vx_pred = np.array([fm[1] for fm in frame_meta_list], dtype=np.float64)
        vy_pred = np.array([fm[2] for fm in frame_meta_list], dtype=np.float64)
        omega_pred = np.array([fm[3] for fm in frame_meta_list], dtype=np.float64)

        # Compute ground truth velocities from position/orientation changes over time
        vx_gt, vy_gt, omega_gt = compute_velocity_gt_indoor(
            data_root,
            sensors_root=sensors_root,
            frame_numbers=frame_numbers,
        )

        # Compute simple metrics (MAE, RMSE) for valid frames
        valid_mask = ~np.isnan(vx_gt)
        if valid_mask.any():
            vx_mae = float(np.mean(np.abs(vx_pred[valid_mask] - vx_gt[valid_mask])))
            vy_mae = float(np.mean(np.abs(vy_pred[valid_mask] - vy_gt[valid_mask])))
            omega_mae = float(
                np.mean(np.abs(omega_pred[valid_mask] - omega_gt[valid_mask]))
            )

            vx_rmse = float(
                np.sqrt(np.mean((vx_pred[valid_mask] - vx_gt[valid_mask]) ** 2))
            )
            vy_rmse = float(
                np.sqrt(np.mean((vy_pred[valid_mask] - vy_gt[valid_mask]) ** 2))
            )
            omega_rmse = float(
                np.sqrt(np.mean((omega_pred[valid_mask] - omega_gt[valid_mask]) ** 2))
            )

            print(
                f"  Velocity MAE  — vx: {vx_mae:.4f} m/s,  vy: {vy_mae:.4f} m/s,  omega: {omega_mae:.4f} rad/s"
            )
            print(
                f"  Velocity RMSE — vx: {vx_rmse:.4f} m/s,  vy: {vy_rmse:.4f} m/s,  omega: {omega_rmse:.4f} rad/s"
            )

        if plot_kpi:
            from hil.plot import plot_velocity_xyz

            plot_velocity_xyz(
                frame_numbers,
                vx_pred,
                vy_pred,
                omega_pred,
                vx_gt,
                vy_gt,
                omega_gt,
            )
    except Exception as exc:
        print(f"KPI computation failed: {exc}")


def compute_velocity_gt_indoor(
    data_root: str,
    sensors_root: str,
    frame_numbers: Sequence[int],
) -> Tuple["np.ndarray", "np.ndarray", "np.ndarray"]:
    """Compute ground-truth velocities (vx, vy, omega) for indoor dataset frames.

    Computes velocity as the rate of change of position and orientation over time
    from mocap or odometry data, aligned to camera timestamps.

    Parameters
    ----------
    data_root:
        Path to the nav-cam folder (e.g. ``…/indoor_1_nav_cam``).
        Must contain ``nav_cam_timestamps.csv``.
    sensors_root:
        Path to the matching sensors folder (e.g. ``…/indoor_1_sensors``).
        Must contain ``mocap_vehicle_data.csv`` (preferred) or ``rs_odom.csv``.
    frame_numbers:
        Dataset frame indices for which to compute ground truth velocities.

    Returns
    -------
    vx_gt, vy_gt, omega_gt : np.ndarray, shape (len(frame_numbers),)
        Ground-truth velocity arrays aligned with *frame_numbers*.
        vx, vy in m/s (velocity in x/y directions),
        omega in rad/s (angular velocity around z-axis).
        Entries are ``NaN`` when velocity cannot be computed (e.g. first frame,
        or when timestamps are too close).

    Raises
    ------
    FileNotFoundError
        If required CSV files are not found.
    """
    import csv as _csv

    # --- Load camera timestamps ---
    cam_csv = os.path.join(data_root, "nav_cam_timestamps.csv")
    if not os.path.isfile(cam_csv):
        raise FileNotFoundError(
            f"nav_cam_timestamps.csv not found in data_root: {data_root}"
        )

    cam_times: list[float] = []
    with open(cam_csv, newline="") as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 2:
                continue
            try:
                cam_times.append(float(parts[1]))
            except ValueError:
                continue
    t_cam = np.array(cam_times, dtype=np.float64)
    n_cam = len(t_cam)
    if n_cam < 2:
        raise ValueError(
            f"nav_cam_timestamps.csv must have at least 2 rows (found {n_cam})."
        )

    # --- Load sensor (mocap / odometry) data ---
    mocap_csv = os.path.join(sensors_root, "mocap_vehicle_data.csv")
    odom_csv = os.path.join(sensors_root, "rs_odom.csv")

    if os.path.isfile(mocap_csv):
        sensor_csv = mocap_csv
        print(f"  GT velocity source: mocap  ({sensor_csv})")
    elif os.path.isfile(odom_csv):
        sensor_csv = odom_csv
        print(f"  GT velocity source: rs_odom  ({sensor_csv})")
    else:
        raise FileNotFoundError(
            f"No ground-truth sensor file found in sensors_root: {sensors_root}\n"
            f"Expected 'mocap_vehicle_data.csv' or 'rs_odom.csv'."
        )

    # CSV header: t, p_x, p_y, p_z, q_w, q_x, q_y, q_z
    s_t: list[float] = []
    s_px: list[float] = []
    s_py: list[float] = []
    s_qw: list[float] = []
    s_qx: list[float] = []
    s_qy: list[float] = []
    s_qz: list[float] = []

    with open(sensor_csv, newline="") as fh:
        reader = _csv.reader(fh)
        header_row = next(reader)
        col = [h.strip() for h in header_row]
        idx_t = col.index("t")
        idx_px = col.index("p_x")
        idx_py = col.index("p_y")
        idx_qw = col.index("q_w")
        idx_qx = col.index("q_x")
        idx_qy = col.index("q_y")
        idx_qz = col.index("q_z")
        for row in reader:
            if not row:
                continue
            try:
                s_t.append(float(row[idx_t]))
                s_px.append(float(row[idx_px]))
                s_py.append(float(row[idx_py]))
                s_qw.append(float(row[idx_qw]))
                s_qx.append(float(row[idx_qx]))
                s_qy.append(float(row[idx_qy]))
                s_qz.append(float(row[idx_qz]))
            except (ValueError, IndexError):
                continue

    t_sensor = np.array(s_t, dtype=np.float64)
    px = np.array(s_px, dtype=np.float64)
    py = np.array(s_py, dtype=np.float64)
    qw = np.array(s_qw, dtype=np.float64)
    qx = np.array(s_qx, dtype=np.float64)
    qy = np.array(s_qy, dtype=np.float64)
    qz = np.array(s_qz, dtype=np.float64)

    # --- Align sensor data to all camera frames using nearest-neighbor ---
    aligned_px = np.empty(n_cam, dtype=np.float64)
    aligned_py = np.empty(n_cam, dtype=np.float64)
    aligned_yaw = np.empty(n_cam, dtype=np.float64)
    aligned_t = np.empty(n_cam, dtype=np.float64)

    for i in range(n_cam):
        si = _nearest_index(t_sensor, t_cam[i])
        aligned_px[i] = px[si]
        aligned_py[i] = py[si]
        aligned_yaw[i] = _quat_to_yaw(qx[si], qy[si], qz[si], qw[si])
        aligned_t[i] = t_cam[i]

    # --- Compute velocities as diff(position) / diff(time) ---
    # Velocity at frame i is computed from frame i-1 to frame i
    vx_all = np.full(n_cam, np.nan, dtype=np.float64)
    vy_all = np.full(n_cam, np.nan, dtype=np.float64)
    omega_all = np.full(n_cam, np.nan, dtype=np.float64)

    for i in range(1, n_cam):
        dt = aligned_t[i] - aligned_t[i - 1]
        if dt > 1e-6:  # avoid division by very small dt
            vx_all[i] = (aligned_px[i] - aligned_px[i - 1]) / dt
            vy_all[i] = (aligned_py[i] - aligned_py[i - 1]) / dt
            omega_all[i] = _wrap_angle(aligned_yaw[i] - aligned_yaw[i - 1]) / dt

    # --- Extract velocities for the requested frame numbers ---
    n_req = len(frame_numbers)
    vx_gt = np.full(n_req, np.nan, dtype=np.float64)
    vy_gt = np.full(n_req, np.nan, dtype=np.float64)
    omega_gt = np.full(n_req, np.nan, dtype=np.float64)

    for idx, fi in enumerate(frame_numbers):
        if 0 <= fi < n_cam:
            vx_gt[idx] = vx_all[fi]
            vy_gt[idx] = vy_all[fi]
            omega_gt[idx] = omega_all[fi]

    return vx_gt, vy_gt, omega_gt
