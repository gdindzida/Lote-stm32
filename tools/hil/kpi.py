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

import csv
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


def compute_gsd(
    altitude_m: float,
    hfov_deg: float,
    vfov_deg: float | None = None,
    orig_image_w: int = 500,
    orig_image_h: int = 500,
    scaled_image_w: int = 96,
    scaled_image_h: int = 96,
) -> tuple[float, float]:
    """Compute Ground Sample Distance for the STM32's scaled image.

    Assumes a pinhole (rectilinear) camera pointing nadir (straight down).
    Horizontal and vertical GSD are computed independently so that cameras
    with non-square sensors (HFOV ≠ VFOV) are handled correctly.

    Parameters
    ----------
    altitude_m:
        Camera altitude above the ground in metres.
    hfov_deg:
        Camera **horizontal** field of view in degrees.  Drives the East
        (tx) axis ground-truth conversion.
    vfov_deg:
        Camera **vertical** field of view in degrees.  Drives the North
        (ty) axis ground-truth conversion.  Defaults to *hfov_deg* (square
        sensor assumption) when ``None``.
    orig_image_w, orig_image_h:
        Original image width / height in pixels (default 500 × 500,
        matching the ALTO UAV dataset).
    scaled_image_w, scaled_image_h:
        Width / height of the scaled image sent to the STM32 (default
        96 × 96).

    Returns
    -------
    gsd_x, gsd_y : (float, float)
        *gsd_x* — metres per pixel along the **horizontal / East** axis of
        the scaled image.
        *gsd_y* — metres per pixel along the **vertical / North** axis of
        the scaled image.

    Examples
    --------
    >>> compute_gsd(530, 60)            # square sensor, 60° HFOV
    (6.366..., 6.366...)
    >>> compute_gsd(530, 60, vfov_deg=45)   # non-square sensor
    (6.366..., 4.637...)
    """
    if vfov_deg is None:
        vfov_deg = hfov_deg

    footprint_x = 2.0 * altitude_m * math.tan(math.radians(hfov_deg) / 2.0)
    footprint_y = 2.0 * altitude_m * math.tan(math.radians(vfov_deg) / 2.0)

    gsd_x = (footprint_x / orig_image_w) * (orig_image_w / scaled_image_w)
    gsd_y = (footprint_y / orig_image_h) * (orig_image_h / scaled_image_h)
    return gsd_x, gsd_y


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
# Ground-truth loading
# ---------------------------------------------------------------------------


def load_ground_truth(
    data_root: str,
    gsd_m_per_px: float = 0.0,
    hfov_deg: float = 60.0,
    vfov_deg: float | None = None,
) -> Tuple[
    "np.ndarray", "np.ndarray", "np.ndarray", "float | np.ndarray", "float | np.ndarray"
]:
    """Load query telemetry and compute per-frame ground-truth tx/ty/theta.

    GSD resolution order
    --------------------
    1. *gsd_m_per_px > 0* → use it for **both** axes (manual override,
       square-pixel assumption).
    2. *gsd_m_per_px ≤ 0* and ``altitude`` column present in query.csv →
       auto-compute **per-frame** per-axis GSD from each frame's individual
       ``altitude`` value + *hfov_deg* / *vfov_deg* via :func:`compute_gsd`.
       This correctly accounts for altitude variation across the flight.
    3. Both conditions fail → raise :class:`ValueError`.

    Parameters
    ----------
    data_root:
        Path to the dataset split folder (e.g. ``/path/to/UAV/Train``).
        Must contain a ``query.csv`` file with columns:
        ``easting``, ``northing``, ``orient_x``, ``orient_y``,
        ``orient_z``, ``orient_w``.
        If ``altitude`` is also present it is used for automatic per-frame
        GSD derivation when *gsd_m_per_px ≤ 0*.
    gsd_m_per_px:
        Uniform Ground Sample Distance in **metres per pixel** applied to
        both axes.  Pass ``0.0`` (default) to auto-compute per-frame
        per-axis GSD from the ``altitude`` column, *hfov_deg*, and *vfov_deg*.
    hfov_deg:
        Camera **horizontal** field of view in degrees (default 60°).
        Controls the *tx* (East) axis GSD.  Used only in auto mode.
    vfov_deg:
        Camera **vertical** field of view in degrees.  Controls the *ty*
        (North) axis GSD.  Defaults to *hfov_deg* (square-sensor
        assumption).  Used only in auto mode.

    Returns
    -------
    tx_gt, ty_gt, theta_gt : np.ndarray, shape (N,)
        Ground-truth arrays aligned with the query image index.
        The *last* entry of each array is ``NaN`` because there is no
        frame *i + 1* for the final image.
    gsd_x, gsd_y : float or np.ndarray, shape (N,)
        GSD values (m/px) for the East (*tx*) and North (*ty*) axes.
        Returns a scalar ``float`` when *gsd_m_per_px* > 0 (manual uniform
        GSD), or a per-frame ``np.ndarray`` when auto-computed from altitude.
    """
    query_csv_path = os.path.join(data_root, "query.csv")
    if not os.path.isfile(query_csv_path):
        raise FileNotFoundError(
            f"query.csv not found in data_root: {data_root}\n"
            f"Expected path: {query_csv_path}"
        )

    # --- Read CSV with stdlib (no pandas dependency) ---
    required_cols = {
        "easting",
        "northing",
        "orient_x",
        "orient_y",
        "orient_z",
        "orient_w",
    }
    with open(query_csv_path, newline="") as fh:
        reader = csv.DictReader(fh)
        header = set(reader.fieldnames or [])
        missing = required_cols - header
        if missing:
            raise ValueError(
                f"query.csv is missing required columns: {missing}\n"
                f"Found columns: {sorted(header)}"
            )
        rows = list(reader)

    n = len(rows)
    if n < 2:
        raise ValueError(
            f"query.csv must have at least 2 rows to compute inter-frame ground truth "
            f"(found {n})."
        )

    # --- Resolve GSD (per-axis) ---
    if gsd_m_per_px > 0.0:
        # Manual uniform GSD: same scalar value for both axes.
        gsd_x: "float | np.ndarray" = gsd_m_per_px
        gsd_y: "float | np.ndarray" = gsd_m_per_px
        print(f"  GSD (manual)     : gsd_x={gsd_x:.4f}  gsd_y={gsd_y:.4f}  m/px")
    elif "altitude" in header:
        # Per-frame GSD: compute gsd_x[i] and gsd_y[i] from each frame's altitude.
        altitudes = np.array([float(r["altitude"]) for r in rows], dtype=np.float64)
        effective_vfov = vfov_deg if vfov_deg is not None else hfov_deg
        gsd_x_arr = np.empty(n, dtype=np.float64)
        gsd_y_arr = np.empty(n, dtype=np.float64)
        for i in range(n):
            gsd_x_arr[i], gsd_y_arr[i] = compute_gsd(altitudes[i], hfov_deg, vfov_deg)
        gsd_x = gsd_x_arr
        gsd_y = gsd_y_arr
        print(
            f"  GSD per-frame auto-computed from altitude"
            f"  [alt range: {altitudes.min():.1f}–{altitudes.max():.1f} m,"
            f" HFOV={hfov_deg:.1f}°, VFOV={effective_vfov:.1f}°]\n"
            f"  gsd_x: mean={np.mean(gsd_x_arr):.4f}"
            f"  min={np.min(gsd_x_arr):.4f}"
            f"  max={np.max(gsd_x_arr):.4f}  m/px\n"
            f"  gsd_y: mean={np.mean(gsd_y_arr):.4f}"
            f"  min={np.min(gsd_y_arr):.4f}"
            f"  max={np.max(gsd_y_arr):.4f}  m/px"
        )
    else:
        raise ValueError(
            "Cannot auto-compute GSD: 'altitude' column not found in query.csv "
            "and --gsd was not provided (or is ≤ 0).\n"
            "Supply --gsd M_PER_PX explicitly."
        )

    easting = np.array([float(r["easting"]) for r in rows], dtype=np.float64)
    northing = np.array([float(r["northing"]) for r in rows], dtype=np.float64)
    orient_x = np.array([float(r["orient_x"]) for r in rows], dtype=np.float64)
    orient_y = np.array([float(r["orient_y"]) for r in rows], dtype=np.float64)
    orient_z = np.array([float(r["orient_z"]) for r in rows], dtype=np.float64)
    orient_w = np.array([float(r["orient_w"]) for r in rows], dtype=np.float64)

    # Pre-compute yaw for every frame
    yaws = np.array(
        [
            _quat_to_yaw(orient_x[i], orient_y[i], orient_z[i], orient_w[i])
            for i in range(n)
        ],
        dtype=np.float64,
    )

    tx_gt = np.full(n, np.nan, dtype=np.float64)
    ty_gt = np.full(n, np.nan, dtype=np.float64)
    theta_gt = np.full(n, np.nan, dtype=np.float64)

    # Ground truth for frame i = motion from frame i → frame i+1.
    # tx uses gsd_x (horizontal/East), ty uses gsd_y (vertical/North).
    # When GSD is a per-frame array, use gsd_x[i] / gsd_y[i] for each
    # inter-frame displacement so that the altitude at the source frame
    # is used for the pixel-unit conversion.
    dx_m = np.diff(easting)  # shape (n-1,)
    dy_m = np.diff(northing)  # shape (n-1,)

    if isinstance(gsd_x, np.ndarray) and isinstance(gsd_y, np.ndarray):
        # Per-frame GSD: gsd_x[i] corresponds to frame i → frame i+1.
        tx_gt[:-1] = dx_m / gsd_x[:-1]
        ty_gt[:-1] = dy_m / gsd_y[:-1]
    else:
        # Uniform GSD (manual override).
        tx_gt[:-1] = dx_m / float(gsd_x)
        ty_gt[:-1] = dy_m / float(gsd_y)
    theta_gt[:-1] = np.array(
        [_wrap_angle(yaws[i + 1] - yaws[i]) for i in range(n - 1)],
        dtype=np.float64,
    )

    return tx_gt, ty_gt, theta_gt, gsd_x, gsd_y


# ---------------------------------------------------------------------------
# Metrics computation
# ---------------------------------------------------------------------------


def _axis_metrics(
    pred: "np.ndarray",
    gt: "np.ndarray",
    pre_wrapped_errors: "np.ndarray | None" = None,
) -> AxisMetrics:
    """Compute accuracy metrics for one axis.

    Parameters
    ----------
    pred:
        Predicted values.
    gt:
        Ground-truth values (must have same length as *pred*).
    pre_wrapped_errors:
        If supplied, these are used as (pred − gt) instead of computing
        ``pred - gt`` directly (useful for circular quantities like theta
        where the raw difference can exceed π).
    """
    err = pre_wrapped_errors if pre_wrapped_errors is not None else (pred - gt)
    abs_err = np.abs(err)

    mae = float(np.mean(abs_err))
    rmse = float(np.sqrt(np.mean(err**2)))
    max_err = float(np.max(abs_err))
    bias = float(np.mean(err))
    std_err = float(np.std(err, ddof=0))

    ss_res = float(np.sum(err**2))
    ss_tot = float(np.sum((gt - np.mean(gt)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")

    return AxisMetrics(
        mae=mae,
        rmse=rmse,
        max_err=max_err,
        bias=bias,
        std_err=std_err,
        r2=r2,
        n=int(len(pred)),
    )


def compute_kpi(
    frame_numbers: Sequence[int],
    tx_pred: Sequence[float],
    ty_pred: Sequence[float],
    theta_pred: Sequence[float],
    tx_gt: "np.ndarray",
    ty_gt: "np.ndarray",
    theta_gt: "np.ndarray",
) -> KPIResult:
    """Compute KPI metrics by aligning predictions with ground-truth.

    Only frames that have a valid (non-NaN) ground-truth entry are used.
    The last query frame always lacks ground truth because there is no
    frame *i + 1* to compute the inter-frame displacement.

    Parameters
    ----------
    frame_numbers:
        Dataset query-image index for each prediction (0-based, matching
        the order returned by :class:`~hil.uav.UAVStreamer`).
    tx_pred, ty_pred, theta_pred:
        STM32 output for each entry in *frame_numbers*.
    tx_gt, ty_gt, theta_gt:
        Ground-truth arrays as returned by :func:`load_ground_truth`.
        Indexed by dataset frame index; entries may be NaN.

    Returns
    -------
    KPIResult
        Accuracy metrics for tx, ty, theta, and combined translation error.

    Raises
    ------
    ValueError
        If no valid ground-truth entries are found for the recorded frames.
    """
    gt_len = len(tx_gt)

    # Find predictions whose corresponding gt entry is not NaN
    valid_mask = [fi < gt_len and not np.isnan(tx_gt[fi]) for fi in frame_numbers]
    valid_positions = [i for i, ok in enumerate(valid_mask) if ok]

    if not valid_positions:
        raise ValueError(
            "No valid ground-truth entries found for the recorded frames.\n"
            "Check that --data-root matches the dataset used during the run\n"
            "and that query.csv is present."
        )

    p_tx = np.array([tx_pred[i] for i in valid_positions], dtype=np.float64)
    p_ty = np.array([ty_pred[i] for i in valid_positions], dtype=np.float64)
    p_theta = np.array([theta_pred[i] for i in valid_positions], dtype=np.float64)

    g_tx = np.array(
        [tx_gt[frame_numbers[i]] for i in valid_positions], dtype=np.float64
    )
    g_ty = np.array(
        [ty_gt[frame_numbers[i]] for i in valid_positions], dtype=np.float64
    )
    g_theta = np.array(
        [theta_gt[frame_numbers[i]] for i in valid_positions], dtype=np.float64
    )

    # Wrap theta errors to (−π, π] before computing metrics
    theta_err = np.array(
        [_wrap_angle(p - g) for p, g in zip(p_theta, g_theta)],
        dtype=np.float64,
    )

    tx_metrics = _axis_metrics(p_tx, g_tx)
    ty_metrics = _axis_metrics(p_ty, g_ty)
    theta_metrics = _axis_metrics(p_theta, g_theta, pre_wrapped_errors=theta_err)

    # Combined 2-D translation error magnitude per frame
    trans_err = np.sqrt((p_tx - g_tx) ** 2 + (p_ty - g_ty) ** 2)
    translation_mae = float(np.mean(trans_err))
    translation_rmse = float(np.sqrt(np.mean(trans_err**2)))
    translation_max = float(np.max(trans_err))

    return KPIResult(
        tx=tx_metrics,
        ty=ty_metrics,
        theta=theta_metrics,
        translation_mae=translation_mae,
        translation_rmse=translation_rmse,
        translation_max=translation_max,
    )


# ---------------------------------------------------------------------------
# Report formatting
# ---------------------------------------------------------------------------

_SEP = "-" * 70


def _fmt_axis(label: str, m: AxisMetrics, unit: str) -> str:
    r2_str = f"{m.r2:+.4f}" if not math.isnan(m.r2) else "  N/A  "
    return (
        f"\n  {label} ({unit})  [n = {m.n}]\n"
        f"    MAE     : {m.mae:>10.4f}\n"
        f"    RMSE    : {m.rmse:>10.4f}\n"
        f"    Max err : {m.max_err:>10.4f}\n"
        f"    Bias    : {m.bias:>+10.4f}\n"
        f"    Std err : {m.std_err:>10.4f}\n"
        f"    R²      : {r2_str}\n"
    )


def print_kpi_report(
    result: KPIResult,
    gsd_x: "float | np.ndarray",
    gsd_y: "float | np.ndarray",
) -> None:
    """Print a formatted KPI report to stdout.

    Parameters
    ----------
    result:
        :class:`KPIResult` from :func:`compute_kpi`.
    gsd_x:
        GSD applied to the East / tx axis (m/px), for display only.
        May be a scalar ``float`` (manual override) or a per-frame
        ``np.ndarray`` (auto-computed from altitude).
    gsd_y:
        GSD applied to the North / ty axis (m/px), for display only.
        May be a scalar ``float`` (manual override) or a per-frame
        ``np.ndarray`` (auto-computed from altitude).
    """
    unit_trans = "px"
    unit_theta = "rad"

    print("")
    print("=" * 70)
    print("  HIL KPI — Translation & Rotation Accuracy")
    if isinstance(gsd_x, np.ndarray) or isinstance(gsd_y, np.ndarray):
        # Per-frame GSD: show summary statistics (mean / min / max).
        gsd_x_arr = np.asarray(gsd_x, dtype=np.float64)
        gsd_y_arr = np.asarray(gsd_y, dtype=np.float64)
        print(
            f"  GSD x (East)  [per-frame]: "
            f"mean={np.mean(gsd_x_arr):.4f}"
            f"  min={np.min(gsd_x_arr):.4f}"
            f"  max={np.max(gsd_x_arr):.4f}  m/px  — tx"
        )
        print(
            f"  GSD y (North) [per-frame]: "
            f"mean={np.mean(gsd_y_arr):.4f}"
            f"  min={np.min(gsd_y_arr):.4f}"
            f"  max={np.max(gsd_y_arr):.4f}  m/px  — ty"
        )
    elif gsd_x == gsd_y:
        print(f"  GSD (x = y): {gsd_x:.4f} m/px  (tx/ty in 96×96 pixel units)")
    else:
        print(f"  GSD x (East) : {gsd_x:.4f} m/px  — used for tx ground truth")
        print(f"  GSD y (North): {gsd_y:.4f} m/px  — used for ty ground truth")
    print("=" * 70)

    print(_fmt_axis("tx (East translation)", result.tx, unit_trans), end="")
    print(_SEP)
    print(_fmt_axis("ty (North translation)", result.ty, unit_trans), end="")
    print(_SEP)
    print(_fmt_axis("theta (rotation)", result.theta, unit_theta), end="")
    print(_SEP)
    print(
        f"\n  Combined 2-D translation error  ||(tx_err, ty_err)||₂\n"
        f"    MAE     : {result.translation_mae:>10.4f} {unit_trans}\n"
        f"    RMSE    : {result.translation_rmse:>10.4f} {unit_trans}\n"
        f"    Max err : {result.translation_max:>10.4f} {unit_trans}\n"
    )
    print("=" * 70)
