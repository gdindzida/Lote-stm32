"""KPI: accuracy metrics for HIL velocity estimates vs. ground truth.

The STM32 firmware receives one 96×96 grayscale query image per frame and
outputs vx, vy (m/s) and omega (rad/s) estimates.  Ground-truth velocities
are pre-computed during streaming (see :class:`CsvDatasetStreamer.get_ground_truth`)
and attached to each :class:`FrameReadEvent`.  This module simply reads
that data and computes summary metrics.

Metrics reported (per axis)
---------------------------
MAE    Mean Absolute Error
RMSE   Root Mean Square Error
MAPE   Mean Absolute Percentage Error (percentage error vs. the reference/real value)

Comparisons reported
--------------------
Predicted vs OpenCV    STM32 prediction compared to OpenCV Farneback optical-flow
Predicted vs GT        STM32 prediction compared to position-derived ground truth
OpenCV vs GT           OpenCV Farneback compared to position-derived ground truth
"""

from __future__ import annotations

from typing import List, Optional
import cv2
from hil.frames import FrameReadEvent
from hil.plot import plot_velocities

try:
    import numpy as np
except ImportError as exc:
    raise ImportError(
        "numpy is required for KPI computation. Install it with: pip install numpy"
    ) from exc


def optical_flow_farneback(
    img1: np.ndarray, img2: np.ndarray
) -> tuple[float, float, np.ndarray]:
    """
    Dense optical flow using Farneback method.
    Computes flow for every pixel; returns average vx, vy.
    """
    # gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    # gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    flow = cv2.calcOpticalFlowFarneback(
        img1,
        img2,
        flow=None,
        pyr_scale=0.5,  # image pyramid scale
        levels=3,  # pyramid levels
        winsize=20,  # averaging window size
        iterations=5,  # iterations per pyramid level
        poly_n=7,  # pixel neighborhood size
        poly_sigma=1.5,  # Gaussian std for polynomial expansion
        flags=0,
    )
    # flow shape: (H, W, 2) — [:,:,0] = vx, [:,:,1] = vy
    avg_vx = float(np.median(flow[:, :, 0]))
    avg_vy = float(np.median(flow[:, :, 1]))

    return avg_vx, avg_vy, flow


def compute_and_print_kpi(
    frame_reads: List[FrameReadEvent],
    plot_kpi: bool = False,
    save_dir: Optional[str] = None,
) -> None:
    """Compute and print velocity KPI metrics comparing STM32 output to ground truth.

    Ground truth is read directly from the ``ground_truth`` field of each
    :class:`FrameReadEvent`, which was populated during streaming by the
    writer thread.  No additional computation or file loading is needed.

    Parameters
    ----------
    frame_reads : List[FrameReadEvent]
        Recorded read events from the HIL run containing STM32 predictions
        and pre-computed ground-truth velocities.
    plot_kpi : bool, optional
        If True, display timeseries plots of predicted vs ground truth
        velocities.  Requires matplotlib.  Default is False.
    save_dir : str or None, optional
        Directory in which to save generated plots as PNG files.
        If None, plots are only displayed (not saved).  Default is None.
    """
    print("")
    print("Computing velocity KPI…")

    # Filter to frames that have ground truth attached
    if not frame_reads:
        print("  No ground-truth data available — skipping KPI.")
        return

    try:
        vx_pred = np.array([fr.meta.vx for fr in frame_reads], dtype=np.float64)
        vy_pred = np.array([fr.meta.vy for fr in frame_reads], dtype=np.float64)
        debug_signal = np.array([fr.meta.debug for fr in frame_reads], dtype=np.float64)

        vx_opencv = np.zeros(len(frame_reads))
        vy_opencv = np.zeros(len(frame_reads))
        vx_pos = np.zeros(len(frame_reads))
        vy_pos = np.zeros(len(frame_reads))
        for i, _ in enumerate(frame_reads):
            if (
                i != 0
                and frame_reads[i].dt != 0
                and frame_reads[i].fx != 0
                and frame_reads[i].fy != 0
            ):
                # ── position-derived velocity (Δpos / Δt) ────────────────────
                vx_pos[i] = (frame_reads[i].px - frame_reads[i - 1].px) / frame_reads[
                    i
                ].dt
                vy_pos[i] = (frame_reads[i].py - frame_reads[i - 1].py) / frame_reads[
                    i
                ].dt

                # ── OpenCV Farneback optical-flow velocity ────────────────────
                pvx, pvy, _ = optical_flow_farneback(
                    frame_reads[i].image, frame_reads[i - 1].image
                )
                vx_opencv[i] = (
                    -pvx * frame_reads[i].pz / (frame_reads[i].dt * frame_reads[i].fx)
                )
                vy_opencv[i] = (
                    pvy * frame_reads[i].pz / (frame_reads[i].dt * frame_reads[i].fy)
                )
            else:
                vx_opencv[0] = 0.0
                vy_opencv[0] = 0.0
                vx_pos[0] = 0.0
                vy_pos[0] = 0.0

        # omega_gt = np.array(
        #     [fr.ground_truth.omega for fr in frames_with_gt], dtype=np.float64  # type: ignore[union-attr]
        # )

        # Only use frames where OpenCV GT velocity is valid (non-NaN)
        valid_mask = ~np.isnan(vx_opencv)
        if not valid_mask.any():
            print("  All ground-truth velocities are NaN — skipping KPI.")
            return

        def _mape(predicted: np.ndarray, reference: np.ndarray) -> float:
            """Mean Absolute Percentage Error.

            Frames where the reference velocity is zero (or very close to zero)
            are excluded from the percentage calculation to avoid division by
            zero producing meaningless infinity values.
            """
            nonzero = np.abs(reference) > 1e-9
            if not nonzero.any():
                return float("nan")
            return float(
                np.mean(np.abs(predicted[nonzero] - reference[nonzero]) / np.abs(reference[nonzero])) * 100.0
            )

        def _mae(a: np.ndarray, b: np.ndarray) -> float:
            return float(np.mean(np.abs(a - b)))

        def _rmse(a: np.ndarray, b: np.ndarray) -> float:
            return float(np.sqrt(np.mean((a - b) ** 2)))

        def _print_metrics(label: str, ref_label: str,
                           vx_a: np.ndarray, vx_b: np.ndarray,
                           vy_a: np.ndarray, vy_b: np.ndarray,
                           mask: np.ndarray) -> None:
            """Print MAE, RMSE and MAPE for a pair of velocity signals."""
            vxa, vxb = vx_a[mask], vx_b[mask]
            vya, vyb = vy_a[mask], vy_b[mask]
            print(f"\n  ── {label} vs {ref_label} ──")
            print(f"  MAE   vx: {_mae(vxa, vxb):.4f} m/s   vy: {_mae(vya, vyb):.4f} m/s")
            print(f"  RMSE  vx: {_rmse(vxa, vxb):.4f} m/s   vy: {_rmse(vya, vyb):.4f} m/s")
            mape_vx = _mape(vxa, vxb)
            mape_vy = _mape(vya, vyb)
            vx_str = f"{mape_vx:.2f} %" if not np.isnan(mape_vx) else "N/A (ref≈0)"
            vy_str = f"{mape_vy:.2f} %" if not np.isnan(mape_vy) else "N/A (ref≈0)"
            print(f"  MAPE  vx: {vx_str}   vy: {vy_str}")

        # Also apply a GT-validity mask (non-zero dt and focal lengths were used)
        valid_gt_mask = valid_mask & (
            np.array([fr.dt for fr in frame_reads]) != 0
        )

        _print_metrics("Predicted", "OpenCV", vx_pred, vx_opencv, vy_pred, vy_opencv, valid_mask)

        _print_metrics("Predicted", "GT", vx_pred, vx_pos, vy_pred, vy_pos, valid_gt_mask)

        _print_metrics("OpenCV", "GT", vx_opencv, vx_pos, vy_opencv, vy_pos, valid_gt_mask)

        if plot_kpi:
            print(f"Plotting velocities!")
            plot_velocities(
                vx_pred, vy_pred, debug_signal, vx_opencv, vy_opencv, vx_pos, vy_pos,
                save_dir=save_dir,
            )

    except Exception as exc:
        print(f"KPI computation failed: {exc}")
