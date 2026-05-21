"""KPI: accuracy metrics for HIL velocity estimates vs. ground truth.

The STM32 firmware receives one 96×96 grayscale query image per frame and
outputs vx, vy (m/s) and omega (rad/s) estimates.  Ground-truth velocities
are pre-computed during streaming (see :class:`CsvDatasetStreamer.get_ground_truth`)
and attached to each :class:`FrameReadEvent`.  This module simply reads
that data and computes summary metrics.

Metrics reported (per axis)
---------------------------
MAE   Mean Absolute Error
RMSE  Root Mean Square Error
"""

from __future__ import annotations

from typing import List
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
        winsize=15,  # averaging window size
        iterations=3,  # iterations per pyramid level
        poly_n=5,  # pixel neighborhood size
        poly_sigma=1.2,  # Gaussian std for polynomial expansion
        flags=0,
    )
    # flow shape: (H, W, 2) — [:,:,0] = vx, [:,:,1] = vy
    avg_vx = float(np.mean(flow[:, :, 0]))
    avg_vy = float(np.mean(flow[:, :, 1]))

    return avg_vx, avg_vy, flow


def compute_and_print_kpi(
    frame_reads: List[FrameReadEvent],
    plot_kpi: bool = False,
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
                    pvx * frame_reads[i].pz / (frame_reads[i].dt * frame_reads[i].fx)
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

        vx_mae = float(np.mean(np.abs(vx_pred[valid_mask] - vx_opencv[valid_mask])))
        vy_mae = float(np.mean(np.abs(vy_pred[valid_mask] - vy_opencv[valid_mask])))
        # omega_mae = float(
        #     np.mean(np.abs(omega_pred[valid_mask] - omega_gt[valid_mask]))
        # )

        vx_rmse = float(
            np.sqrt(np.mean((vx_pred[valid_mask] - vx_opencv[valid_mask]) ** 2))
        )
        vy_rmse = float(
            np.sqrt(np.mean((vy_pred[valid_mask] - vy_opencv[valid_mask]) ** 2))
        )
        # omega_rmse = float(
        #     np.sqrt(np.mean((omega_pred[valid_mask] - omega_gt[valid_mask]) ** 2))
        # )

        print(f"  Velocity MAE  — vx: {vx_mae:.4f} m/s,  vy: {vy_mae:.4f} m/s")
        print(f"  Velocity RMSE — vx: {vx_rmse:.4f} m/s,  vy: {vy_rmse:.4f} m/s")

        if plot_kpi:
            print(f"Plotting velocities!")
            plot_velocities(
                vx_pred, vy_pred, debug_signal, vx_opencv, vy_opencv, vx_pos, vy_pos
            )

    except Exception as exc:
        print(f"KPI computation failed: {exc}")
