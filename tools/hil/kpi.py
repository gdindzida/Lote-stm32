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

from hil.frames import FrameReadEvent
from hil.plot import plot_velocities

try:
    import numpy as np
except ImportError as exc:
    raise ImportError(
        "numpy is required for KPI computation. Install it with: pip install numpy"
    ) from exc


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
        helper_pred = np.array([fr.meta.omega for fr in frame_reads], dtype=np.float64)

        vx_gt = np.zeros(len(frame_reads))
        vy_gt = np.zeros(len(frame_reads))
        for i, _ in enumerate(frame_reads):
            if i != 0:
                vx_gt[i] = (frame_reads[i].px - frame_reads[i - 1].px) / frame_reads[
                    i
                ].dt
                vy_gt[i] = (frame_reads[i].py - frame_reads[i - 1].py) / frame_reads[
                    i
                ].dt
            else:
                vx_gt[0] = 0
                vy_gt[0] = 0

        # omega_gt = np.array(
        #     [fr.ground_truth.omega for fr in frames_with_gt], dtype=np.float64  # type: ignore[union-attr]
        # )

        # Only use frames where GT velocity is valid (non-NaN)
        valid_mask = ~np.isnan(vx_gt)
        if not valid_mask.any():
            print("  All ground-truth velocities are NaN — skipping KPI.")
            return

        vx_mae = float(np.mean(np.abs(vx_pred[valid_mask] - vx_gt[valid_mask])))
        vy_mae = float(np.mean(np.abs(vy_pred[valid_mask] - vy_gt[valid_mask])))
        # omega_mae = float(
        #     np.mean(np.abs(omega_pred[valid_mask] - omega_gt[valid_mask]))
        # )

        vx_rmse = float(
            np.sqrt(np.mean((vx_pred[valid_mask] - vx_gt[valid_mask]) ** 2))
        )
        vy_rmse = float(
            np.sqrt(np.mean((vy_pred[valid_mask] - vy_gt[valid_mask]) ** 2))
        )
        # omega_rmse = float(
        #     np.sqrt(np.mean((omega_pred[valid_mask] - omega_gt[valid_mask]) ** 2))
        # )

        print(f"  Velocity MAE  — vx: {vx_mae:.4f} m/s,  vy: {vy_mae:.4f} m/s")
        print(f"  Velocity RMSE — vx: {vx_rmse:.4f} m/s,  vy: {vy_rmse:.4f} m/s")

        if plot_kpi:
            print(f"Plotting velocities!")
            plot_velocities(vx_pred, vy_pred, helper_pred, vx_gt, vy_gt)

    except Exception as exc:
        print(f"KPI computation failed: {exc}")
