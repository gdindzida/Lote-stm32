from typing import TYPE_CHECKING, List, Optional
from hil.frames import FrameWriteEvent, FrameReadEvent
import numpy as np

if TYPE_CHECKING:
    import numpy as np
    from hil.protocol import Coordinate

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def plot_frame_metrics(
    writes: List[FrameWriteEvent],
    reads: List[FrameReadEvent],
) -> None:
    """
    Single plot with:
      - write→read latency (ms) and process elapsed time (ms) on y-axis
      - read timestamps on x-axis
      - vertical lines for write times, deadlines, and read times
    """
    n = min(len(writes), len(reads))
    paired_writes = writes[:n]
    paired_reads = reads[:n]

    write_times = np.array([w.write_time for w in paired_writes])
    deadlines = np.array([w.deadline for w in paired_writes])
    read_times = np.array([r.read_time for r in paired_reads])

    write_to_read_ms = (read_times - write_times) * 1000.0
    process_elapsed_ms = np.array(
        [r.meta.process_elapsed_time_ms for r in paired_reads]
    )

    fig, ax = plt.subplots(figsize=(14, 6))
    fig.suptitle("Frame timing metrics", fontsize=14, fontweight="bold")

    # ── metric lines ──────────────────────────────────────────────────────
    ax.plot(
        read_times,
        write_to_read_ms,
        color="#4C9BE8",
        linewidth=1.5,
        marker="o",
        markersize=3,
        label="write→read latency (ms)",
        zorder=3,
    )
    ax.plot(
        read_times,
        process_elapsed_ms,
        color="#E8A84C",
        linewidth=1.5,
        marker="o",
        markersize=3,
        label="process elapsed (ms)",
        zorder=3,
    )

    # ── vertical event lines — one legend entry each via label on first only ─
    for i, wt in enumerate(write_times):
        ax.axvline(
            wt,
            color="#4CE87A",
            alpha=0.3,
            linewidth=1,
            label="write time" if i == 0 else "_nolegend_",
        )

    for i, dl in enumerate(deadlines):
        ax.axvline(
            dl,
            color="#E84C4C",
            alpha=0.3,
            linewidth=0.8,
            label="deadline" if i == 0 else "_nolegend_",
        )

    for i, rt in enumerate(read_times):
        ax.axvline(
            rt,
            color="#A64CE8",
            alpha=0.3,
            linewidth=0.8,
            label="read time" if i == 0 else "_nolegend_",
        )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Time (ms)")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, linestyle="--", alpha=0.4)

    plt.tight_layout()
    plt.show()


def plot_velocities(
    vx: np.ndarray,
    vy: np.ndarray,
    omega: np.ndarray,
    vx_gt: np.ndarray,
    vy_gt: np.ndarray,
    omega_gt: np.ndarray,
) -> None:
    """
    Plot estimated vs ground-truth velocities in 3 side-by-side subplots.
    """
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    fig.suptitle("Velocity estimates vs ground truth", fontsize=14, fontweight="bold")

    specs = [
        (axes[0], vx, vx_gt, "vx", "m/s"),
        (axes[1], vy, vy_gt, "vy", "m/s"),
        (axes[2], omega, omega_gt, "omega", "rad/s"),
    ]

    for ax, est, gt, title, unit in specs:
        ax.plot(est, color="#4C9BE8", linewidth=1.5, label="estimated")
        ax.plot(
            gt, color="#E84C4C", linewidth=1.5, label="ground truth", linestyle="--"
        )
        ax.set_title(title)
        ax.set_xlabel("Frame index")
        ax.set_ylabel(unit)
        ax.legend(fontsize=8)
        ax.grid(True, linestyle="--", alpha=0.4)

    plt.tight_layout()
    plt.show()


def plot_velocity_xy(
    frame_numbers: List[int],
    vx_kf: "np.ndarray",
    vy_kf: "np.ndarray",
    vx_gt: "np.ndarray",
    vy_gt: "np.ndarray",
) -> None:
    """Plot Kalman-filtered velocity estimates vs. ground-truth velocity.

    Two vertically-stacked subplots share the same x-axis (frame index):

    * **Top**    — vx (East / horizontal velocity, m/frame).
    * **Bottom** — vy (North / vertical velocity, m/frame).

    Each panel shows:

    * Solid line  — KF velocity estimate.
    * Dashed grey — Ground-truth velocity (NaN entries silently omitted).

    A summary MAE for each axis is annotated in the figure title.

    Args:
        frame_numbers: Dataset frame indices used as the x-axis.
        vx_kf, vy_kf: Kalman-filtered velocity estimates in m/frame.
        vx_gt, vy_gt: Ground-truth velocity in m/frame (may contain NaN).
    """
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print(
            "matplotlib and numpy are required for --plot-kpi.  "
            "Install with: pip install matplotlib numpy"
        )
        return

    if not frame_numbers:
        print("Not enough data for velocity plot (no frames recorded).")
        return

    _KF_VX_COLOR = "steelblue"
    _KF_VY_COLOR = "tomato"
    _GT_COLOR = "dimgrey"

    fig, (ax_vx, ax_vy) = plt.subplots(
        2, 1, figsize=(max(12, len(frame_numbers) * 0.25), 8), sharex=True
    )

    _PRED_STYLE = dict(linewidth=1.4, marker="o", markersize=3, zorder=3)
    _GT_STYLE = dict(
        linewidth=1.2,
        linestyle="--",
        marker="x",
        markersize=4,
        color=_GT_COLOR,
        zorder=2,
    )

    # --- Valid GT mask (non-NaN entries) ---
    valid_mask = ~np.isnan(vx_gt)
    gt_frames = [frame_numbers[i] for i in range(len(frame_numbers)) if valid_mask[i]]
    gt_vx = vx_gt[valid_mask]
    gt_vy = vy_gt[valid_mask]

    # ---- vx (East) ----
    ax_vx.plot(
        frame_numbers, vx_kf, color=_KF_VX_COLOR, label="vx KF estimate", **_PRED_STYLE
    )
    if gt_frames:
        ax_vx.plot(gt_frames, gt_vx, label="vx ground truth", **_GT_STYLE)
    ax_vx.set_ylabel("vx (m/frame)")
    ax_vx.set_title("East velocity (vx) — KF estimate vs. ground truth")
    ax_vx.legend(loc="upper right")
    ax_vx.grid(True, linestyle=":", alpha=0.5)

    # ---- vy (North) ----
    ax_vy.plot(
        frame_numbers, vy_kf, color=_KF_VY_COLOR, label="vy KF estimate", **_PRED_STYLE
    )
    if gt_frames:
        ax_vy.plot(gt_frames, gt_vy, label="vy ground truth", **_GT_STYLE)
    ax_vy.set_ylabel("vy (m/frame)")
    ax_vy.set_xlabel("Frame index")
    ax_vy.set_title("North velocity (vy) — KF estimate vs. ground truth")
    ax_vy.legend(loc="upper right")
    ax_vy.grid(True, linestyle=":", alpha=0.5)

    # Annotate summary MAE in the suptitle (only when GT is available).
    if len(gt_frames) > 0:
        vx_mae = float(np.mean(np.abs(vx_kf[valid_mask] - gt_vx)))
        vy_mae = float(np.mean(np.abs(vy_kf[valid_mask] - gt_vy)))
        title = (
            "Kalman Filter Velocity Estimate vs. Ground Truth  "
            f"[vx MAE = {vx_mae:.4f} m/frame | vy MAE = {vy_mae:.4f} m/frame]"
        )
    else:
        title = "Kalman Filter Velocity Estimate vs. Ground Truth"

    fig.suptitle(title, fontsize=12, fontweight="bold")
    plt.tight_layout()
    plt.show()
