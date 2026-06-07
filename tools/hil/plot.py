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
    save_dir: Optional[str] = None,
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

    if save_dir is not None:
        import os

        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, "frame_metrics.png")
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Saved plot: {save_path}")

    plt.show()


def plot_velocities(
    vx_pred: np.ndarray,
    vy_pred: np.ndarray,
    helper_var: np.ndarray,
    vx_opencv: np.ndarray,
    vy_opencv: np.ndarray,
    vx_pos: np.ndarray,
    vy_pos: np.ndarray,
    save_dir: Optional[str] = None,
) -> None:
    """
    Plot velocity estimates from three sources in 3 side-by-side subplots (top row):
      - vx_pred / vy_pred : STM32 firmware predictions
      - vx_opencv / vy_opencv : OpenCV Farneback optical-flow ground truth
      - vx_pos / vy_pos : position-derived ground truth (Δpos / Δt)

    A second row shows the prediction errors (predicted − reference) for each
    velocity axis using both reference signals.
    """
    fig = plt.figure(figsize=(15, 8))
    fig.suptitle("Velocity estimates vs ground truth", fontsize=14, fontweight="bold")

    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45)

    ax_vx = fig.add_subplot(gs[0, 0])
    ax_vy = fig.add_subplot(gs[0, 1])
    ax_helper = fig.add_subplot(gs[0, 2])
    ax_vx_err = fig.add_subplot(gs[1, 0])
    ax_vy_err = fig.add_subplot(gs[1, 1])

    # ── top row: velocity traces (predicted + opencv GT + position GT) ────────
    for ax, pred, gt_cv, gt_pos, title, unit in [
        (ax_vx, vx_pred, vx_opencv, vx_pos, "vx", "m/s"),
        (ax_vy, vy_pred, vy_opencv, vy_pos, "vy", "m/s"),
    ]:
        ax.plot(pred, color="#4C9BE8", linewidth=1.5, label="predicted")
        ax.plot(
            gt_cv,
            color="#E84C4C",
            linewidth=1.5,
            linestyle="--",
            label="opencv",
        )
        ax.plot(
            gt_pos,
            color="#4CE87A",
            linewidth=1.5,
            linestyle=":",
            label="from position",
        )
        ax.set_title(title)
        ax.set_xlabel("Frame index")
        ax.set_ylabel(unit)
        ax.legend(fontsize=8)
        ax.grid(True, linestyle="--", alpha=0.4)

    ax_helper.plot(helper_var, color="#E8A84C", linewidth=1.5, label="helper")
    ax_helper.set_title("helper")
    ax_helper.set_xlabel("Frame index")
    ax_helper.set_ylabel("")
    ax_helper.legend(fontsize=8)
    ax_helper.grid(True, linestyle="--", alpha=0.4)

    # ── bottom row: prediction error traces ───────────────────────────────────
    err_specs = [
        (ax_vx_err, vx_pred - vx_opencv, vx_pred - vx_pos, "vx error (pred − ref)"),
        (ax_vy_err, vy_pred - vy_opencv, vy_pred - vy_pos, "vy error (pred − ref)"),
    ]

    for ax, err_cv, err_pos, title in err_specs:
        ax.plot(err_cv, color="#E84C4C", linewidth=1.5, label="pred − opencv")
        ax.plot(
            err_pos, color="#4CE87A", linewidth=1.5, linestyle=":", label="pred − pos"
        )
        ax.axhline(0, color="white", linewidth=0.8, linestyle="--", alpha=0.6)
        ax.set_title(title)
        ax.set_xlabel("Frame index")
        ax.set_ylabel("m/s")
        ax.legend(fontsize=8)
        ax.grid(True, linestyle="--", alpha=0.4)

    plt.tight_layout()

    if save_dir is not None:
        import os

        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, "velocities.png")
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Saved plot: {save_path}")

    plt.show()
