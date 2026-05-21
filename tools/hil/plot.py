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
    helper_var: np.ndarray,
    vx_gt: np.ndarray,
    vy_gt: np.ndarray,
) -> None:
    """
    Plot estimated vs ground-truth velocities in 3 side-by-side subplots (top row),
    with error (predicted − GT) for vx and vy in a second row below them.
    """
    fig = plt.figure(figsize=(15, 8))
    fig.suptitle("Velocity estimates vs ground truth", fontsize=14, fontweight="bold")

    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45)

    ax_vx = fig.add_subplot(gs[0, 0])
    ax_vy = fig.add_subplot(gs[0, 1])
    ax_helper = fig.add_subplot(gs[0, 2])
    ax_vx_err = fig.add_subplot(gs[1, 0])
    ax_vy_err = fig.add_subplot(gs[1, 1])

    # ── top row: velocity traces ──────────────────────────────────────────────
    vel_specs = [
        (ax_vx, vx, vx_gt, "vx", "m/s"),
        (ax_vy, vy, vy_gt, "vy", "m/s"),
        (ax_helper, helper_var, None, "helper", ""),
    ]

    for ax, est, gt, title, unit in vel_specs:
        ax.plot(est, color="#4C9BE8", linewidth=1.5, label="estimated")
        if gt is not None:
            ax.plot(
                gt, color="#E84C4C", linewidth=1.5, label="ground truth", linestyle="--"
            )
        ax.set_title(title)
        ax.set_xlabel("Frame index")
        ax.set_ylabel(unit)
        ax.legend(fontsize=8)
        ax.grid(True, linestyle="--", alpha=0.4)

    # ── bottom row: prediction error traces ───────────────────────────────────
    err_specs = [
        (ax_vx_err, vx - vx_gt, "vx error (pred − GT)"),
        (ax_vy_err, vy - vy_gt, "vy error (pred − GT)"),
    ]

    for ax, err, title in err_specs:
        ax.plot(err, color="#E8A84C", linewidth=1.5)
        ax.axhline(0, color="white", linewidth=0.8, linestyle="--", alpha=0.6)
        ax.set_title(title)
        ax.set_xlabel("Frame index")
        ax.set_ylabel("m/s")
        ax.grid(True, linestyle="--", alpha=0.4)

    plt.tight_layout()
    plt.show()
