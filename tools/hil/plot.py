from typing import TYPE_CHECKING, List, Optional

if TYPE_CHECKING:
    import numpy as np
    from hil.protocol import Coordinate


def plot_timing(
    loop_times: List[float],
    process_elapsed_times: List[float],
    missed_frame_times: List[float],
    start_time: float,
    frame_write_times: List[float],
    frame_deadline_times: List[float],
    write_freq_hz: Optional[float] = None,
) -> None:
    """Display a timeseries plot after a HIL run.

    The plot shows the write-to-read latency and MCU process time as line
    series over wall-clock time.  Individual write events, read events,
    deadlines, and missed frames are overlaid as markers / vertical lines so
    the full picture of when each event occurred is visible at a glance.

    Horizontal grey dashed lines are drawn at every integer multiple of the
    frame period (1 / write_freq_hz) in milliseconds, giving an instant visual
    reference for whether each frame's round-trip fits within a single period.
    The period is derived from ``write_freq_hz`` when supplied, or inferred
    from the spacing of consecutive deadline timestamps as a fallback.

    Layout
    ------
    X axis : time from run start (seconds).
    Y axis : duration (ms).

    Series / markers
    ----------------
    Yellow line  – write-to-read loop time per frame, plotted at each frame's
                   read time (write_time + loop_time).
    Orange line  – MCU process time per frame, also plotted at read time.
    Blue  ▼      – frame write events (y = 0 baseline).
    Cyan  ▲      – frame read events, positioned at write_time + loop_time
                   (y = 0 baseline).
    Green dashes – absolute deadlines for each sent frame.
    Grey dashes  – horizontal lines at multiples of the frame period.
    Red   ×      – missed/skipped frames (semaphore buffer full).

    Args:
        loop_times:            Write-to-read latency per frame in seconds.
        process_elapsed_times: MCU elapsed-time values in milliseconds.
        missed_frame_times:    Absolute timestamps of each missed frame.
        start_time:            Absolute timestamp of the run start.
        frame_write_times:     Absolute write timestamp for each sent frame.
        frame_deadline_times:  Absolute deadline for each sent frame.
        write_freq_hz:         Frame write frequency in Hz.  Used to draw
                               horizontal period-multiple lines.  If None the
                               period is inferred from deadline spacing.
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print(
            "matplotlib is required for --plot.  "
            "Install it with: pip install matplotlib"
        )
        return

    if not loop_times:
        print("Not enough data to generate a timing plot (no loop times recorded).")
        return

    n_sent = len(loop_times)

    # ------------------------------------------------------------------ #
    # Relative timestamps (seconds from run start)                         #
    # ------------------------------------------------------------------ #
    write_rel: List[float] = [t - start_time for t in frame_write_times[:n_sent]]

    # Read time = write time + loop time (write-to-read latency)
    read_rel: List[float] = [
        (frame_write_times[i] - start_time) + loop_times[i] for i in range(n_sent)
    ]

    loop_times_ms: List[float] = [lt * 1000.0 for lt in loop_times]
    process_times_ms: List[float] = list(process_elapsed_times[:n_sent])

    deadline_rel: List[float] = [d - start_time for d in frame_deadline_times]

    # ------------------------------------------------------------------ #
    # Plot                                                                 #
    # ------------------------------------------------------------------ #
    fig, ax = plt.subplots(figsize=(max(14, n_sent * 0.3), 6))

    # --- Loop time line series (plotted at read time = write + loop_time) ---
    ax.plot(
        read_rel,
        loop_times_ms,
        color="gold",
        linewidth=1.2,
        marker="o",
        markersize=3,
        label="Loop time (write→read)",
        zorder=3,
    )

    # --- MCU process time line series (plotted at read time) ---
    if process_times_ms:
        ax.plot(
            read_rel[: len(process_times_ms)],
            process_times_ms,
            color="darkorange",
            linewidth=1.2,
            marker="s",
            markersize=3,
            label="MCU process time",
            zorder=3,
        )

    # --- Deadline vertical dashed lines ---
    for i, d_rel in enumerate(deadline_rel):
        ax.axvline(
            x=d_rel,
            color="limegreen",
            linestyle="--",
            linewidth=0.8,
            alpha=0.7,
            # Only label the first one so the legend stays clean
            label="Deadline" if i == 0 else None,
        )

    # --- Write event markers on the time axis (blue ▼ at y = 0) ---
    ax.scatter(
        write_rel,
        [0.0] * n_sent,
        marker="v",
        color="royalblue",
        s=50,
        zorder=5,
        label="Write event",
    )

    # --- Read event: vertical cyan line + ▲ marker at y = 0 ---
    for i, r_rel in enumerate(read_rel):
        ax.axvline(
            x=r_rel,
            color="cyan",
            linestyle="-",
            linewidth=0.9,
            alpha=0.8,
            zorder=4,
            label="Read event" if i == 0 else None,
        )
    ax.scatter(
        read_rel,
        [0.0] * n_sent,
        marker="^",
        color="cyan",
        s=60,
        zorder=6,
    )

    # --- Missed frame: vertical red line + × marker at y = 0 ---
    if missed_frame_times:
        missed_rel = [t - start_time for t in missed_frame_times]
        for i, m_rel in enumerate(missed_rel):
            ax.axvline(
                x=m_rel,
                color="red",
                linestyle="-",
                linewidth=0.9,
                alpha=0.8,
                zorder=4,
                label="Missed frame" if i == 0 else None,
            )
        ax.scatter(
            missed_rel,
            [0.0] * len(missed_rel),
            marker="x",
            color="red",
            s=120,
            linewidths=2,
            zorder=6,
        )

    # --- Horizontal lines at every multiple of the frame period ---
    # Determine period_ms from write_freq_hz or from deadline spacing.
    period_ms: Optional[float] = None
    if write_freq_hz is not None and write_freq_hz > 0:
        period_ms = 1000.0 / write_freq_hz
    elif len(frame_deadline_times) >= 2:
        period_ms = (frame_deadline_times[1] - frame_deadline_times[0]) * 1000.0

    if period_ms is not None and period_ms > 0:
        all_durations = loop_times_ms + process_times_ms
        y_max = max(all_durations) * 1.2 if all_durations else period_ms * 3
        multiple = 1
        while multiple * period_ms <= y_max:
            ax.axhline(
                y=multiple * period_ms,
                color="grey",
                linestyle="--",
                linewidth=0.7,
                alpha=0.6,
                label=f"Period × {multiple}" if multiple == 1 else None,
            )
            multiple += 1

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Duration (ms)")
    ax.set_title("HIL Frame Timing — Timeseries")
    ax.set_ylim(bottom=0)

    ax.legend(loc="upper right")
    plt.tight_layout()
    plt.show()


def plot_predictions(
    frame_numbers: List[int],
    tx_pred: List[float],
    ty_pred: List[float],
    theta_pred: List[float],
    tx_gt: "np.ndarray",
    ty_gt: "np.ndarray",
    theta_gt: "np.ndarray",
) -> None:
    """Plot predicted vs. ground-truth tx, ty and theta over time.

    Three vertically-stacked subplots share the same x-axis (frame index):

    * **tx** (East translation, pixels)  — top panel.
    * **ty** (North translation, pixels) — middle panel.
    * **theta** (rotation, radians)      — bottom panel.

    Each panel shows:

    * Solid coloured line + circle markers — STM32 predictions.
    * Dashed grey line + cross markers     — ground-truth values (NaN entries,
      e.g. the last frame, are silently omitted).

    Args:
        frame_numbers: Dataset query-image index for each prediction.  Used as
            the x-axis ("time") coordinate.
        tx_pred:    Predicted East translation per frame (pixels).
        ty_pred:    Predicted North translation per frame (pixels).
        theta_pred: Predicted rotation per frame (radians).
        tx_gt:      Ground-truth tx array indexed by dataset frame index.
                    May contain NaN for frames without a successor.
        ty_gt:      Ground-truth ty array (same shape / indexing as *tx_gt*).
        theta_gt:   Ground-truth theta array (same shape / indexing as *tx_gt*).
    """
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print(
            "matplotlib and numpy are required for --plot-kpi.  "
            "Install them with: pip install matplotlib numpy"
        )
        return

    if not frame_numbers:
        print("Not enough data to generate a prediction plot (no frames recorded).")
        return

    gt_len = len(tx_gt)

    # ------------------------------------------------------------------ #
    # Build aligned ground-truth series (only frames with valid gt)       #
    # ------------------------------------------------------------------ #
    gt_frames: List[int] = []
    gt_tx: List[float] = []
    gt_ty: List[float] = []
    gt_theta: List[float] = []
    for fi in frame_numbers:
        if fi < gt_len and not np.isnan(tx_gt[fi]):
            gt_frames.append(fi)
            gt_tx.append(float(tx_gt[fi]))
            gt_ty.append(float(ty_gt[fi]))
            gt_theta.append(float(theta_gt[fi]))

    # ------------------------------------------------------------------ #
    # Plot                                                                 #
    # ------------------------------------------------------------------ #
    fig, axes = plt.subplots(
        3, 1, figsize=(max(12, len(frame_numbers) * 0.25), 10), sharex=True
    )

    _PRED_STYLE = dict(linewidth=1.4, marker="o", markersize=3, zorder=3)
    _GT_STYLE = dict(
        linewidth=1.2,
        linestyle="--",
        marker="x",
        markersize=4,
        color="dimgrey",
        zorder=2,
    )

    # ---- tx ----
    axes[0].plot(
        frame_numbers, tx_pred, color="steelblue", label="tx predicted", **_PRED_STYLE
    )
    if gt_frames:
        axes[0].plot(gt_frames, gt_tx, label="tx ground truth", **_GT_STYLE)
    axes[0].set_ylabel("tx (px)")
    axes[0].set_title("East translation (tx) — predicted vs. ground truth")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, linestyle=":", alpha=0.5)

    # ---- ty ----
    axes[1].plot(
        frame_numbers, ty_pred, color="tomato", label="ty predicted", **_PRED_STYLE
    )
    if gt_frames:
        axes[1].plot(gt_frames, gt_ty, label="ty ground truth", **_GT_STYLE)
    axes[1].set_ylabel("ty (px)")
    axes[1].set_title("North translation (ty) — predicted vs. ground truth")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, linestyle=":", alpha=0.5)

    # ---- theta ----
    axes[2].plot(
        frame_numbers,
        theta_pred,
        color="mediumseagreen",
        label="θ predicted",
        **_PRED_STYLE,
    )
    if gt_frames:
        axes[2].plot(gt_frames, gt_theta, label="θ ground truth", **_GT_STYLE)
    axes[2].set_ylabel("theta (rad)")
    axes[2].set_xlabel("Frame index")
    axes[2].set_title("Rotation (theta) — predicted vs. ground truth")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, linestyle=":", alpha=0.5)

    fig.suptitle(
        "HIL Predictions vs. Ground Truth — Timeseries", fontsize=13, fontweight="bold"
    )
    plt.tight_layout()
    plt.show()
