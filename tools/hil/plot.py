from typing import List


def plot_timing(
    loop_times: List[float],
    process_elapsed_times: List[float],
    missed_frame_times: List[float],
    start_time: float,
) -> None:
    """Display a per-frame timing bar chart after a HIL run.

    For each sent frame two narrow side-by-side bars are drawn:
      - Yellow  : loop time (write-to-write interval, ms)
      - Orange  : MCU process time (ms)

    Missed frames are marked with a red × at the corresponding timeline
    position.

    Args:
        loop_times:            Write-to-write intervals in seconds.
        process_elapsed_times: MCU elapsed-time values in milliseconds.
        missed_frame_times:    Absolute timestamps (time.time()) of each
                               missed/skipped frame.
        start_time:            Absolute timestamp of the run start, used to
                               convert missed-frame times to relative seconds.
    """
    try:
        import matplotlib.patches as mpatches
        import matplotlib.lines as mlines
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

    # ------------------------------------------------------------------ #
    # Derived data                                                         #
    # ------------------------------------------------------------------ #
    n_sent = len(loop_times) + 1  # first frame + one per loop_times entry

    # Cumulative send times in seconds from run start
    sent_times_s: List[float] = [0.0]
    for lt in loop_times:
        sent_times_s.append(sent_times_s[-1] + lt)

    # Y values in ms
    loop_times_ms = [lt * 1000.0 for lt in loop_times]
    process_times_ms = list(process_elapsed_times)

    # Bar width: 30 % of the average loop interval per bar, so the pair
    # occupies ~62 % of the slot (two bars + a 2 % gap between them).
    avg_loop_s = sum(loop_times) / len(loop_times)
    bar_w = avg_loop_s * 0.30
    gap_w = bar_w * 0.07  # small gap between the two bars in a pair

    # ------------------------------------------------------------------ #
    # Plot                                                                 #
    # ------------------------------------------------------------------ #
    fig, ax = plt.subplots(figsize=(max(12, n_sent * 0.25), 6))

    for i in range(n_sent):
        x = sent_times_s[i]

        # Yellow bar: loop time
        h_loop = loop_times_ms[i] if i < len(loop_times_ms) else 0.0
        if h_loop > 0:
            ax.bar(
                x,
                h_loop,
                width=bar_w,
                align="edge",
                color="yellow",
                edgecolor="goldenrod",
                linewidth=0.5,
            )

        # Orange bar: process time (placed immediately to the right)
        if i < len(process_times_ms):
            h_proc = process_times_ms[i]
            if h_proc > 0:
                ax.bar(
                    x + bar_w + gap_w,
                    h_proc,
                    width=bar_w,
                    align="edge",
                    color="darkorange",
                    edgecolor="saddlebrown",
                    linewidth=0.5,
                )

    # Red × markers for missed frames (placed on the bottom x axis).
    # A blended transform (x=data coords, y=axes coords) is used so that
    # y=0 means "the bottom spine" regardless of the data range, keeping
    # the y-axis scale and tick labels unaffected.
    if missed_frame_times:
        from matplotlib.transforms import blended_transform_factory

        missed_rel_s = [t - start_time for t in missed_frame_times]
        trans = blended_transform_factory(ax.transData, ax.transAxes)
        ax.scatter(
            missed_rel_s,
            [0] * len(missed_rel_s),
            marker="x",
            color="red",
            s=120,
            linewidths=2,
            zorder=5,
            clip_on=False,
            transform=trans,
        )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Duration (ms)")
    ax.set_title("HIL Frame Timing")

    legend_handles = [
        mpatches.Patch(facecolor="yellow", edgecolor="goldenrod", label="Loop time"),
        mpatches.Patch(
            facecolor="darkorange", edgecolor="saddlebrown", label="Process time"
        ),
    ]
    if missed_frame_times:
        legend_handles.append(
            mlines.Line2D(
                [],
                [],
                marker="x",
                color="red",
                linestyle="None",
                markersize=10,
                markeredgewidth=2,
                label="Missed frame",
            )
        )

    ax.legend(handles=legend_handles)
    plt.tight_layout()
    plt.show()
