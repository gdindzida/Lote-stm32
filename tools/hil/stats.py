"""Statistics computation and display for HIL test runs.

This module provides functionality to compute and display various statistics
from a HIL (Hardware-in-the-Loop) test run, including:
- Loop times (write-to-read latency)
- Process elapsed times
- Memory usage (stack and heap)
- Missed frames statistics
"""

from typing import List
from hil.frames import FrameWriteEvent, FrameReadEvent
import numpy as np


def compute_and_print_metrics(
    writes: List[FrameWriteEvent],
    reads: List[FrameReadEvent],
):
    """
    Compute timing and memory metrics from paired write/read events.

    Pairing: first write -> first read, etc. Excess writes are ignored.
    All times are converted from seconds to milliseconds.

    Returns a dict with the computed arrays for further use.
    """
    n = min(len(writes), len(reads))
    paired_writes = writes[:n]
    paired_reads = reads[:n]

    # --- Timing arrays (ms) ---
    write_to_read_ms = np.array(
        [
            (r.read_time - w.write_time) * 1000.0
            for w, r in zip(paired_writes, paired_reads)
        ]
    )

    process_elapsed_ms = np.array(
        [
            r.meta.process_elapsed_time_ms  # already in ms per dataclass name
            for r in paired_reads
        ]
    )

    stride_elapsed_ms = np.array(
        [
            r.meta.stride_elapsed_tim_ms  # already in ms per dataclass name
            for r in paired_reads
        ]
    )

    def _print_stats(arr: np.ndarray) -> None:
        arr_mean = arr.mean()
        arr_max = arr.max()
        arr_min = arr.min()
        arr_std = arr.std()
        print(f"  mean : {arr_mean:.3f} ms", " f(Hz)= ", 1000 / arr_mean)
        print(f"  min  : {arr_min:.3f} ms", " f(Hz)= ", 1000 / arr_min)
        print(f"  max  : {arr_max:.3f} ms", " f(Hz)= ", 1000 / arr_max)
        print(f"  std  : {arr_std:.3f} ms")

    print("=" * 50)
    print("Write -> Read latency")
    _print_stats(write_to_read_ms)

    print()
    print("Process elapsed time")
    _print_stats(process_elapsed_ms)

    print()
    print("Stride elapsed time")
    _print_stats(stride_elapsed_ms)

    # --- Memory ---
    max_stack = max(r.meta.stack_mem_usage for r in paired_reads)
    max_heap = max(r.meta.heap_mem_usage for r in paired_reads)
    print()
    print("Memory")
    print(f"  max stack : {max_stack * 100:.3f}%")
    print(f"  max heap  : {max_heap * 100:.3f}%")

    # --- Missed frames (from the full write list) ---
    total_frames = len(writes)
    missed_frames = sum(1 for w in writes if w.missed)
    missed_rate = missed_frames / total_frames if total_frames > 0 else 0.0
    print()
    print("Frame statistics")
    print(f"  total frames  : {total_frames}")
    print(f"  missed frames : {missed_frames}")
    print(f"  missed rate   : {missed_rate * 100:.2f} %")
    print("=" * 50)
