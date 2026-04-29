"""Statistics computation and display for HIL test runs.

This module provides functionality to compute and display various statistics
from a HIL (Hardware-in-the-Loop) test run, including:
- Loop times (write-to-read latency)
- Process elapsed times
- Memory usage (stack and heap)
- Missed frames statistics
"""

import statistics
from typing import List, Optional


def print_statistics(
    elapsed_time: float,
    write_freq_hz: Optional[float],
    frame_write_times: List[float],
    frame_loop_times: List[float],
    process_elapsed_times: List[float],
    peak_stack_memory: float,
    peak_heap_memory: float,
    missed_frames_count: int,
) -> None:
    """Print comprehensive statistics from a HIL test run.

    Parameters
    ----------
    elapsed_time : float
        Total elapsed time in seconds for the test run.
    write_freq_hz : float or None
        Desired write frequency in Hz, or None if unlimited.
    frame_write_times : list of float
        Timestamps when each frame was written to serial port.
    frame_loop_times : list of float
        Write-to-read latency per frame in seconds.
    process_elapsed_times : list of float
        Process time per frame in milliseconds.
    peak_stack_memory : float
        Peak stack memory usage as a fraction (0.0 to 1.0).
    peak_heap_memory : float
        Peak heap memory usage as a fraction (0.0 to 1.0).
    missed_frames_count : int
        Number of frames that were skipped/missed.
    """
    print("")
    print("Statistics")

    print("")
    print("Total elapsed time(s): ", elapsed_time)
    print("Desired freq: ", write_freq_hz)
    if frame_loop_times:
        print("Avg time(ms): ", 1000 * elapsed_time / len(frame_write_times))

        max_loop_time = 1000 * max(frame_loop_times)
        min_loop_time = 1000 * min(frame_loop_times)
        avg_loop_time = 1000 * sum(frame_loop_times) / len(frame_loop_times)
        std_loop_time = 1000 * statistics.stdev(frame_loop_times)

        print("")
        print("max loop time(ms): ", max_loop_time, " f(Hz)= ", 1000 / max_loop_time)
        print("min loop time(ms): ", min_loop_time, " f(Hz)= ", 1000 / min_loop_time)
        print("avg loop time(ms): ", avg_loop_time, " f(Hz)= ", 1000 / avg_loop_time)
        print("std loop time(ms): ", std_loop_time)

    if process_elapsed_times:
        max_process_elapsed_time = max(process_elapsed_times)
        min_process_elapsed_time = min(process_elapsed_times)
        avg_process_elapsed_time = sum(process_elapsed_times) / len(
            process_elapsed_times
        )
        std_process_elapsed_time = statistics.stdev(process_elapsed_times)

        print("")
        print(
            "max process elapsed time(ms): ",
            max_process_elapsed_time,
            " f(Hz)= ",
            1000 / max_process_elapsed_time,
        )
        print(
            "min process elapsed time(ms): ",
            min_process_elapsed_time,
            " f(Hz)= ",
            1000 / min_process_elapsed_time,
        )
        print(
            "avg process elapsed time(ms): ",
            avg_process_elapsed_time,
            " f(Hz)= ",
            1000 / avg_process_elapsed_time,
        )
        print("std process elapsed time(ms): ", std_process_elapsed_time)

    print("")
    print("Peak stack memory usage: ", 100 * peak_stack_memory, "%")
    print("Peak heap memory usage: ", 100 * peak_heap_memory, "%")

    total_sent = len(frame_write_times)
    total_attempted = total_sent + missed_frames_count
    missed_pct = (
        100.0 * missed_frames_count / total_attempted if total_attempted > 0 else 0.0
    )
    print("")
    print("Missed (skipped) frames:  ", missed_frames_count)
    print("Total frames attempted:   ", total_attempted)
    print("Total frames sent:        ", total_sent)
    print(f"Missed frames rate:        {missed_pct:.1f}%")
