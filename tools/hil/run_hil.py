import os
import queue
import statistics
import struct
import sys
import threading
import time
import argparse
from typing import List, Optional

os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
import cv2
import serial

from hil.frames import FrameItem, FrameRecord, IMG_SCALE_SIZE
from hil.uav import UAVStreamer
from hil.protocol import COORD_FMT
from hil.stm32 import find_stm32_port
from hil.streamer import DatasetStreamer
from hil.threads import reader_thread_fn, writer_thread_fn
from hil.plot import plot_timing

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Run HIL test with optional playback.")
    playback_group = parser.add_mutually_exclusive_group()
    playback_group.add_argument(
        "--playback",
        type=int,
        metavar="DELAY_MS",
        default=None,
        help="Replay recorded frames after statistics with a fixed delay in ms between frames.",
    )
    playback_group.add_argument(
        "--playback-realtime",
        action="store_true",
        default=False,
        help="Replay recorded frames after statistics using the original inter-frame timings.",
    )
    parser.add_argument(
        "--write-freq",
        type=float,
        metavar="HZ",
        default=30,
        help="Frequency (in Hz) at which frames are written to the serial port. "
        "Omit for maximum throughput (no throttling).",
    )
    parser.add_argument(
        "--data-root",
        type=str,
        metavar="PATH",
        required=True,
        help="Path to a UAV split folder (e.g. /path/to/alto-dataset/UAV/Train).",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        default=False,
        help="After the run, display a timing bar chart (loop time / process time per frame, "
        "with markers for missed frames).",
    )
    args = parser.parse_args()

    do_record = args.playback is not None or args.playback_realtime
    write_freq_hz: Optional[float] = args.write_freq

    if write_freq_hz is not None and write_freq_hz <= 0:
        parser.error("--write-freq must be a positive number")

    data_root: str = args.data_root
    print("Entering ", data_root)

    streamer: DatasetStreamer = UAVStreamer(data_root, None)

    port = find_stm32_port()
    ser = serial.Serial(port, timeout=1)
    print(f"Connected to {port}")

    if write_freq_hz is not None:
        print(
            f"Write frequency: {write_freq_hz} Hz (period: {1000.0 / write_freq_hz:.1f} ms)"
        )
    else:
        print("Write frequency: unlimited (max throughput)")

    frame_write_times: List[float] = []
    process_elapsed_times: List[float] = []
    peak_memory: List[float] = [0.0, 0.0]  # [stack, heap]
    recorded_frames: List[FrameRecord] = []
    missed_frames: List[int] = [0]  # [missed_frame_count]
    missed_frame_times: List[float] = []  # absolute timestamps of each missed frame

    print("Starting UAV clip playback...")
    print("Press 'q' + Enter at any time to stop streaming early.")

    frame_queue: "queue.Queue[Optional[FrameItem]]" = queue.Queue(maxsize=0)
    # Models the STM32's 2-frame receive buffer.  The writer acquires a slot
    # before each transmission; the reader releases it only after the MCU
    # serial response has been fully received.
    frame_buffer_sem = threading.Semaphore(2)
    error_event = threading.Event()
    stop_event = threading.Event()

    start_time = time.time()

    def keyboard_listener_fn() -> None:
        """Sets stop_event when the user types 'q' + Enter."""
        try:
            while not stop_event.is_set() and not error_event.is_set():
                line = sys.stdin.readline()
                if not line:  # EOF (e.g. piped input ended)
                    break
                if line.strip().lower() == "q":
                    print("\nStopping early (q pressed)...")
                    stop_event.set()
                    break
        except OSError:
            pass

    writer = threading.Thread(
        target=writer_thread_fn,
        name="ser-writer",
        args=(
            ser,
            streamer,
            frame_queue,
            write_freq_hz,
            do_record,
            frame_write_times,
            missed_frames,
            missed_frame_times,
            frame_buffer_sem,
            error_event,
            stop_event,
        ),
        daemon=True,
    )

    reader = threading.Thread(
        target=reader_thread_fn,
        name="ser-reader",
        args=(
            ser,
            frame_queue,
            do_record,
            process_elapsed_times,
            recorded_frames,
            peak_memory,
            frame_buffer_sem,
            error_event,
            streamer.total,
        ),
        daemon=True,
    )

    keyboard = threading.Thread(
        target=keyboard_listener_fn,
        name="kbd-listener",
        daemon=True,
    )

    writer.start()
    reader.start()
    keyboard.start()

    writer.join()
    reader.join()
    stop_event.set()  # unblock keyboard listener if stream finished naturally

    # Derive write-to-write intervals from the absolute timestamps collected by
    # the writer.  This avoids any timing measurement inside the hot loop.
    loop_times = [t2 - t1 for t1, t2 in zip(frame_write_times, frame_write_times[1:])]

    if error_event.is_set():
        print("An error occurred during serial communication. Aborting.")
        raise SystemExit(1)

    peak_stack_memory = peak_memory[0]
    peak_heap_memory = peak_memory[1]

    print("")
    print("Statistics")

    elapsed_time = time.time() - start_time
    print("")
    print("Total elapsed time(s): ", elapsed_time)
    print("Desired freq: ", write_freq_hz)
    if loop_times:
        print("Avg time(ms): ", 1000 * elapsed_time / len(frame_write_times))

        max_loop_time = 1000 * max(loop_times)
        min_loop_time = 1000 * min(loop_times)
        avg_loop_time = 1000 * sum(loop_times) / len(loop_times)
        std_loop_time = 1000 * statistics.stdev(loop_times)

        print("")
        print("max loop time(ms): ", max_loop_time, " f(Hz)= ", 1000 / max_loop_time)
        print("min loop time(ms): ", min_loop_time, " f(Hz)= ", 1000 / min_loop_time)
        print("avg loop time(ms): ", avg_loop_time, " f(Hz)= ", 1000 / avg_loop_time)
        print("std loop time(ms): ", std_loop_time)

    if process_elapsed_times:
        max_process_elapsed_time = 0.001 * max(process_elapsed_times)
        min_process_elapsed_time = 0.001 * min(process_elapsed_times)
        avg_process_elapsed_time = (
            0.001 * sum(process_elapsed_times) / len(process_elapsed_times)
        )
        std_process_elapsed_time = 0.001 * statistics.stdev(process_elapsed_times)

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
    total_attempted = total_sent + missed_frames[0]
    missed_pct = (
        100.0 * missed_frames[0] / total_attempted if total_attempted > 0 else 0.0
    )
    print("")
    print("Missed (skipped) frames:  ", missed_frames[0])
    print("Total frames attempted:   ", total_attempted)
    print("Total frames sent:        ", total_sent)
    print(f"Missed frames rate:        {missed_pct:.1f}%")

    if args.playback is not None or args.playback_realtime:
        print("")
        if args.playback_realtime:
            print(
                f"Starting realtime playback of {len(recorded_frames)} frames (original timings)..."
            )
        else:
            print(
                f"Starting playback of {len(recorded_frames)} frames (delay={args.playback}ms)..."
            )
        for idx, frame in enumerate(recorded_frames):
            if args.playback_realtime:
                if idx + 1 < len(recorded_frames):
                    frame_delay_ms = int(
                        (recorded_frames[idx + 1].timestamp - frame.timestamp) * 1000
                    )
                else:
                    frame_delay_ms = 1  # last frame: just wait 1ms
            else:
                frame_delay_ms = args.playback  # type: ignore[assignment]

            small_annotated = cv2.cvtColor(frame.small_img.copy(), cv2.COLOR_GRAY2BGR)
            big_annotated = cv2.cvtColor(frame.left_img.copy(), cv2.COLOR_GRAY2BGR)

            scale_x = frame.left_img.shape[1] / IMG_SCALE_SIZE[0]
            scale_y = frame.left_img.shape[0] / IMG_SCALE_SIZE[1]

            cv2.imshow("Left", big_annotated)
            cv2.imshow("Small left", small_annotated)
            key = cv2.waitKey(frame_delay_ms)
            if key == ord("q"):  # press Q to quit
                break

    cv2.destroyAllWindows()

    if args.plot:
        plot_timing(loop_times, process_elapsed_times, missed_frame_times, start_time)
