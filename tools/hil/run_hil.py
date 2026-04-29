import os
import queue
import sys
import threading
import time
import argparse
from typing import List, Optional

os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
import cv2
import serial

from hil.frames import FrameItem, FrameRecord
from hil.indoor import IndoorStreamer
from hil.kpi import compute_and_print_kpi
from hil.playback import playback_recorded_frames
from hil.plot import plot_timing
from hil.statistics import print_statistics
from hil.stm32 import find_stm32_port
from hil.streamer import DatasetStreamer
from hil.threads import reader_thread_fn, writer_thread_fn

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
        help=(
            "Path to the dataset folder.  "
            "the nav-cam folder containing img/ and nav_cam_timestamps.csv "
            "(e.g. /path/to/insane-dataset/indoor_1_nav_cam)."
        ),
    )
    parser.add_argument(
        "--sensors-root",
        type=str,
        metavar="PATH",
        default=None,
        help=(
            "Path to the sensors folder for the indoor dataset "
            "(e.g. /path/to/insane-dataset/indoor_1_sensors).  "
            "When omitted the path is auto-derived from --data-root by "
            "replacing the trailing '_nav_cam' suffix with '_sensors'.  "
        ),
    )
    parser.add_argument(
        "--start-frame",
        type=int,
        metavar="N",
        default=1000,
        help=(
            "1-based frame number at which to start streaming "
            "(default: 1000).  Frames before this number are skipped.  "
        ),
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        default=False,
        help="After the run, display a timing bar chart (loop time / process time per frame, "
        "with markers for missed frames).",
    )
    parser.add_argument(
        "--kpi",
        action="store_true",
        default=False,
        help="After the run, compute and print accuracy KPI (MAE, RMSE, R², …) comparing "
        "the STM32's vx/vy/omega estimates to ground truth.",
    )
    parser.add_argument(
        "--plot-kpi",
        action="store_true",
        default=False,
        help="After the run, display a timeseries plot of vx, vy and omega "
        "(predicted vs. ground truth).  Implies --kpi.",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        metavar="PATH",
        default=None,
        help="Directory in which to save every annotated big image during playback "
        "(one PNG per frame, named frame_XXXXXX.png).  The directory is created "
        "automatically if it does not exist.  Only has effect when --playback or "
        "--playback-realtime is also specified.",
    )
    parser.add_argument(
        "--timeout",
        type=str,
        metavar="ms",
        default=None,
        help="Timeout in ms for serial connection.",
    )
    args = parser.parse_args()

    do_record = args.playback is not None or args.playback_realtime
    write_freq_hz: Optional[float] = args.write_freq

    if write_freq_hz is not None and write_freq_hz <= 0:
        parser.error("--write-freq must be a positive number")

    data_root: str = args.data_root
    print("Using dataset in ", data_root)

    streamer: DatasetStreamer = IndoorStreamer(
        data_root, None, start_frame=args.start_frame
    )

    port = find_stm32_port()
    ser = serial.Serial(port, timeout=args.timeout)
    print(f"Connected to {port}")

    if write_freq_hz is not None:
        print(
            f"Write frequency: {write_freq_hz} Hz (period: {1000.0 / write_freq_hz:.1f} ms)"
        )
    else:
        print("Write frequency: unlimited (max throughput)")

    frame_write_times: List[float] = []
    frame_deadline_times: List[float] = []  # absolute deadline for each sent frame
    frame_loop_times: List[float] = []  # write-to-read latency per frame (seconds)
    process_elapsed_times: List[float] = []
    peak_memory: List[float] = [0.0, 0.0]  # [stack, heap]
    recorded_frames: List[FrameRecord] = []
    missed_frames: List[int] = [0]  # [missed_frame_count]
    missed_frame_times: List[float] = []  # absolute timestamps of each missed frame
    # (frame_number, vx, vy, omega) per received frame — populated when --kpi or --plot-kpi
    frame_meta_list: "List[tuple[int, float, float, float]] | None" = (
        [] if (args.kpi or args.plot_kpi) else None
    )

    print("Starting clip playback...")
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
            frame_deadline_times,
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
            frame_loop_times,
        ),
        kwargs={"frame_meta_list": frame_meta_list},
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

    # Write-to-read latency per frame: collected by the reader as each MCU
    # response is fully received.  One entry per successfully processed frame.
    loop_times = frame_loop_times

    if error_event.is_set():
        print("An error occurred during serial communication. Aborting.")
        raise SystemExit(1)

    peak_stack_memory = peak_memory[0]
    peak_heap_memory = peak_memory[1]

    elapsed_time = time.time() - start_time

    print_statistics(
        elapsed_time=elapsed_time,
        write_freq_hz=write_freq_hz,
        frame_write_times=frame_write_times,
        frame_loop_times=loop_times,
        process_elapsed_times=process_elapsed_times,
        peak_stack_memory=peak_stack_memory,
        peak_heap_memory=peak_heap_memory,
        missed_frames_count=missed_frames[0],
    )

    if args.playback is not None or args.playback_realtime:
        playback_recorded_frames(
            recorded_frames=recorded_frames,
            playback_delay_ms=args.playback,
            playback_realtime=args.playback_realtime,
            save_dir=args.save_dir,
        )

    if args.plot:
        plot_timing(
            loop_times,
            process_elapsed_times,
            missed_frame_times,
            start_time,
            frame_write_times,
            frame_deadline_times,
            write_freq_hz,
        )

    do_kpi = args.kpi or args.plot_kpi
    if do_kpi and frame_meta_list:
        # Derive sensors_root from data_root when not supplied.
        sensors_root: str = args.sensors_root or data_root.replace(
            "_nav_cam", "_sensors"
        )
        compute_and_print_kpi(
            frame_meta_list=frame_meta_list,
            data_root=data_root,
            sensors_root=sensors_root,
            plot_kpi=args.plot_kpi,
        )
    elif do_kpi:
        print("KPI requested but no frames were successfully received — skipping.")
