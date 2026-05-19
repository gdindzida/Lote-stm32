import os
import queue
import sys
import threading
import time
import argparse
from typing import List, Optional, Dict

os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
import serial

from hil.frames import FrameItem, FrameWriteEvent, FrameReadEvent
from hil.csv_dataset import CsvDatasetStreamer
from hil.calibration import load_camchain

from hil.kpi import compute_and_print_kpi
from hil.playback import playback_recorded_frames
from hil.plot import plot_frame_metrics
from hil.stats import compute_and_print_metrics
from hil.stm32 import find_stm32_port
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
        default=None,
        help="Frequency (in Hz) at which frames are written to the serial port. "
        "Omit for maximum throughput (no throttling).",
    )
    parser.add_argument(
        "--dataset-csv",
        type=str,
        metavar="PATH",
        required=True,
        help=(
            "Path to a dataset.csv file (created by tools/calibration/create_dataset.py). "
            "The CSV must contain 'timestamp_cam' and 'image_path' columns."
        ),
    )
    parser.add_argument(
        "--camchain",
        type=str,
        metavar="PATH",
        required=True,
        help="Path to camchain.yaml file containing camera calibration parameters (fx, fy, cx, cy, k1, k2).",
    )
    parser.add_argument(
        "--data-root",
        type=str,
        metavar="PATH",
        default=None,
        required=True,
        help=(
            "Root directory for resolving relative image paths in the CSV. "
            "If not provided, image paths in CSV are assumed to be absolute."
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
        type=int,
        metavar="ms",
        default=None,
        help="Timeout in ms for serial connection.",
    )
    args = parser.parse_args()

    write_freq_hz: Optional[float] = args.write_freq

    if write_freq_hz is not None and write_freq_hz <= 0:
        parser.error("--write-freq must be a positive number")

    # Load camera calibration
    print(f"Loading camera calibration from: {args.camchain}")
    calibration: Dict[str, float] = load_camchain(args.camchain)
    print(
        f"Calibration: fx={calibration['fx']:.2f}, fy={calibration['fy']:.2f}, "
        f"cx={calibration['cx']:.2f}, cy={calibration['cy']:.2f}, "
        f"k1={calibration['k1']:.6f}, k2={calibration['k2']:.6f}"
    )

    # Initialize CSV dataset streamer
    print(f"Using CSV dataset: {args.dataset_csv}")
    streamer: CsvDatasetStreamer = CsvDatasetStreamer(
        dataset_csv_path=args.dataset_csv,
        data_root=args.data_root,  # May be None, CSV paths can be absolute
        dataset_streamer_adapter=None,
        start_frame=args.start_frame,
    )
    data_root: Optional[str] = args.data_root

    port = find_stm32_port()
    ser = serial.Serial(port, timeout=args.timeout)
    print(f"Connected to {port}")

    if write_freq_hz is not None:
        print(
            f"Write frequency: {write_freq_hz} Hz (period: {1000.0 / write_freq_hz:.1f} ms)"
        )
    else:
        print("Write frequency: unlimited (max throughput)")

    frame_writes: List[FrameWriteEvent] = []
    frame_reads: List[FrameReadEvent] = []

    print("Starting clip playback...")
    print("Press 'q' + Enter at any time to stop streaming early.")

    frame_queue: "queue.Queue[Optional[FrameItem]]" = queue.Queue(maxsize=0)
    frame_buffer_sem = threading.Semaphore(1)
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
            write_freq_hz,
            frame_writes,
            frame_queue,
            frame_buffer_sem,
            error_event,
            stop_event,
            calibration,
        ),
        daemon=True,
    )

    reader = threading.Thread(
        target=reader_thread_fn,
        name="ser-reader",
        args=(
            ser,
            frame_reads,
            frame_queue,
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

    if error_event.is_set():
        print("An error occurred during serial communication. Aborting.")
        raise SystemExit(1)

    elapsed_time = time.time() - start_time

    # Statitics
    print("")
    print("Statistics")

    print("")
    print("Total elapsed time(s): ", elapsed_time)
    print("Desired freq: ", write_freq_hz)
    compute_and_print_metrics(frame_writes, frame_reads)

    if args.plot:
        plot_frame_metrics(frame_writes, frame_reads)

    do_kpi = args.kpi or args.plot_kpi
    if do_kpi:
        compute_and_print_kpi(
            frame_reads=frame_reads,
            plot_kpi=args.plot_kpi,
        )

    if args.playback is not None or args.playback_realtime:
        playback_recorded_frames(
            frame_reads=frame_reads,
            playback_delay_ms=args.playback,
            playback_realtime=args.playback_realtime,
            save_dir=args.save_dir,
        )
