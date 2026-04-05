import os
import queue
import statistics
import struct
import sys
import threading
import time
import argparse
from typing import List, Optional

from cycler import V

os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
import cv2
import serial

from hil.frames import FrameItem, FrameRecord, IMG_SCALE_SIZE
from hil.uav import UAVStreamer
from hil.indoor import IndoorStreamer
from hil.protocol import COORD_FMT
from hil.stm32 import find_stm32_port
from hil.streamer import DatasetStreamer
from hil.threads import reader_thread_fn, writer_thread_fn
import numpy as np

from hil.plot import plot_predictions, plot_timing, plot_velocity_xy
from hil.kpi import (
    load_ground_truth,
    load_ground_truth_indoor,
    compute_kpi,
    print_kpi_report,
    kalman_filter_velocity,
    compute_velocity_gt,
)

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
            "For UAV (--dataset-type uav): a split folder containing "
            "query_images/ and reference_images/.  "
            "For indoor (--dataset-type indoor): the nav-cam folder "
            "containing img/ and nav_cam_timestamps.csv "
            "(e.g. /path/to/insane-dataset/indoor_1_nav_cam)."
        ),
    )
    parser.add_argument(
        "--dataset-type",
        type=str,
        choices=["uav", "indoor"],
        default="uav",
        help=(
            "Dataset type.  "
            "'uav'    – UAV split (default).  "
            "'indoor' – INSANE indoor nav-cam dataset."
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
            "Only used when --dataset-type indoor."
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
            "Only applied when --dataset-type indoor."
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
        "the STM32's tx/ty/theta estimates to ground truth.",
    )
    parser.add_argument(
        "--plot-kpi",
        action="store_true",
        default=False,
        help="After the run, display a timeseries plot of tx, ty and theta "
        "(predicted vs. ground truth).  Implies --kpi.",
    )
    parser.add_argument(
        "--gsd",
        type=float,
        metavar="M_PER_PX",
        default=0.0,
        help="Ground Sample Distance in metres per pixel for the 96×96 image used by the "
        "STM32.  When omitted (or 0), it is auto-computed from the altitude column in "
        "query.csv and --hfov.",
    )
    parser.add_argument(
        "--hfov",
        type=float,
        metavar="DEG",
        default=101.0,
        help="Camera horizontal field of view in degrees (default: 60).  Used to "
        "auto-compute the GSD from altitude when --gsd is not provided.",
    )
    parser.add_argument(
        "--vfov",
        type=float,
        metavar="DEG",
        default=114,
        help="Camera vertical field of view in degrees.  Defaults to --hfov (square "
        "sensor).  Provide this when VFOV differs from HFOV so that the ty (North) "
        "ground truth uses the correct per-axis GSD.",
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
    args = parser.parse_args()

    do_record = args.playback is not None or args.playback_realtime
    write_freq_hz: Optional[float] = args.write_freq

    if write_freq_hz is not None and write_freq_hz <= 0:
        parser.error("--write-freq must be a positive number")

    data_root: str = args.data_root
    print("Entering ", data_root)

    dataset_type: str = args.dataset_type

    if dataset_type == "indoor":
        streamer: DatasetStreamer = IndoorStreamer(
            data_root, None, start_frame=args.start_frame
        )
    else:
        streamer = UAVStreamer(data_root, None)

    port = find_stm32_port()
    ser = serial.Serial(port, timeout=1000)
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
    # (frame_number, tx, ty, theta) per received frame — populated when --kpi or --plot-kpi
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

        # Create the save directory once before the loop (if requested).
        save_dir: Optional[str] = args.save_dir
        if save_dir is not None:
            os.makedirs(save_dir, exist_ok=True)
            print(f"Saving annotated big images to: {save_dir}")

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

            # ------------------------------------------------------------------
            # Overlay optical-flow vectors on the small (96×96) image.
            #
            # The 11×11 grid of vectors is laid out as follows:
            #   - Column origins: x = 8, 16, 24, …, 88  (start=8, step=8)
            #   - Row    origins: y = 8, 16, 24, …, 88  (start=8, step=8)
            #
            # Each Coordinate (u, v) is the optical-flow displacement at that
            # grid point.  An arrow is drawn from (gx, gy) to (gx+u, gy+v).
            # ------------------------------------------------------------------
            GRID_START = 8
            GRID_STEP = 8
            GRID_COLS = 11
            GRID_ROWS = 11

            if frame.coords:
                for row_idx in range(GRID_ROWS):
                    for col_idx in range(GRID_COLS):
                        coord = frame.coords[row_idx * GRID_COLS + col_idx]
                        gx = GRID_START + col_idx * GRID_STEP  # 8, 16, …, 88
                        gy = GRID_START + row_idx * GRID_STEP  # 8, 16, …, 88
                        ex = gx + coord.u
                        ey = gy + coord.v
                        color = (0, 255, 0)
                        if not coord.valid:
                            color = (0, 0, 255)

                        cv2.arrowedLine(
                            small_annotated,
                            (gx, gy),
                            (ex, ey),
                            color,  # green arrow
                            1,
                            tipLength=0.4,
                        )

                        # --------------------------------------------------
                        # Same arrow on the big (full-resolution) image.
                        # Scale both origins and displacements by the ratio
                        # between the big image and the 96×96 small image.
                        # --------------------------------------------------
                        gx_big = int(gx * scale_x)
                        gy_big = int(gy * scale_y)
                        ex_big = int((gx + coord.u) * scale_x)
                        ey_big = int((gy + coord.v) * scale_y)
                        cv2.arrowedLine(
                            big_annotated,
                            (gx_big, gy_big),
                            (ex_big, ey_big),
                            color,  # green arrow
                            max(1, int(scale_x)),
                            tipLength=0.3,
                        )

            cv2.imshow("Left", big_annotated)
            cv2.imshow("Small left", small_annotated)

            # Save the annotated big image to disk (if --save-dir was given).
            if save_dir is not None:
                filename = os.path.join(save_dir, f"frame_{idx:06d}.png")
                cv2.imwrite(filename, big_annotated)

            key = cv2.waitKey(frame_delay_ms)
            if key == ord("q"):  # press Q to quit
                break

    cv2.destroyAllWindows()

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
        print("")
        print("Computing KPI…")
        try:
            gt_px: "np.ndarray | None" = None
            gt_py: "np.ndarray | None" = None

            if dataset_type == "indoor":
                # Derive sensors_root from data_root when not supplied.
                sensors_root: str = args.sensors_root or data_root.replace(
                    "_nav_cam", "_sensors"
                )
                (
                    tx_gt,
                    ty_gt,
                    theta_gt,
                    gsd_x,
                    gsd_y,
                    gt_px,
                    gt_py,
                ) = load_ground_truth_indoor(
                    data_root,
                    sensors_root=sensors_root,
                    gsd_m_per_px=args.gsd,
                    hfov_deg=args.hfov,
                    vfov_deg=args.vfov,
                )
            else:
                tx_gt, ty_gt, theta_gt, gsd_x, gsd_y = load_ground_truth(
                    data_root,
                    gsd_m_per_px=args.gsd,
                    hfov_deg=args.hfov,
                    vfov_deg=args.vfov,
                )

            frame_numbers = [fm[0] for fm in frame_meta_list]
            tx_pred = [fm[1] for fm in frame_meta_list]
            ty_pred = [fm[2] for fm in frame_meta_list]
            theta_pred = [fm[3] for fm in frame_meta_list]

            kpi_result = compute_kpi(
                frame_numbers,
                tx_pred,
                ty_pred,
                theta_pred,
                tx_gt,
                ty_gt,
                theta_gt,
            )
            print_kpi_report(kpi_result, gsd_x, gsd_y)

            if args.plot_kpi:
                plot_predictions(
                    frame_numbers,
                    tx_pred,
                    ty_pred,
                    theta_pred,
                    tx_gt,
                    ty_gt,
                    theta_gt,
                )

            # ------------------------------------------------------------------
            # Kalman-filter velocity estimation vs. ground-truth velocity
            # Available for both UAV and indoor datasets.
            # ------------------------------------------------------------------
            if args.plot_kpi and len(frame_numbers) > 1:
                print("")
                print("Running Kalman filter for velocity estimation…")

                vx_kf, vy_kf = kalman_filter_velocity(
                    frame_numbers, tx_pred, ty_pred, gsd_x, gsd_y
                )

                # Ground-truth velocity in m/frame, aligned with frame_numbers.
                vx_gt_aligned, vy_gt_aligned = compute_velocity_gt(
                    frame_numbers, tx_gt, ty_gt, gsd_x, gsd_y
                )

                # Report per-axis velocity MAE (valid GT frames only).
                valid_mask = ~np.isnan(vx_gt_aligned)
                if valid_mask.any():
                    vx_mae = float(
                        np.mean(np.abs(vx_kf[valid_mask] - vx_gt_aligned[valid_mask]))
                    )
                    vy_mae = float(
                        np.mean(np.abs(vy_kf[valid_mask] - vy_gt_aligned[valid_mask]))
                    )
                    print(f"  KF velocity MAE — vx: {vx_mae:.4f} m/frame")
                    print(f"  KF velocity MAE — vy: {vy_mae:.4f} m/frame")

                plot_velocity_xy(
                    frame_numbers,
                    vx_kf,
                    vy_kf,
                    vx_gt_aligned,
                    vy_gt_aligned,
                )
        except Exception as exc:
            print(f"KPI computation failed: {exc}")
    elif do_kpi:
        print("KPI requested but no frames were successfully received — skipping.")
