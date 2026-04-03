from streamer.kitti.streamer import KittiStreamer
from streamer.base import DatasetStreamer
from streamer.base import DatasetStreamerAdapter
from typing import List, Optional
import numpy as np
import os
import statistics
import argparse

os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
import cv2
import time
import serial
import serial.tools.list_ports
import struct
import threading
import queue
from dataclasses import dataclass

MAGIC = 0xABCD
HEADER_FMT = "<HH"
METADATA_FMT = "<IhhHfffff"
COORD_FMT = "BB"


@dataclass
class Metadata:
    process_elapsed_time_ms: int
    u_sum: int
    v_sum: int
    num_points: int
    stack_mem_usage: float
    heap_mem_usage: float
    tx: float
    ty: float
    theta: float


@dataclass
class Coordinate:
    x: int
    y: int


def find_stm32_port() -> str:
    """Auto-detect the STM32 CDC port (VID 0x0483)."""
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483:
            return p.device
    raise RuntimeError("STM32 CDC port not found")


@dataclass
class FrameRecord:
    small_img: np.ndarray
    left_img: np.ndarray
    payload: bytes
    meta: Metadata
    meta_size: int
    timestamp: float = 0.0


@dataclass
class _FrameItem:
    """Data pushed into the inter-thread queue by the writer for each sent frame."""

    small_img: np.ndarray
    left_img: np.ndarray
    write_time: float


def writer_thread_fn(
    ser: serial.Serial,
    streamer: DatasetStreamer,
    frame_queue: "queue.Queue[Optional[_FrameItem]]",
    write_freq_hz: Optional[float],
    do_record: bool,
    loop_times: List[float],
    error_event: threading.Event,
):
    """Reads frames from the streamer, writes them to serial, and enqueues frame data for the reader."""
    period = (1.0 / write_freq_hz) if write_freq_hz is not None else 0.0

    # --- First frame (sent before the main loop) ---
    result = streamer.next()
    if result is None:
        print("Images are None!")
        error_event.set()
        frame_queue.put(None)
        return

    left_img, right_img = result

    if left_img is None:
        print("Left image is None!")
        error_event.set()
        frame_queue.put(None)
        return

    if right_img is None:
        print("Right image is None!")
        error_event.set()
        frame_queue.put(None)
        return

    small_img = cv2.resize(left_img, (128, 64), interpolation=cv2.INTER_AREA)
    print(left_img.shape)
    print(small_img.shape)
    img_data = small_img.tobytes()

    loop_time = time.time()
    frame_write_time = loop_time

    ser.write(img_data)
    # Enqueue the frame images so the reader can build a FrameRecord
    frame_queue.put(_FrameItem(small_img.copy(), left_img.copy(), frame_write_time))

    # --- Subsequent frames ---
    while streamer.has_next():
        result = streamer.next()
        if result is None:
            print("Images are None!")
            continue

        left_img, right_img = result

        if left_img is None:
            print("Left image is None!")
            continue

        if right_img is None:
            print("Right image is None!")
            continue

        small_img = cv2.resize(left_img, (128, 64), interpolation=cv2.INTER_AREA)
        img_data = small_img.tobytes()

        # Frequency throttling: sleep for the remainder of the period
        if period > 0.0:
            elapsed_since_last = time.time() - frame_write_time
            sleep_time = period - elapsed_since_last
            if sleep_time > 0:
                time.sleep(sleep_time)

        new_loop_time = time.time()
        loop_times.append(new_loop_time - loop_time)
        loop_time = new_loop_time

        frame_write_time = time.time()
        ser.write(img_data)
        frame_queue.put(_FrameItem(small_img.copy(), left_img.copy(), frame_write_time))

    # Signal the reader that no more frames will be written (None = sentinel)
    frame_queue.put(None)


def reader_thread_fn(
    ser: serial.Serial,
    frame_queue: "queue.Queue[Optional[_FrameItem]]",
    do_record: bool,
    process_elapsed_times: List[float],
    recorded_frames: List[FrameRecord],
    peak_memory: List[float],  # [peak_stack, peak_heap]
    error_event: threading.Event,
):
    """Reads serial responses and collects statistics / records frames."""
    iter: int = 0
    while True:
        print("Current iter: ", iter)
        iter += 1
        # Get the matching frame entry from the writer
        item = frame_queue.get()

        if item is None:
            # None sentinel: no more frames; reader is done
            break

        small_img = item.small_img
        left_img = item.left_img

        # Read header
        header_bytes = ser.read(struct.calcsize(HEADER_FMT))
        if len(header_bytes) < struct.calcsize(HEADER_FMT):
            print("Reader: timeout waiting for header")
            error_event.set()
            break

        magic, length = struct.unpack(HEADER_FMT, header_bytes)
        print("Size of header: ", len(header_bytes))
        if magic != MAGIC:
            print(f"Reader: bad magic: {hex(magic)}")
            error_event.set()
            break

        payload = ser.read(length)
        if len(payload) < length:
            print("Reader: timeout waiting for payload")
            error_event.set()
            break

        print("Got payload of size: ", len(payload))

        meta_size = struct.calcsize(METADATA_FMT)
        meta_raw = struct.unpack(METADATA_FMT, payload[:meta_size])
        meta = Metadata(*meta_raw)

        process_elapsed_times.append(meta.process_elapsed_time_ms)
        peak_memory[0] = meta.stack_mem_usage
        peak_memory[1] = meta.heap_mem_usage
        print("Got this many valid points: ", meta.num_points)
        print("u_sum: ", meta.u_sum, ", v_sum: ", meta.v_sum)
        print("tx: ", meta.tx, ", ty: ", meta.ty, ", theta: ", meta.theta)
        print("")

        if do_record:
            recorded_frames.append(
                FrameRecord(
                    small_img=small_img,
                    left_img=left_img,
                    payload=payload,
                    meta=meta,
                    meta_size=meta_size,
                    timestamp=time.time(),
                )
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
    args = parser.parse_args()

    do_record = args.playback is not None or args.playback_realtime
    write_freq_hz: Optional[float] = args.write_freq

    if write_freq_hz is not None and write_freq_hz <= 0:
        parser.error("--write-freq must be a positive number")

    data_root: str = "modules/gaspar/data/2011_09_26_drive_0001_sync"
    print("Entering ", data_root)

    streamer: DatasetStreamer = KittiStreamer(data_root, None)

    port = find_stm32_port()
    ser = serial.Serial(port, timeout=10)
    print(f"Connected to {port}")

    if write_freq_hz is not None:
        print(
            f"Write frequency: {write_freq_hz} Hz (period: {1000.0 / write_freq_hz:.1f} ms)"
        )
    else:
        print("Write frequency: unlimited (max throughput)")

    loop_times: List[float] = []
    process_elapsed_times: List[float] = []
    peak_memory: List[float] = [0.0, 0.0]  # [stack, heap]
    recorded_frames: List[FrameRecord] = []

    print("Starting KITTI clip playback...")

    # Shared queue: writer pushes _FrameItem entries; None sentinel signals completion
    frame_queue: "queue.Queue[Optional[_FrameItem]]" = queue.Queue(maxsize=0)
    error_event = threading.Event()

    start_time = time.time()

    writer = threading.Thread(
        target=writer_thread_fn,
        name="ser-writer",
        args=(
            ser,
            streamer,
            frame_queue,
            write_freq_hz,
            do_record,
            loop_times,
            error_event,
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
            error_event,
        ),
        daemon=True,
    )

    writer.start()
    reader.start()

    writer.join()
    reader.join()

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
    if loop_times:
        print("Avg time(ms): ", 1000 * elapsed_time / (len(loop_times) + 1))

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

            scale_x = frame.left_img.shape[1] / 128.0
            scale_y = frame.left_img.shape[0] / 64.0

            if frame.meta.num_points > 0:
                coord_size = struct.calcsize(COORD_FMT)
                offset = frame.meta_size
                # print("Got this many points: ", frame.meta.num_points)
                # for i in range(frame.meta.num_points):
                #     x, y = struct.unpack(
                #         COORD_FMT, frame.payload[offset : offset + coord_size]
                #     )
                #     offset += coord_size
                #     x, y = int(x) & 0xFF, int(y) & 0xFF  # interpret as uint8_t (0–255)
                #     # print(f"  point[{i}]: x={y}, y={x}")

                #     # Draw on small image (coordinates are in small image space)
                #     cv2.circle(
                #         small_annotated,
                #         (y, x),
                #         radius=2,
                #         color=(0, 255, 0),
                #         thickness=-1,
                #     )

                #     # Scale up and draw on big image
                #     big_x = int(x * scale_y)
                #     big_y = int(y * scale_x)
                #     cv2.circle(
                #         big_annotated,
                #         (big_y, big_x),
                #         radius=5,
                #         color=(0, 255, 0),
                #         thickness=-1,
                #     )
            else:
                print("No points!")

            cv2.imshow("Left", big_annotated)
            cv2.imshow("Small left", small_annotated)
            key = cv2.waitKey(frame_delay_ms)
            if key == ord("q"):  # press Q to quit
                break

    # streamer.run()
    cv2.destroyAllWindows()
