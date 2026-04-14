import queue
import struct
import threading
import time
from typing import List, Optional

import cv2
import serial

from hil.frames import FrameItem, FrameRecord
from hil.protocol import HEADER_FMT, MAGIC, METADATA_FMT, Metadata
from hil.streamer import DatasetStreamer


def writer_thread_fn(
    ser: serial.Serial,
    streamer: DatasetStreamer,
    frame_queue: "queue.Queue[Optional[FrameItem]]",
    write_freq_hz: Optional[float],
    do_record: bool,
    loop_times: List[float],
    error_event: threading.Event,
    stop_event: threading.Event,
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
    frame_queue.put(FrameItem(small_img.copy(), left_img.copy(), frame_write_time))

    # --- Subsequent frames ---
    while streamer.has_next() and not stop_event.is_set():
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
        frame_queue.put(FrameItem(small_img.copy(), left_img.copy(), frame_write_time))

    # Signal the reader that no more frames will be written (None = sentinel)
    frame_queue.put(None)


def reader_thread_fn(
    ser: serial.Serial,
    frame_queue: "queue.Queue[Optional[FrameItem]]",
    do_record: bool,
    process_elapsed_times: List[float],
    recorded_frames: List[FrameRecord],
    peak_memory: List[float],  # [peak_stack, peak_heap]
    error_event: threading.Event,
    total: int,
):
    """Reads serial responses and collects statistics / records frames."""
    iter: int = 0
    while True:
        # Get the matching frame entry from the writer
        item = frame_queue.get()

        if item is None:
            # None sentinel: no more frames; reader is done
            break

        iter += 1
        print(f"Current iter: {iter} / {total}")

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
