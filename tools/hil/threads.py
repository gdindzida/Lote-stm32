import queue
import struct
import threading
import time
from typing import List, Optional

import serial

from hil.csv_dataset import CsvDatasetStreamer
from hil.frames import FrameItem, FrameRecord
from hil.protocol import (
    HEADER_FMT,
    MAGIC,
    METADATA_FMT,
    COORD_FMT,
    NUM_COORDS,
    PACKET_SIZE,
    APP_RX_BUFFER_SIZE,
    Metadata,
    Coordinate,
)


def writer_thread_fn(
    ser: serial.Serial,
    streamer: CsvDatasetStreamer,
    frame_queue: "queue.Queue[Optional[FrameItem]]",
    write_freq_hz: Optional[float],
    do_record: bool,
    frame_write_times: List[float],
    missed_frames: List[int],
    missed_frame_times: List[float],
    frame_buffer_sem: threading.Semaphore,
    error_event: threading.Event,
    stop_event: threading.Event,
    frame_deadline_times: List[float],
    calibration: dict,
):
    """Reads frames from the streamer, writes them to serial, and enqueues frame data for the reader.

    The STM32 MCU has a two-frame receive buffer modelled by ``frame_buffer_sem``
    (a Semaphore initialised to 2).  Before transmitting each frame the writer
    tries a non-blocking acquire.  If no slot is available the MCU buffer is
    full and the frame is skipped; the writer still waits for the full period so
    the overall timing cadence is preserved.  The semaphore slot is released by
    the reader only *after* it has fully received the MCU serial response,
    ensuring the signal is tied to actual MCU completion rather than the
    inter-thread queue pop.  Every skipped frame is counted in
    ``missed_frames[0]``; its absolute timestamp is appended to
    ``missed_frame_times``.

    Timing behavior:
    - When write_freq_hz is defined: Uses camera timestamps from the CSV dataset
      to compute inter-frame delays. This preserves the original timing from the dataset.
    - When write_freq_hz is None: Maximum throughput mode (no throttling).

    The deadline of every successfully sent frame is appended to
    ``frame_deadline_times``; the actual write timestamp is appended
    to ``frame_write_times``.
    """
    # Check if streamer provides timestamps (CsvDatasetStreamer always does)
    use_dataset_timestamps = write_freq_hz is None

    if use_dataset_timestamps and write_freq_hz is not None:
        print("Using camera timestamps from dataset for frame timing")

    period = (1.0 / write_freq_hz) if write_freq_hz is not None else 0.0

    # --- First frame (sent before the main loop) ---
    left_img = streamer.next()

    if left_img is None:
        print("Left image is None!")
        error_event.set()
        frame_queue.put(None)
        return

    frame_number: int = 0

    small_img = left_img
    print(left_img.shape)
    print(small_img.shape)
    img_data = small_img.tobytes()

    # Get first frame metadata (dt=0 for first frame)
    entry = streamer.entries[0]  # type: ignore

    # Build PacketHeader with metadata and calibration (56 bytes total)
    # Then pad the first packet to PACKET_SIZE to ensure it's sent separately
    packet_header = struct.pack(
        HEADER_FMT,
        MAGIC,
        APP_RX_BUFFER_SIZE,
        0.0,  # dt = 0 for first frame
        float(entry.get("p_x", 0.0)),
        float(entry.get("p_y", 0.0)),
        float(entry.get("p_z", 0.0)),
        float(entry.get("roll", 0.0)),
        float(entry.get("pitch", 0.0)),
        float(entry.get("yaw", 0.0)),
        calibration["fx"],
        calibration["fy"],
        calibration["cx"],
        calibration["cy"],
        calibration["k1"],
        calibration["k2"],
    )

    print("debug: size of packet header: ", len(packet_header))

    # Pad to PACKET_SIZE to ensure separate USB packet
    sync_pkt = packet_header.ljust(PACKET_SIZE, b"\x00")

    print("debug: size of sync packet: ", len(sync_pkt))
    print("debug: size of image data: ", len(img_data))

    # Acquire one MCU buffer slot for the first frame (always available at start).
    # frame_buffer_sem.acquire()
    frame_write_time = time.time()
    # t0 is the absolute origin for all subsequent deadlines.
    t0: float = frame_write_time
    print("debug: syncing")
    ser.write(sync_pkt)
    print("debug: writing image")
    ser.write(img_data)
    frame_write_times.append(frame_write_time)
    frame_deadline_times.append(t0)  # deadline for frame 0 is t0 by definition
    # Enqueue the frame images so the reader can build a FrameRecord
    # frame_queue.put(
    #     FrameItem(small_img.copy(), left_img.copy(), frame_write_time, frame_number)
    # )

    # --- Subsequent frames ---
    while streamer.has_next() and not stop_event.is_set():
        left_img = streamer.next()

        if left_img is None:
            print("Left image is None!")
            continue

        frame_number += 1

        small_img = left_img
        img_data = small_img.tobytes()

        # Compute deadline for this frame:
        # - If dataset timestamps available: use relative time from dataset
        # - Otherwise: use fixed-frequency timing with absolute deadlines
        if use_dataset_timestamps:
            # Get timestamps for current and previous frames from the dataset
            # streamer.index was incremented by next(), so current frame is at index-1
            curr_idx = streamer.index - 1  # type: ignore
            prev_idx = curr_idx - 1

            # Get dataset timestamps
            curr_ts = streamer.get_timestamp(curr_idx)  # type: ignore
            prev_ts = streamer.get_timestamp(prev_idx)  # type: ignore

            # Calculate inter-frame delay from dataset
            dataset_delay = curr_ts - prev_ts
            # Schedule this frame relative to when we sent the previous frame
            deadline = frame_write_times[-1] + dataset_delay
            sleep_time = deadline - time.time()
            print(
                "curr index: ",
                curr_idx,
                ", prev index: ",
                prev_idx,
                ", sleep time: ",
                sleep_time,
                ", curr timestamp: ",
                curr_ts,
                ", prev timestamp: ",
                prev_ts,
            )
            if sleep_time > 0:
                time.sleep(sleep_time)
        elif period > 0.0:
            # Fixed-frequency timing: absolute deadline to prevent drift
            deadline = t0 + frame_number * period
            sleep_time = deadline - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
        else:
            deadline = time.time()

        # Check whether the MCU has finished processing the last two frames.
        # The STM32 has a 2-frame receive buffer modelled by frame_buffer_sem.
        # A non-blocking acquire fails when both slots are occupied, meaning
        # the MCU has not yet responded to either outstanding frame.
        # We have already waited until the deadline above so the timing cadence
        # is preserved regardless of whether we send or skip.
        if not frame_buffer_sem.acquire(blocking=False):
            missed_frames[0] += 1
            missed_frame_times.append(time.time())
            print("debug: missing frame")
            continue

        # Get current frame metadata from dataset
        curr_idx = streamer.index - 1  # type: ignore
        prev_idx = curr_idx - 1
        entry = streamer.entries[curr_idx]  # type: ignore
        prev_entry = streamer.entries[prev_idx]  # type: ignore

        # Calculate dt from timestamps
        curr_ts = float(entry.get("timestamp_cam", 0.0))
        prev_ts = float(prev_entry.get("timestamp_cam", 0.0))
        dt = curr_ts - prev_ts

        # Build PacketHeader with current frame metadata and calibration
        packet_header = struct.pack(
            HEADER_FMT,
            MAGIC,
            APP_RX_BUFFER_SIZE,
            dt,
            float(entry.get("p_x", 0.0)),
            float(entry.get("p_y", 0.0)),
            float(entry.get("p_z", 0.0)),
            float(entry.get("roll", 0.0)),
            float(entry.get("pitch", 0.0)),
            float(entry.get("yaw", 0.0)),
            calibration["fx"],
            calibration["fy"],
            calibration["cx"],
            calibration["cy"],
            calibration["k1"],
            calibration["k2"],
        )

        # Pad to PACKET_SIZE to ensure separate USB packet
        print("debug: size of packet header: ", len(packet_header))

        sync_pkt = packet_header.ljust(PACKET_SIZE, b"\x00")

        print("debug: size of sync packet: ", len(sync_pkt))
        print("debug: size of image data: ", len(img_data))

        frame_write_time = time.time()
        print("debug: syncing")
        ser.write(sync_pkt)
        print("debug: writing image")
        ser.write(img_data)
        frame_write_times.append(frame_write_time)
        frame_deadline_times.append(deadline)
        frame_queue.put(FrameItem(small_img.copy(), frame_write_time, frame_number))

    # Signal the reader that no more frames will be written (None = sentinel)
    frame_queue.put(None)


def reader_thread_fn(
    ser: serial.Serial,
    frame_queue: "queue.Queue[Optional[FrameItem]]",
    do_record: bool,
    process_elapsed_times: List[float],
    recorded_frames: List[FrameRecord],
    peak_memory: List[float],  # [peak_stack, peak_heap]
    frame_buffer_sem: threading.Semaphore,
    error_event: threading.Event,
    total: int,
    frame_loop_times: List[float],
    frame_meta_list: "List[tuple[int, float, float, float]] | None" = None,
):
    """Reads serial responses and collects statistics / records frames.

    After successfully receiving the MCU serial response for each frame the
    semaphore slot acquired by the writer is released, signalling that the MCU
    buffer slot is now free.  The release also happens on error paths so the
    writer is never left blocked on a semaphore that will never be freed.

    The write-to-read latency of every successfully received frame is appended
    to ``frame_loop_times`` as ``read_time - item.write_time`` (seconds).  This
    captures the true per-frame round-trip: from when the writer sent the frame
    to when the reader finished receiving the MCU response.
    """

    iter: int = 0
    while True:
        # Get the matching frame entry from the writer
        item: FrameItem | None = frame_queue.get()

        if item is None:
            # None sentinel: no more frames; reader is done
            break

        iter += 1

        print(f"Reading frame number: {item.frame_number} / {total}")
        print(f"Frames read: {iter} / {total}")

        small_img = item.small_img

        # Read header
        print("debug: waiting for frame...")
        header_bytes = ser.read(struct.calcsize(HEADER_FMT))
        if len(header_bytes) < struct.calcsize(HEADER_FMT):
            print("Reader: timeout waiting for header")
            print("")
            frame_buffer_sem.release()
            continue
            # error_event.set()
            # break

        # Unpack header: magic, length, and all the metadata/calibration fields
        header_data = struct.unpack(HEADER_FMT, header_bytes)
        magic = header_data[0]
        length = header_data[1]
        # header_data[2:] contains dt, p_x, p_y, p_z, roll, pitch, yaw, fx, fy, cx, cy, k1, k2
        print("Size of header: ", len(header_bytes))
        if magic != MAGIC:
            print(f"Reader: bad magic: {hex(magic)}")
            frame_buffer_sem.release()
            error_event.set()
            break

        payload = ser.read(length)
        if len(payload) < length:
            print("Reader: timeout waiting for payload")
            frame_buffer_sem.release()
            error_event.set()
            break

        # Record write-to-read latency: time from when the writer sent this
        # frame to now (the moment the full MCU response has been received).
        read_time = time.time()
        frame_loop_times.append(read_time - item.write_time)

        # MCU has finished processing this frame — release its buffer slot so
        # the writer knows it may send another frame.
        frame_buffer_sem.release()

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
        print("Peak stack memory: ", meta.stack_mem_usage)
        print("Heap mem usage: ", meta.heap_mem_usage)
        print("")

        # Collect (frame_number, tx, ty, theta) for KPI computation when requested.
        if frame_meta_list is not None:
            frame_meta_list.append((item.frame_number, meta.tx, meta.ty, meta.theta))

        if do_record:
            # Parse the NUM_COORDS optical-flow vectors that follow the metadata.
            # Each Coordinate is two signed int16_t values: (u=row, v=col) as packed
            # by the firmware's Coordinate struct {int16_t row; int16_t col;}.
            coord_size = struct.calcsize(COORD_FMT)
            coords: List[Coordinate] = []
            for i in range(NUM_COORDS):
                offset = meta_size + i * coord_size
                u, v, valid = struct.unpack(
                    COORD_FMT, payload[offset : offset + coord_size]
                )
                coords.append(Coordinate(u=u, v=v, valid=bool(valid)))

            recorded_frames.append(
                FrameRecord(
                    small_img=small_img,
                    payload=payload,
                    meta=meta,
                    meta_size=meta_size,
                    timestamp=time.time(),
                    coords=coords,
                )
            )
