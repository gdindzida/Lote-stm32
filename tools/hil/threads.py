import struct
import queue
import threading
import time
from typing import List, Optional, Dict
import serial
from hil.csv_dataset import CsvDatasetStreamer
from hil.frames import FrameItem, FrameReadEvent, FrameWriteEvent
from hil.protocol import (
    SEND_HEADER_FMT,
    RECV_HEADER_FMT,
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
    write_freq_hz: Optional[float],
    frame_writes: List[FrameWriteEvent],
    frame_queue: "queue.Queue[Optional[FrameItem]]",
    frame_buffer_sem: threading.Semaphore,
    error_event: threading.Event,
    stop_event: threading.Event,
    calibration: Dict[str, float],
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
    image, entry = streamer.next()

    if image is None or entry is None:
        print("Left image is None!")
        error_event.set()
        return

    frame_number: int = 0

    print("Image shape: ", image.shape)
    img_data = image.tobytes()

    # print(
    #     "debug: Packet header contains: ",
    #     MAGIC,
    #     APP_RX_BUFFER_SIZE,
    #     0.0,  # dt = 0 for first frame
    #     float(entry.get("p_z", 0.0)),
    #     float(entry.get("acc_x", 0.0)),
    #     float(entry.get("acc_y", 0.0)),
    #     float(entry.get("acc_z", 0.0)),
    #     float(entry.get("gyro_x", 0.0)),
    #     float(entry.get("gyro_y", 0.0)),
    #     float(entry.get("gyro_z", 0.0)),
    #     calibration["fx"],
    #     calibration["fy"],
    #     calibration["cx"],
    #     calibration["cy"],
    #     calibration["k1"],
    #     calibration["k2"],
    # )
    packet_header = struct.pack(
        SEND_HEADER_FMT,
        MAGIC,
        APP_RX_BUFFER_SIZE,
        0.0,  # dt = 0 for first frame
        float(entry.get("p_z", 0.0)),
        float(entry.get("acc_x", 0.0)),
        float(entry.get("acc_y", 0.0)),
        float(entry.get("acc_z", 0.0)),
        float(entry.get("gyro_x", 0.0)),
        float(entry.get("gyro_y", 0.0)),
        float(entry.get("gyro_z", 0.0)),
        calibration["fx"],
        calibration["fy"],
        calibration["cx"],
        calibration["cy"],
        calibration["k1"],
        calibration["k2"],
    )

    # Pad to PACKET_SIZE to ensure separate USB packet
    sync_pkt = packet_header.ljust(PACKET_SIZE, b"\x00")

    frame_write_time = time.time()
    ser.write(sync_pkt)
    ser.write(img_data)
    frame_writes.append(FrameWriteEvent(frame_write_time, frame_write_time, False))
    t0 = frame_write_time

    # --- Subsequent frames ---
    while streamer.has_next() and not stop_event.is_set():
        image, entry = streamer.next()

        if image is None or entry is None:
            print("Image is None!")
            continue
        frame_number += 1

        img_data = image.tobytes()

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
        # print(
        #     "debug: Packet header contains: ",
        #     MAGIC,
        #     APP_RX_BUFFER_SIZE,
        #     dt,  # dt = 0 for first frame
        #     float(entry.get("p_z", 0.0)),
        #     float(entry.get("acc_x", 0.0)),
        #     float(entry.get("acc_y", 0.0)),
        #     float(entry.get("acc_z", 0.0)),
        #     float(entry.get("gyro_x", 0.0)),
        #     float(entry.get("gyro_y", 0.0)),
        #     float(entry.get("gyro_z", 0.0)),
        #     calibration["fx"],
        #     calibration["fy"],
        #     calibration["cx"],
        #     calibration["cy"],
        #     calibration["k1"],
        #     calibration["k2"],
        # )
        packet_header = struct.pack(
            SEND_HEADER_FMT,
            MAGIC,
            APP_RX_BUFFER_SIZE,
            dt,
            float(entry.get("p_z", 0.0)),
            float(entry.get("acc_x", 0.0)),
            float(entry.get("acc_y", 0.0)),
            float(entry.get("acc_z", 0.0)),
            float(entry.get("gyro_x", 0.0)),
            float(entry.get("gyro_y", 0.0)),
            float(entry.get("gyro_z", 0.0)),
            calibration["fx"],
            calibration["fy"],
            calibration["cx"],
            calibration["cy"],
            calibration["k1"],
            calibration["k2"],
        )

        # Pad to PACKET_SIZE to ensure separate USB packet
        sync_pkt = packet_header.ljust(PACKET_SIZE, b"\x00")

        if use_dataset_timestamps:
            deadline = frame_write_time + dt
            sleep_time = deadline - time.time()
            # print(
            #     "curr index: ",
            #     curr_idx,
            #     ", prev index: ",
            #     prev_idx,
            #     ", sleep time: ",
            #     sleep_time,
            #     ", curr timestamp: ",
            #     curr_ts,
            #     ", prev timestamp: ",
            #     prev_ts,
            # )
            if sleep_time > 0:
                time.sleep(sleep_time)
        elif period > 0.0:
            deadline = t0 + frame_number * period
            sleep_time = deadline - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
        else:
            deadline = time.time()

        # Check whether the MCU has finished processing the last two frames.
        if not frame_buffer_sem.acquire(blocking=False):
            frame_writes.append(FrameWriteEvent(time.time(), deadline, True))
            continue

        frame_write_time = time.time()
        ser.write(sync_pkt)
        ser.write(img_data)

        frame_writes.append(FrameWriteEvent(frame_write_time, deadline, False))

        frame_queue.put(
            FrameItem(
                image.copy(),
                frame_number,
                dt,
                float(entry.get("p_x", 0.0)),
                float(entry.get("p_y", 0.0)),
            )
        )

    # Signal the reader that no more frames will be written (None = sentinel)
    frame_queue.put(None)


def reader_thread_fn(
    ser: serial.Serial,
    frame_reads: List[FrameReadEvent],
    frame_queue: "queue.Queue[Optional[FrameItem]]",
    frame_buffer_sem: threading.Semaphore,
    error_event: threading.Event,
    total: int,
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
            frame_buffer_sem.release()
            break

        iter += 1

        print(f"Reading frame number: {item.frame_number} / {total}")
        print(f"Frames read: {iter} / {total}")

        image = item.image

        # Read header
        header_bytes = ser.read(struct.calcsize(RECV_HEADER_FMT))
        if len(header_bytes) < struct.calcsize(RECV_HEADER_FMT):
            print("Reader: timeout waiting for header")
            print("")
            continue
            # error_event.set()
            # break

        # Unpack header: magic, length, and all the metadata/calibration fields
        header_data = struct.unpack(RECV_HEADER_FMT, header_bytes)
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
        frame_buffer_sem.release()
        if len(payload) < length:
            print("Reader: timeout waiting for payload")
            error_event.set()
            break

        # Record write-to-read latency: time from when the writer sent this
        # frame to now (the moment the full MCU response has been received).
        read_time = time.time()

        print("Got payload of size: ", len(payload))

        meta_size = struct.calcsize(METADATA_FMT)
        meta_raw = struct.unpack(METADATA_FMT, payload[:meta_size])
        meta = Metadata(*meta_raw)

        print("Got this many valid points: ", meta.num_points)
        print("u_sum: ", meta.u_sum, ", v_sum: ", meta.v_sum)
        print("vx: ", meta.vx, ", vy: ", meta.vy, ", omega: ", meta.omega)
        print("Peak stack memory: ", meta.stack_mem_usage)
        print("Heap mem usage: ", meta.heap_mem_usage)
        print("")

        # Parse the NUM_COORDS optical-flow vectors that follow the metadata.
        # Each Coordinate is two signed int16_t values: (u=row, v=col) as packed
        # by the firmware's Coordinate struct {int16_t row; int16_t col;}.
        coord_size = struct.calcsize(COORD_FMT)
        coords: List[Coordinate] = []
        for i in range(NUM_COORDS):
            offset = meta_size + i * coord_size
            v, u, valid = struct.unpack(
                COORD_FMT, payload[offset : offset + coord_size]
            )
            coords.append(Coordinate(u=u, v=v, valid=bool(valid)))

        frame_reads.append(
            FrameReadEvent(
                read_time=read_time,
                image=image,
                payload=payload,
                frame_number=item.frame_number,
                meta=meta,
                dt=item.dt,
                px=item.px,
                py=item.py,
                coords=coords,
            )
        )
