import queue
import struct
import threading
import time
from typing import List, Optional

import serial

from hil.frames import FrameItem, FrameRecord, scale_image
from hil.protocol import HEADER_FMT, MAGIC, METADATA_FMT, Metadata
from hil.streamer import DatasetStreamer


def writer_thread_fn(
    ser: serial.Serial,
    streamer: DatasetStreamer,
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

    Timing uses absolute deadlines anchored to the first-frame write time
    (``t0``).  Deadline for frame *n* is ``t0 + n * period``.  This prevents
    drift accumulation that occurs when sleeping only for the remaining portion
    of the previous interval.  The deadline of every successfully sent frame is
    appended to ``frame_deadline_times``; the actual write timestamp is appended
    to ``frame_write_times``.
    """
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

    frame_number: int = 0

    small_img = scale_image(left_img)
    print(left_img.shape)
    print(small_img.shape)
    img_data = small_img.tobytes()

    # Acquire one MCU buffer slot for the first frame (always available at start).
    frame_buffer_sem.acquire()
    frame_write_time = time.time()
    # t0 is the absolute origin for all subsequent deadlines.
    t0: float = frame_write_time
    ser.write(img_data)
    frame_write_times.append(frame_write_time)
    frame_deadline_times.append(t0)  # deadline for frame 0 is t0 by definition
    # Enqueue the frame images so the reader can build a FrameRecord
    frame_queue.put(
        FrameItem(small_img.copy(), left_img.copy(), frame_write_time, frame_number)
    )

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

        frame_number += 1

        small_img = scale_image(left_img)
        img_data = small_img.tobytes()

        # Absolute-deadline throttling: sleep until the scheduled send time for
        # this frame number.  Using an absolute deadline (t0 + n * period) rather
        # than sleeping for "period - elapsed" prevents drift accumulation across
        # frames.
        if period > 0.0:
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
            continue

        frame_write_time = time.time()
        ser.write(img_data)
        frame_write_times.append(frame_write_time)
        frame_deadline_times.append(deadline)
        frame_queue.put(
            FrameItem(small_img.copy(), left_img.copy(), frame_write_time, frame_number)
        )

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
        left_img = item.left_img

        # Read header
        header_bytes = ser.read(struct.calcsize(HEADER_FMT))
        if len(header_bytes) < struct.calcsize(HEADER_FMT):
            print("Reader: timeout waiting for header")
            print("")
            frame_buffer_sem.release()
            continue
            # error_event.set()
            # break

        magic, length = struct.unpack(HEADER_FMT, header_bytes)
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
