from streamer.kitti.streamer import KittiStreamer
from streamer.base import DatasetStreamer
from streamer.base import DatasetStreamerAdapter
from typing import Optional, Tuple, Any
import numpy as np
import os
import statistics

os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
import cv2
import time
import serial
import serial.tools.list_ports
import struct
from dataclasses import dataclass
from typing import List

MAGIC = 0xABCD
HEADER_FMT = "<HH"
METADATA_FMT = "<IIHff"
COORD_FMT = "BB"


@dataclass
class Metadata:
    process_elapsed_time_ms: int
    sum: int
    num_points: int
    stack_mem_usage: float
    heap_mem_usage: float


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


# def send_image(ser: serial.Serial, img: np.ndarray) -> None:
#     """send raw image bytes."""
#     raw = img.tobytes()  # H×W×C uint8, row-major
#     ser.write(raw)

if __name__ == "__main__":

    data_root: str = "modules/gaspar/data/2011_09_26_drive_0001_sync"
    print("Entering ", data_root)

    streamer: DatasetStreamer = KittiStreamer(data_root, None)

    port = find_stm32_port()
    ser = serial.Serial(port, timeout=10)
    print(f"Connected to {port}")

    elapsed_times: List[float] = []
    process_elapsed_times: List[float] = []
    peak_stack_memory = 0
    peak_heap_memory = 0

    print("Starting KITTI clip playback...")
    while streamer.has_next():

        print("\n")
        print("\n")
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
        print(left_img.shape)
        print(small_img.shape)
        img_data = small_img.tobytes()

        start_time = time.time()
        # Send image
        # print("Sending data...")
        ser.write(img_data)
        # time.sleep(1)

        # Wait for response
        # print("Waiting for header...")
        header_bytes = ser.read(struct.calcsize(HEADER_FMT))
        magic, length = struct.unpack(HEADER_FMT, header_bytes)
        # time.sleep(1)

        if magic != MAGIC:
            raise ValueError(f"Bad magic: {hex(magic)}")

        # print("Waiting for payload= ", length, " bytes")
        payload = ser.read(length)
        # time.sleep(1)

        elapsed_time = time.time() - start_time
        print("Elapsed time: ", elapsed_time)

        meta_size = struct.calcsize(METADATA_FMT)
        meta_raw = struct.unpack(METADATA_FMT, payload[:meta_size])
        meta = Metadata(*meta_raw)

        elapsed_times.append(elapsed_time)
        process_elapsed_times.append(meta.process_elapsed_time_ms)
        peak_stack_memory = meta.stack_mem_usage
        peak_heap_memory = meta.heap_mem_usage

        print("Got sum= ", meta.sum, " for real sum= ", sum(img_data))
        print("Got process process elapsed time(us)= ", meta.process_elapsed_time_ms)
        print("Peak stack mem usage= ", 100 * meta.stack_mem_usage, "%")
        print("Peak stack mem usage= ", 100 * meta.heap_mem_usage, "%")
        small_annotated = cv2.cvtColor(small_img.copy(), cv2.COLOR_GRAY2BGR)
        big_annotated = cv2.cvtColor(left_img.copy(), cv2.COLOR_GRAY2BGR)

        scale_x = left_img.shape[1] / 128.0
        scale_y = left_img.shape[0] / 64.0

        if meta.num_points > 0:
            coord_size = struct.calcsize(COORD_FMT)
            offset = meta_size
            print("Got this many points: ", meta.num_points)
            for i in range(meta.num_points):
                x, y = struct.unpack(COORD_FMT, payload[offset : offset + coord_size])
                offset += coord_size
                x, y = int(x) & 0xFF, int(y) & 0xFF  # interpret as uint8_t (0–255)
                print(f"  point[{i}]: x={y}, y={x}")

                # Draw on small image (coordinates are in small image space)
                cv2.circle(
                    small_annotated, (y, x), radius=2, color=(0, 255, 0), thickness=-1
                )

                # Scale up and draw on big image
                big_x = int(x * scale_y)
                big_y = int(y * scale_x)
                cv2.circle(
                    big_annotated,
                    (big_y, big_x),
                    radius=5,
                    color=(0, 255, 0),
                    thickness=-1,
                )
        else:
            print("No points!")

        cv2.imshow("Left", big_annotated)
        cv2.imshow("Small left", small_annotated)
        key = cv2.waitKey(1)
        if key == ord("q"):  # press Q to quit
            break

    max_elapsed_time = 1000 * max(elapsed_times)
    min_elapsed_time = 1000 * min(elapsed_times)
    avg_elapsed_time = 1000 * sum(elapsed_times) / len(elapsed_times)
    std_elapsed_time = 1000 * statistics.stdev(elapsed_times)

    print("")
    print("Statistics")
    print("")
    print(
        "max elapsed time(ms): ", max_elapsed_time, " f(Hz)= ", 1000 / max_elapsed_time
    )
    print(
        "min elapsed time(ms): ", min_elapsed_time, " f(Hz)= ", 1000 / min_elapsed_time
    )
    print(
        "avg elapsed time(ms): ", avg_elapsed_time, " f(Hz)= ", 1000 / avg_elapsed_time
    )
    print("std elapsed time(ms): ", std_elapsed_time)

    max_process_elapsed_time = 0.001 * max(process_elapsed_times)
    min_process_elapsed_time = 0.001 * min(process_elapsed_times)
    avg_process_elapsed_time = (
        0.001 * sum(process_elapsed_times) / len(process_elapsed_times)
    )
    std_process_elapsed_time = 0.001 * statistics.stdev(process_elapsed_times)

    print("")
    print("max process elapsed time(ms): ", max_process_elapsed_time)
    print("min process elapsed time(ms): ", min_process_elapsed_time)
    print("avg process elapsed time(ms): ", avg_process_elapsed_time)
    print("std process elapsed time(ms): ", std_process_elapsed_time)

    print("")
    print("Peak stack memory usage: ", 100 * peak_stack_memory, "%")
    print("Peak heap memory usage: ", 100 * peak_heap_memory, "%")

    # streamer.run()
    cv2.destroyAllWindows()
