from dataclasses import dataclass

# STM32 binary protocol constants
MAGIC = 0xABCD
PACKET_SIZE = 64  # USB FS CDC max OUT packet size (matches firmware PACKET_SIZE)
# PacketHeader: magic(2) + length(2) + input_metadata(28) + calibration(24) = 56 bytes
# Includes: dt, position, orientation, and camera calibration (fx, fy, cx, cy, k1, k2)
HEADER_FMT = "<HHfffffffffffff"  # magic, length, dt, p_x, p_y, p_z, roll, pitch, yaw, fx, fy, cx, cy, k1, k2
INPUT_METADATA_SIZE = 52  # Header size minus magic/length: 56 - 4 = 52
IMAGE_SIZE = 9216  # 96x96 grayscale image
APP_RX_BUFFER_SIZE = (
    INPUT_METADATA_SIZE + IMAGE_SIZE  # 52 + 9216 = 9268 (matches firmware)
)
# Output metadata received FROM MCU
METADATA_FMT = "<IhhHfffff"
# Each Coordinate is two signed int16_t fields (row=u, col=v) plus one uint8_t bool
# (valid = min_sad < SAD_CEILING) as packed by the firmware.
COORD_FMT = "<hhB"
NUM_COORDS = 121  # 11 columns × 11 rows


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
    """Optical-flow vector at a single grid point.

    ``u`` is the horizontal displacement (x direction).
    ``v`` is the vertical displacement (y direction).

    These correspond to the firmware's ``u = search_index.col`` and
    ``v = search_index.row`` values packed as ``{int16_t row=u; int16_t col=v}``.
    """

    u: int  # horizontal (x) displacement  — firmware field: row
    v: int  # vertical   (y) displacement  — firmware field: col
    valid: bool = False  # True when min_sad < SAD_CEILING (point taken into account)
