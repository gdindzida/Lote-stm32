from dataclasses import dataclass

# STM32 binary protocol constants
MAGIC = 0xABCD
PACKET_SIZE = 64
SEND_HEADER_FMT = "<HHffffffffffffff"
RECV_HEADER_FMT = "<HH"
APP_RX_BUFFER_SIZE = 9216
METADATA_FMT = "<IIHfffff"
COORD_FMT = "<hhB"
NUM_COORDS = 121  # 11 columns × 11 rows


@dataclass
class Metadata:
    process_elapsed_time_ms: int
    stride_elapsed_tim_ms: int
    num_points: int
    stack_mem_usage: float
    heap_mem_usage: float
    vx: float
    vy: float
    debug: float


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
