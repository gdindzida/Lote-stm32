from dataclasses import dataclass

# STM32 binary protocol constants
MAGIC = 0xABCD
HEADER_FMT = "<HH"
METADATA_FMT = "<IhhHfffff"
# Each Coordinate is two signed int16_t fields (row=u, col=v) as packed by the firmware.
COORD_FMT = "<hh"
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
