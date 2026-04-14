from dataclasses import dataclass

# STM32 binary protocol constants
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
