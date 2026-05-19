from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

import numpy as np

from hil.protocol import Metadata, Coordinate

# ---------------------------------------------------------------------------
# Image scaling constants – single source of truth for the downscaled size
# ---------------------------------------------------------------------------

IMG_SCALE_SIZE: tuple[int, int] = (96, 96)


@dataclass
class FrameWriteEvent:
    """A record of frame write."""

    write_time: float
    deadline: float
    missed: bool


@dataclass
class FrameReadEvent:
    """A record of frame read."""

    read_time: float
    image: np.ndarray
    payload: bytes
    frame_number: int
    meta: Metadata
    dt: float
    px: float
    py: float
    pz: float
    fx: float
    fy: float
    coords: List[Coordinate] = field(default_factory=lambda: [])


@dataclass
class FrameItem:
    """Data pushed into the inter-thread queue by the writer for each sent frame."""

    image: np.ndarray
    frame_number: int
    dt: float
    px: float
    py: float
    pz: float
    fx: float
    fy: float
