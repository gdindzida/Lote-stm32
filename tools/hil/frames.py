from dataclasses import dataclass

import numpy as np

from hil.protocol import Metadata


@dataclass
class FrameRecord:
    """A fully recorded frame: images + STM32 response payload and metadata."""

    small_img: np.ndarray
    left_img: np.ndarray
    payload: bytes
    meta: Metadata
    meta_size: int
    timestamp: float = 0.0


@dataclass
class FrameItem:
    """Data pushed into the inter-thread queue by the writer for each sent frame."""

    small_img: np.ndarray
    left_img: np.ndarray
    write_time: float
    frame_number: int
