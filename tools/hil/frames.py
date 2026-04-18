from dataclasses import dataclass

import cv2
import numpy as np

from hil.protocol import Metadata

# ---------------------------------------------------------------------------
# Image scaling constants – single source of truth for the downscaled size
# ---------------------------------------------------------------------------

IMG_SCALE_SIZE: tuple[int, int] = (96, 96)


def scale_image(img: np.ndarray) -> np.ndarray:
    """Resize *img* to :data:`IMG_SCALE_SIZE` using area interpolation."""
    return cv2.resize(img, IMG_SCALE_SIZE, interpolation=cv2.INTER_AREA)


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
