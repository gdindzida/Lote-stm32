"""IndoorStreamer: stream frames from the INSANE indoor nav-cam dataset.

Expected dataset layout
-----------------------
    <data_root>/img/<n>.png                – grayscale camera frames (1-based)
    <data_root>/nav_cam_timestamps.csv     – per-frame Unix timestamps

The timestamps CSV uses the header::

    #img_no, t[ns], filename

where ``t`` values are floating-point **Unix epoch seconds** (the ``[ns]``
label in the header is misleading).

Since the INSANE indoor dataset has a single camera stream (no separate
reference/database images), ``next()`` returns the same frame as both the
query (left) and reference (right) image.  The HIL writer thread only
consumes the left image, so this is safe.

Images are still rescaled to 96×96 inside the writer thread via
``frames.scale_image()`` — no change required there.
"""

from __future__ import annotations

import os
from glob import glob
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np

from hil.streamer import DatasetStreamer, DatasetStreamerAdapter


class IndoorStreamer(DatasetStreamer):
    """Stream grayscale frames from an INSANE indoor nav-cam folder.

    Parameters
    ----------
    data_root:
        Path to the nav-cam dataset directory, e.g.
        ``/path/to/insane-dataset/indoor_1_nav_cam``.
        Must contain an ``img/`` sub-directory with ``<n>.png`` files and a
        ``nav_cam_timestamps.csv`` file.
    dataset_streamer_adapter:
        Optional adapter called by :meth:`run`.  May be ``None`` when the
        streamer is driven externally (e.g. by the HIL writer thread).
    """

    def __init__(
        self,
        data_root: str,
        dataset_streamer_adapter: DatasetStreamerAdapter | None,
        start_frame: int = 1100,
    ) -> None:
        self.data_root = data_root
        self.img_folder: str = os.path.join(data_root, "img")
        self.timestamps_csv: str = os.path.join(data_root, "nav_cam_timestamps.csv")

        print("cwd            : ", os.getcwd())
        print("img folder     : ", self.img_folder)
        print("timestamps csv : ", self.timestamps_csv)

        if not os.path.isdir(self.img_folder):
            raise FileNotFoundError(
                f"Image folder not found: {self.img_folder}\n"
                f"Expected structure: <data_root>/img/<n>.png"
            )
        if not os.path.isfile(self.timestamps_csv):
            raise FileNotFoundError(
                f"Timestamps CSV not found: {self.timestamps_csv}\n"
                f"Expected: <data_root>/nav_cam_timestamps.csv"
            )

        # Sort image paths numerically by stem (1, 2, 3, …) so that
        # glob's lexicographic order (1, 10, 100, …) does not scramble the
        # sequence.  The start_frame parameter is 1-based (matching the
        # dataset's image numbering) and clipped to valid range.
        all_pngs: List[str] = glob(os.path.join(self.img_folder, "*.png"))
        if not all_pngs:
            raise ValueError(
                f"No PNG images found in {self.img_folder}\n"
                f"Check that --data-root points to the nav-cam folder."
            )
        self.image_paths: List[str] = sorted(
            all_pngs, key=lambda p: int(os.path.splitext(os.path.basename(p))[0])
        )

        # Load timestamps (one per frame, aligned with image_paths by row order).
        self.timestamps: np.ndarray = self._load_timestamps()

        if len(self.timestamps) != len(self.image_paths):
            raise ValueError(
                f"Image count ({len(self.image_paths)}) does not match "
                f"timestamp count ({len(self.timestamps)}) in "
                f"{self.timestamps_csv}."
            )

        n_images = len(self.image_paths)
        # Convert 1-based start_frame to a 0-based list index and clamp.
        start_index: int = max(0, min(start_frame - 1, n_images - 1))
        if start_index > 0:
            print(f"Skipping to frame {start_frame} (index {start_index})")

        print(
            f"Found {n_images} images total, streaming {n_images - start_index} frames"
        )
        print(
            f"Timestamp range: {self.timestamps[start_index]:.3f} → {self.timestamps[-1]:.3f} s"
        )

        self.index: int = start_index
        self.total: int = n_images
        self.dataset_streamer_adapter: DatasetStreamerAdapter | None = (
            dataset_streamer_adapter
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _load_timestamps(self) -> np.ndarray:
        """Parse ``nav_cam_timestamps.csv`` and return a float64 array.

        The CSV format is::

            #img_no, t[ns], filename
            1, 1614007545.846..., 1
            ...

        The first column is a 1-based image number; the second is the
        Unix epoch timestamp in **seconds** (the ``[ns]`` label is
        misleading); the third is the bare filename stem used to locate
        the PNG.  Rows are expected to be sorted by ``img_no``.
        """
        timestamps: List[float] = []
        with open(self.timestamps_csv, newline="") as fh:
            for raw_line in fh:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = [p.strip() for p in line.split(",")]
                if len(parts) < 2:
                    continue
                try:
                    timestamps.append(float(parts[1]))
                except ValueError:
                    continue  # skip malformed rows
        return np.array(timestamps, dtype=np.float64)

    # ------------------------------------------------------------------
    # DatasetStreamer interface
    # ------------------------------------------------------------------

    def reset(self) -> None:
        self.index = 0

    def has_next(self) -> bool:
        return self.index < self.total

    def next(
        self,
    ) -> Optional[Tuple["np.ndarray[Any, Any] | None", "np.ndarray[Any, Any] | None"]]:
        """Return ``(query_img, reference_img)`` for the current frame.

        Both elements are the **same** grayscale image because the indoor
        dataset does not have a separate reference stream.  The HIL writer
        thread only uses the left/query image anyway.

        Returns ``None`` when the dataset is exhausted.
        """
        if not self.has_next():
            return None

        img: np.ndarray | None = cv2.imread(
            self.image_paths[self.index], cv2.IMREAD_GRAYSCALE
        )
        self.index += 1
        # Return same frame for both query and reference slots.
        return img, img

    def run(self) -> None:
        """Run the full stream through the adapter (if set)."""
        if self.dataset_streamer_adapter is None:
            print("Error: Dataset streamer adapter is None.")
            return

        while self.has_next():
            result = self.next()
            if result is None:
                print("Image is None!")
                continue

            query_img, reference_img = result

            if query_img is None:
                print("Query image is None!")
                continue

            self.dataset_streamer_adapter.process((query_img, reference_img))
