"""CsvDatasetStreamer: stream frames from a dataset.csv file.

Expected dataset layout
-----------------------
    dataset.csv with columns:
        - timestamp_cam       : camera timestamp (s)
        - timestamp_sensor    : matched sensor timestamp (s)
        - delta_t             : timestamp_cam - timestamp_sensor (s)
        - p_x, p_y, p_z       : position (m)
        - q_w, q_x, q_y, q_z  : quaternion
        - roll, pitch, yaw    : euler angles (rad)
        - image_path          : relative path to rectified image

Images are rescaled to 96×96 inside the writer thread via
``frames.scale_image()`` — no change required there.
"""

from __future__ import annotations

import csv
import os
from typing import Any, List, Optional, Dict, Tuple

import cv2
import numpy as np


class CsvDatasetStreamer:
    """Stream grayscale frames from a dataset.csv file.

    Parameters
    ----------
    dataset_csv_path:
        Path to the dataset.csv file.
    data_root:
        Root directory for resolving relative image paths in the CSV.
        If None, image paths are assumed to be absolute or relative to cwd.
    dataset_streamer_adapter:
        Optional adapter called by :meth:`run`.  May be ``None`` when the
        streamer is driven externally (e.g. by the HIL writer thread).
    start_frame:
        1-based frame number at which to start streaming. Frames before this
        are skipped.
    """

    def __init__(
        self,
        dataset_csv_path: str,
        data_root: str | None = None,
        dataset_streamer_adapter: Any | None = None,
        start_frame: int = 1,
    ) -> None:
        self.dataset_csv_path = dataset_csv_path
        self.data_root = data_root

        print("cwd              : ", os.getcwd())
        print("dataset CSV      : ", self.dataset_csv_path)
        print("data root        : ", self.data_root)

        if not os.path.isfile(self.dataset_csv_path):
            raise FileNotFoundError(f"Dataset CSV not found: {self.dataset_csv_path}")

        self.entries: List[Dict[str, Any]] = self.load_dataset()

        if not self.entries:
            raise ValueError(
                f"No valid entries found in dataset CSV: {self.dataset_csv_path}"
            )

        n_entries = len(self.entries)
        start_index: int = max(0, min(start_frame, n_entries - 1))
        if start_index > 0:
            print(f"Skipping to frame {start_frame} (index {start_index})")

        print(
            f"Found {n_entries} entries total, streaming {n_entries - start_index} frames"
        )

        # Extract timestamps for range display
        timestamps = [float(e["timestamp_cam"]) for e in self.entries]
        print(
            f"Timestamp range: {timestamps[start_index]:.3f} → {timestamps[-1]:.3f} s"
        )

        self._start_index: int = start_index
        self.index: int = start_index
        self.total: int = n_entries
        self.dataset_streamer_adapter: Any | None = dataset_streamer_adapter

    # ------------------------------------------------------------------
    # Public methods
    # ------------------------------------------------------------------

    def load_dataset(self) -> List[Dict[str, Any]]:
        """Parse dataset.csv and return a list of entry dicts.

        Each dict contains all columns from the CSV, with keys matching
        the column names.
        """
        entries: List[Dict[str, Any]] = []

        with open(self.dataset_csv_path, newline="") as fh:
            reader = csv.DictReader(fh)

            # Verify required columns
            required_cols = {"timestamp_cam", "image_path"}
            if reader.fieldnames is None:
                raise ValueError(f"Dataset CSV has no header: {self.dataset_csv_path}")

            fieldnames_set = set(reader.fieldnames)
            missing = required_cols - fieldnames_set
            if missing:
                raise ValueError(
                    f"Dataset CSV is missing required columns: {missing}\n"
                    f"Found columns: {reader.fieldnames}"
                )

            for row in reader:
                if not row or not row.get("image_path"):
                    continue

                # Verify image file exists
                image_path = row["image_path"]
                if self.data_root:
                    full_path = os.path.join(self.data_root, image_path)
                else:
                    full_path = image_path

                if not os.path.isfile(full_path):
                    print(f"  ⚠  Image not found, skipping: {full_path}")
                    continue

                # Store the entry with the full path for easy loading
                entry: Dict[str, Any] = dict(row)
                entry["_full_image_path"] = full_path
                entries.append(entry)

        return entries

    def get_timestamp(self, index: int) -> float:
        """Return the camera timestamp for the given index.

        Parameters
        ----------
        index : int
            0-based frame index

        Returns
        -------
        float
            Camera timestamp in seconds
        """
        if 0 <= index < self.total:
            return float(self.entries[index]["timestamp_cam"])
        return 0.0

    def reset(self) -> None:
        self.index = 0

    def has_next(self) -> bool:
        return self.index < self.total

    def next(
        self,
    ) -> Tuple[Optional[np.ndarray[Any, Any]], Optional[Dict[str, Any]]]:
        """Return the grayscale image for the current frame.

        Both elements are the **same** grayscale image because we have a
        single image stream. The HIL writer thread only uses the left/query
        image anyway.

        Returns ``None`` when the dataset is exhausted.
        """
        if not self.has_next():
            return None, None

        entry = self.entries[self.index]
        img: np.ndarray | None = cv2.imread(
            entry["_full_image_path"], cv2.IMREAD_GRAYSCALE
        )

        if img is None:
            print(f"  ⚠  Failed to load image: {entry['_full_image_path']}")

        self.index += 1
        # Return same frame for both query and reference slots.
        return img, entry
