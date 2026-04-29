import os
from glob import glob
from typing import Optional, Tuple, Any

import cv2
import numpy as np

from hil.streamer import DatasetStreamer, DatasetStreamerAdapter


class UAVStreamer(DatasetStreamer):
    def __init__(
        self,
        data_root: str,
        dataset_streamer_adapter: DatasetStreamerAdapter | None,
    ) -> None:
        """data_root: one of the UAV split folders (e.g. Train/, Val/).

        Expected structure:
            <data_root>/query_images/*.png          – query (left) images
            <data_root>/reference_images/offset_0_None/*.png – reference (right) images
        """
        self.query_folder: str = os.path.join(data_root, "rectified_img")
        self.reference_folder: str = os.path.join(
            data_root, "reference_images", "offset_0_None"
        )

        print("cwd: ", os.getcwd())
        print("query folder   : ", self.query_folder)
        print("reference folder: ", self.reference_folder)

        if not os.path.isdir(self.query_folder):
            raise FileNotFoundError(
                f"Query image folder not found: {self.query_folder}\n"
                f"Expected structure: <data_root>/query_images/*.png"
            )
        if not os.path.isdir(self.reference_folder):
            raise FileNotFoundError(
                f"Reference image folder not found: {self.reference_folder}\n"
                f"Expected structure: <data_root>/reference_images/offset_0_None/*.png"
            )

        self.query_images: list[str] = sorted(
            glob(os.path.join(self.query_folder, "*.png"))
        )
        self.reference_images: list[str] = sorted(
            glob(os.path.join(self.reference_folder, "*.png"))
        )

        if len(self.query_images) == 0:
            raise ValueError(
                f"No PNG images found in {self.query_folder}\n"
                f"Check that --data-root points to a UAV split folder "
                f"(the one that directly contains query_images/)."
            )

        # Reference images are spaced at a lower rate than query images
        # (ratio ~3.6:1).  We keep the lengths independent and use modulo
        # indexing for the reference so we never run out of reference frames.
        print("Found ", len(self.query_images), " query images")
        print("Found ", len(self.reference_images), " reference images")

        self.index: int = 0
        self.total: int = len(self.query_images)

        self.dataset_streamer_adapter: DatasetStreamerAdapter | None = (
            dataset_streamer_adapter
        )

    def reset(self) -> None:
        self.index = 0

    def has_next(self) -> bool:
        return self.index < self.total

    def next(
        self,
    ) -> Optional[Tuple[np.ndarray[Any, Any] | None, np.ndarray[Any, Any] | None]]:
        if not self.has_next():
            return None

        img_query: np.ndarray | None = cv2.imread(
            self.query_images[self.index], cv2.IMREAD_GRAYSCALE
        )

        # Wrap reference index so it always has a matching frame
        ref_index = self.index % len(self.reference_images)
        img_reference: np.ndarray | None = cv2.imread(
            self.reference_images[ref_index], cv2.IMREAD_GRAYSCALE
        )

        self.index += 1

        return img_query, img_reference

    def run(self) -> None:
        """Runs stream in given frequency in Hz."""
        if self.dataset_streamer_adapter is None:
            print("Error: Dataset streamer adapter is None.")
            return

        while self.has_next():
            result = self.next()
            if result is None:
                print("Images are None!")
                continue

            query_img, reference_img = result

            if query_img is None:
                print("Query image is None!")
                continue

            if reference_img is None:
                print("Reference image is None!")
                continue

            self.dataset_streamer_adapter.process((query_img, reference_img))
