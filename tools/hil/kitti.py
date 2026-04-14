import os
from glob import glob
from typing import Optional, Tuple, Any

import cv2
import numpy as np

from hil.streamer import DatasetStreamer, DatasetStreamerAdapter


class KittiStreamer(DatasetStreamer):
    def __init__(
        self,
        data_root: str,
        dataset_streamer_adapter: DatasetStreamerAdapter | None,
    ) -> None:
        """data_root: top folder of 2011_09_26 raw KITTI data."""
        self.left_folder: str = os.path.join(data_root, "image_00/data")
        self.right_folder: str = os.path.join(data_root, "image_01/data")

        print("cwd: ", os.getcwd())
        print("left folder : ", self.left_folder)
        print("right folder : ", self.right_folder)

        self.left_images: list[str] = sorted(
            glob(os.path.join(self.left_folder, "*.png"))
        )
        self.right_images: list[str] = sorted(
            glob(os.path.join(self.right_folder, "*.png"))
        )

        if not os.path.isdir(self.left_folder):
            raise FileNotFoundError(
                f"Left image folder not found: {self.left_folder}\n"
                f"Expected structure: <data_root>/image_00/data/*.png"
            )
        if not os.path.isdir(self.right_folder):
            raise FileNotFoundError(
                f"Right image folder not found: {self.right_folder}\n"
                f"Expected structure: <data_root>/image_01/data/*.png"
            )

        print("Found ", len(self.left_images), " images")

        if len(self.left_images) == 0:
            raise ValueError(
                f"No PNG images found in {self.left_folder}\n"
                f"Check that --data-root points to the drive folder "
                f"(the one that directly contains image_00/ and image_01/)."
            )

        if len(self.left_images) != len(self.right_images):
            raise ValueError("Left and right image counts do not match!")

        self.index: int = 0
        self.total: int = len(self.left_images)
        print("Found ", self.total, " images")

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

        img_left: np.ndarray | None = cv2.imread(
            self.left_images[self.index], cv2.IMREAD_GRAYSCALE
        )
        img_right: np.ndarray | None = cv2.imread(
            self.right_images[self.index], cv2.IMREAD_GRAYSCALE
        )

        self.index += 1

        return img_left, img_right

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

            left_img, right_img = result

            if left_img is None:
                print("Left image is None!")
                continue

            if right_img is None:
                print("Right image is None!")
                continue

            self.dataset_streamer_adapter.process((left_img, right_img))
