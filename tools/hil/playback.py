"""Playback functionality for recorded HIL test frames.

This module provides functionality to replay recorded frames with optical flow
visualization. Supports both fixed-delay and realtime playback modes, with
optional frame saving to disk.
"""

import os
from typing import List, Optional

import cv2

from hil.frames import FrameRecord, IMG_SCALE_SIZE


def playback_recorded_frames(
    recorded_frames: List[FrameRecord],
    playback_delay_ms: Optional[int],
    playback_realtime: bool,
    save_dir: Optional[str] = None,
) -> None:
    """Replay recorded frames with optical flow visualization.

    Parameters
    ----------
    recorded_frames : list of FrameRecord
        The frames to replay, in chronological order.
    playback_delay_ms : int or None
        Fixed delay in milliseconds between frames (used when playback_realtime is False).
        Ignored if playback_realtime is True.
    playback_realtime : bool
        If True, use original inter-frame timings; if False, use playback_delay_ms.
    save_dir : str or None
        Directory path where annotated big images should be saved.
        If None, frames are only displayed (not saved).
    """
    print("")
    if playback_realtime:
        print(
            f"Starting realtime playback of {len(recorded_frames)} frames (original timings)..."
        )
    else:
        print(
            f"Starting playback of {len(recorded_frames)} frames (delay={playback_delay_ms}ms)..."
        )

    # Create the save directory once before the loop (if requested).
    if save_dir is not None:
        os.makedirs(save_dir, exist_ok=True)
        print(f"Saving annotated big images to: {save_dir}")

    for idx, frame in enumerate(recorded_frames):
        if playback_realtime:
            if idx + 1 < len(recorded_frames):
                frame_delay_ms = int(
                    (recorded_frames[idx + 1].timestamp - frame.timestamp) * 1000
                )
            else:
                frame_delay_ms = 1  # last frame: just wait 1ms
        else:
            frame_delay_ms = playback_delay_ms  # type: ignore[assignment]

        small_annotated = cv2.cvtColor(frame.small_img.copy(), cv2.COLOR_GRAY2BGR)
        big_annotated = cv2.cvtColor(frame.left_img.copy(), cv2.COLOR_GRAY2BGR)

        scale_x = frame.left_img.shape[1] / IMG_SCALE_SIZE[0]
        scale_y = frame.left_img.shape[0] / IMG_SCALE_SIZE[1]

        # ------------------------------------------------------------------
        # Overlay optical-flow vectors on the small (96×96) image.
        #
        # The 11×11 grid of vectors is laid out as follows:
        #   - Column origins: x = 8, 16, 24, …, 88  (start=8, step=8)
        #   - Row    origins: y = 8, 16, 24, …, 88  (start=8, step=8)
        #
        # Each Coordinate (u, v) is the optical-flow displacement at that
        # grid point.  An arrow is drawn from (gx, gy) to (gx+u, gy+v).
        # ------------------------------------------------------------------
        GRID_START = 8
        GRID_STEP = 8
        GRID_COLS = 11
        GRID_ROWS = 11

        if frame.coords:
            for row_idx in range(GRID_ROWS):
                for col_idx in range(GRID_COLS):
                    coord = frame.coords[row_idx * GRID_COLS + col_idx]
                    gx = GRID_START + col_idx * GRID_STEP  # 8, 16, …, 88
                    gy = GRID_START + row_idx * GRID_STEP  # 8, 16, …, 88
                    ex = gx + coord.u
                    ey = gy + coord.v
                    color = (0, 255, 0)
                    if not coord.valid:
                        color = (0, 0, 255)

                    cv2.arrowedLine(
                        small_annotated,
                        (gx, gy),
                        (ex, ey),
                        color,  # green arrow
                        1,
                        tipLength=0.4,
                    )

                    # --------------------------------------------------
                    # Same arrow on the big (full-resolution) image.
                    # Scale both origins and displacements by the ratio
                    # between the big image and the 96×96 small image.
                    # --------------------------------------------------
                    gx_big = int(gx * scale_x)
                    gy_big = int(gy * scale_y)
                    ex_big = int((gx + coord.u) * scale_x)
                    ey_big = int((gy + coord.v) * scale_y)
                    cv2.arrowedLine(
                        big_annotated,
                        (gx_big, gy_big),
                        (ex_big, ey_big),
                        color,  # green arrow
                        max(1, int(scale_x)),
                        tipLength=0.3,
                    )

        # Resize big image for display (max 800px width)
        display_scale = min(1.0, 800.0 / big_annotated.shape[1])
        if display_scale < 1.0:
            big_display = cv2.resize(
                big_annotated,
                None,
                fx=display_scale,
                fy=display_scale,
                interpolation=cv2.INTER_AREA,
            )
        else:
            big_display = big_annotated

        cv2.imshow("Left", big_display)
        cv2.imshow("Small left", small_annotated)

        # Save the annotated big image to disk (if --save-dir was given).
        if save_dir is not None:
            filename = os.path.join(save_dir, f"frame_{idx:06d}.png")
            cv2.imwrite(filename, big_annotated)

        key = cv2.waitKey(frame_delay_ms)
        if key == ord("q"):  # press Q to quit
            break

    cv2.destroyAllWindows()
